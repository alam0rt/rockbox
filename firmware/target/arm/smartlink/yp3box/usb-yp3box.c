/*
 * USB mass storage for the SL6801: the boot ROM is the device.
 *
 * The ROM contains a complete Bulk-Only Transport / SCSI target - it is what
 * enumerates in download mode - and it is medium-agnostic. It never touches
 * flash or SD itself: every media operation goes through a table of function
 * pointers the running firmware registers with usb_set_userfn (ROM 0x5914,
 * one instruction: store to 0x00801068). So this file is not a USB device
 * driver. It is a clock, a module enable, two pin mux writes, that table, and
 * one SCSI hook.
 *
 * Everything below is transcribed from the boot ROM and from the vendor's
 * usbd_msc_cardreader_init at FIRM 0xd67068. docs/USB.md has the full map.
 *
 * THE ONE TRAP, and it is a big one: the ROM's own READ(10) does not work.
 * Its data-in pump at ROM 0x5bcc calls the media read op, and then overwrites
 * the 512 bytes it just read with a 0,1,2,...255 ramp before sending them
 * (0x5c3a..0x5c46). It is factory test code. The vendor does not use it
 * either - its hook at SRAM 0x8133c4 intercepts opcodes 0x28 and 0x2a and
 * serves them itself - and neither do we. Filling in the read and write ops
 * at +0x10/+0x14 and expecting the ROM to do the rest gets you a disk full of
 * ramps.
 */
#include "config.h"
#include "system.h"
#include "usb.h"
#include "power.h"
#include "storage.h"
#include "kernel.h"
#include "sdmmc.h"
#include <string.h>
#include "sl6801-regs.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

/* --- the ROM's mass-storage state block -----------------------------------
 *
 * One structure at 0x00800de8, below the 0x806000 the linker reserves for the
 * ROM. Offsets recovered from the ROM's own code; the address of each is the
 * routine that establishes it.
 */
#define MSC_BASE            0x00800de8u
#define MSC8(o)             REG8(MSC_BASE + (o))
#define MSC16(o)            REG16(MSC_BASE + (o))
#define MSC32(o)            REG32(MSC_BASE + (o))

#define MSC_STATE           MSC8(0x004)     /* 0x63b8, 0x6380, 0x6454 */
#define MSC_XFER_LEN        MSC16(0x006)    /* 0x5b3c, 0x63b8 */
#define MSC_BUF             ((void *)(MSC_BASE + 0x008))  /* 512 bytes, 0x5c30 */
#define MSC_CDB             ((const uint8_t *)(MSC_BASE + 0x217)) /* 0x62b2 */
#define MSC_BLOCK_SIZE      MSC32(0x25c)    /* 0x5b26 */
#define MSC_BLOCK_COUNT     MSC32(0x260)    /* 0x5b14 */

/* MSC_STATE, from the three completion callbacks that switch on it:
 *   0  idle, waiting for a CBW
 *   1  data-out in flight     (0x6454 continues the command)
 *   2  data-in in flight      (0x6380 continues the command)
 *   3  last chunk in flight; the ROM sends the CSW when it completes
 *   5  failed; the ROM stalls
 * The ROM's own READ(10) sets 2 while chunks remain and 3 on the last one
 * (0x5c76/0x5c82), and that is the protocol this file follows. */
#define MSC_ST_IDLE         0
#define MSC_ST_DATA_OUT     1
#define MSC_ST_DATA_IN      2
#define MSC_ST_LAST         3
#define MSC_ST_FAIL         5

/* Boot ROM entry points, thumb bit included. Local to this file because
 * nothing else has any business calling them. */
#define ROM_USB_SET_USERFN  ((void (*)(const void *))0x5915u)
#define ROM_USB_ATTACH      ((void (*)(void))0x58f3u)
/* send one data-in chunk from a buffer; ROM 0x5930, used by the ROM's own
 * multi-chunk pump at 0x5c4c */
#define ROM_MSC_SEND        ((void (*)(const void *, unsigned))0x5931u)
/* the USB PHY/controller enable the vendor reaches through SRAM thunk
 * 0x80db54, which is one branch to this */
#define ROM_USB_PHY         ((void (*)(unsigned))0x3ad5u)
/* USB pad and PHY configuration: two more pin-mux writes (port 2 pins 0 and
 * 1, mode 2) and a configuration word into the PHY register at 0x40085100,
 * behind a busy bit. See the comment on usb_hw_enable(). */
#define ROM_USB_PHY_INIT    ((void (*)(void))0x3c2du)
#define ROM_CLK_DISABLE     ((void (*)(unsigned))0x2571u)

#define USB_MODULE          0x23u   /* FIRM 0xd66d98 clk_enable, 0xd66c40 off */
#define USB_CLOCK           0x12u   /* FIRM 0xd66da6/0xd66dae, source 9 */
#define USB_CLOCK_SRC       9u
/* FIRM 0xd66dbe/0xd66dc4 push these through the ROM GPIO helper at 0x7ac:
 * port 1 pin 23 and port 1 pin 22, both mode 15 - the USB data pair's mux. */
#define USB_PIN_DP          0x0001bf90u
#define USB_PIN_DM          0x0001b790u
/* FIRM 0xd7ed88, called right after the PHY enable */
#define USB_SYS_ENABLE      REG32(0x40000000)
#define USB_SYS_ENABLE_BIT  0x40u


/* --- media operations ------------------------------------------------------
 *
 * The table the ROM dereferences for every media access. Only the entries the
 * ROM actually calls are filled; the vendor's own table has stubs in the same
 * places (SRAM 0x813365/0x813369/0x8133c1 are all "return 0").
 */
static bool usb_exposed;        /* the host owns the card right now */

/* +0x04, called from READ CAPACITY at ROM 0x5b0c as f(&count, &size), where
 * the two pointers are MSC_BLOCK_COUNT and MSC_BLOCK_SIZE themselves. The ROM
 * reads the size back as a full word, so write a word, not the halfword the
 * vendor's version writes. */
static int usb_msc_capacity(uint32_t *block_count, uint32_t *block_size)
{
    tCardInfo *card = card_get_info_target(0);

    if (!card || card->numblocks == 0)
        return -1;

    *block_count = card->numblocks;
    *block_size = 512;
    return 0;
}

/* +0x08, medium present and ready. Non-zero is ready (ROM 0x5cee, 0x5fa6). */
static int usb_msc_ready(void)
{
    tCardInfo *card = card_get_info_target(0);

    return (card && card->initialized > 0 && card->numblocks != 0) ? 1 : 0;
}

/* +0x0c, write protect. 1 means protected: ROM 0x5fb0 turns that into sense
 * 0x27, WRITE PROTECTED. */
static int usb_msc_write_protected(void)
{
    return 0;
}

/* +0x00, +0x10, +0x14, +0x18, +0x1c: the ROM calls +0x10 and +0x14 only from
 * its own READ(10)/WRITE(10) pumps, which the hook below takes over, so these
 * are never reached. The vendor's are stubs for the same reason. */
static int usb_msc_stub(void)
{
    return 0;
}

/* --- the SCSI hook ---------------------------------------------------------
 *
 * +0x20 gets first crack at every opcode. ROM 0x631c:
 *
 *     r0 = CDB[0]; if (hook(r0) != 0) return 0;   // handled
 *
 * and it is re-entered once per chunk, because the data-phase completion
 * callbacks re-dispatch: ROM 0x6380 (data-in) and 0x6454 (data-out) both call
 * the dispatcher again while MSC_STATE says a transfer is in progress. So the
 * hook is a per-chunk state machine, exactly like the ROM's own pump, and it
 * keeps its position in these two variables.
 */
static uint32_t msc_lba;
static uint32_t msc_blocks_left;

/* Decide whether to take the command, and set up if so.
 *
 * Every reason NOT to take it is delegated rather than reported, because the
 * hook cannot report a failure: its return value means "handled", so a
 * negative return still tells ROM 0x631c the command succeeded, and no CHECK
 * CONDITION is ever sent. Returning false instead lets the ROM's own
 * READ(10)/WRITE(10) run, and those fail correctly - not ready is sense 2/3a
 * at ROM 0x5ce4, out of range is sense 5/21 at 0x5ca6 - without ever reaching
 * the ramp, which is downstream of both checks. The vendor's hook delegates
 * the not-ready case the same way (SRAM 0x813404). */
static bool usb_msc_begin(void)
{
    const uint8_t *cdb = MSC_CDB;
    tCardInfo *card = card_get_info_target(0);

    if (!usb_msc_ready())
        return false;

    /* CDB[2..5] big-endian LBA, CDB[7..8] big-endian block count. */
    msc_lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16)
            | ((uint32_t)cdb[4] << 8)  |  (uint32_t)cdb[5];
    msc_blocks_left = ((uint32_t)cdb[7] << 8) | cdb[8];

    /* Zero length is legal and means no data phase; the ROM handles that
     * ending correctly and we would not. */
    if (msc_blocks_left == 0)
        return false;

    if (msc_lba > card->numblocks
            || msc_blocks_left > card->numblocks - msc_lba)
        return false;

    return true;
}

/* One 512-byte chunk per call, which is what the ROM's pump does and what the
 * single staging buffer allows. */
static void usb_msc_read_chunk(void)
{
    if (sd_read_sectors(IF_MD(0,) msc_lba, 1, MSC_BUF) != 0) {
        /* Nothing here can fail the command either: by this point the data
         * phase is in flight and the only paths out of ROM 0x6380 are "send
         * the next chunk" and "send the CSW". Sending a zeroed block keeps
         * the transfer counted and the bus alive; wedging it would need the
         * host to time out. The log line is the real report. */
        logf("usb: read error at lba %lu", (unsigned long)msc_lba);
        memset(MSC_BUF, 0, 512);
    }

    msc_lba++;
    msc_blocks_left--;

    /* State 3 tells the ROM this is the last chunk: when it completes, ROM
     * 0x6380 falls into its state 3..4 arm and sends the CSW itself. */
    MSC_STATE = msc_blocks_left ? MSC_ST_DATA_IN : MSC_ST_LAST;
    MSC_XFER_LEN = 512;
    ROM_MSC_SEND(MSC_BUF, 512);
}

/* The data-out completion at ROM 0x6454 has already re-armed the receive into
 * the staging buffer before it re-dispatches, so on entry the block the host
 * sent is sitting in MSC_BUF. */
static void usb_msc_write_chunk(void)
{
    if (sd_write_sectors(IF_MD(0,) msc_lba, 1, MSC_BUF) != 0)
        logf("usb: write error at lba %lu", (unsigned long)msc_lba);

    msc_lba++;
    msc_blocks_left--;

    MSC_STATE = msc_blocks_left ? MSC_ST_DATA_OUT : MSC_ST_LAST;
}

static int usb_msc_hook(unsigned opcode)
{
    if (opcode != 0x28 && opcode != 0x2a)
        return 0;                       /* the ROM's own handlers are fine */

    if (!usb_exposed)
        return 0;

    if (MSC_STATE == MSC_ST_IDLE) {
        if (!usb_msc_begin())
            return 0;                   /* let the ROM fail it properly */

        MSC_STATE = (opcode == 0x28) ? MSC_ST_DATA_IN : MSC_ST_DATA_OUT;

        if (opcode == 0x2a) {
            /* The first block has not arrived yet: the ROM armed the receive
             * when it accepted the CBW, and 0x6454 will re-dispatch us once
             * it lands. Nothing to do on this pass. */
            return 1;
        }
    }

    if (opcode == 0x28)
        usb_msc_read_chunk();
    else
        usb_msc_write_chunk();

    return 1;
}

/* The table itself, typed rather than an array of void* so a wrong signature
 * is a compile error. It must be in .data, not .rodata: .rodata is SPI NOR
 * and the ROM dereferences this from interrupt context. */
struct usb_msc_ops {
    int (*unused0)(void);                               /* +0x00 */
    int (*capacity)(uint32_t *count, uint32_t *size);   /* +0x04 */
    int (*ready)(void);                                 /* +0x08 */
    int (*write_protected)(void);                       /* +0x0c */
    int (*read)(void *buf, uint32_t lba, uint32_t n);   /* +0x10 */
    int (*write)(void *buf, uint32_t lba, uint32_t n);  /* +0x14 */
    int (*unused18)(void);                              /* +0x18 */
    int (*unused1c)(void);                              /* +0x1c */
    int (*hook)(unsigned opcode);                       /* +0x20 */
};

static int usb_msc_rw_unused(void *buf, uint32_t lba, uint32_t n)
{
    (void)buf; (void)lba; (void)n;
    return 0;
}

static struct usb_msc_ops usb_msc_ops = {
    .unused0         = usb_msc_stub,
    .capacity        = usb_msc_capacity,
    .ready           = usb_msc_ready,
    .write_protected = usb_msc_write_protected,
    .read            = usb_msc_rw_unused,   /* never reached, see file header */
    .write           = usb_msc_rw_unused,
    .unused18        = usb_msc_stub,
    .unused1c        = usb_msc_stub,
    .hook            = usb_msc_hook,
};


/* --- bring-up --------------------------------------------------------------
 *
 * driver_usbd_params_init, FIRM 0xd66d98 onwards. Module enable AND a
 * committed clock, as always on this SoC, and .icode because it commits one.
 */
static void __attribute__((section(".icode"), noinline))
usb_hw_enable(void)
{
    /* The PHY and its pads first.
     *
     * This is the step the vendor's driver_usbd_params_init does NOT do, and
     * the reason is the oldest rule this port has: our boot path skips the
     * vendor bootloader entirely, so everything it would normally set up is
     * ours to do. FIRM inherits a configured USB PHY; we inherit nothing.
     *
     * The boot ROM's own USB bring-up at 0x4c034 - the one that enumerates in
     * download mode, the known-good reference - opens with three calls before
     * the module enable. Two of them, 0x3cd4 and 0x3b98, reconfigure the
     * whole clock tree onto sources 15 and 10; those are download mode
     * setting up the machine and must not be repeated here, or the CPU clock
     * moves under us. The third, 0x3c2c, is USB-specific and is this:
     *
     *     gpio_cfg(0x00020100)          port 2 pin 0, mode 2
     *     gpio_cfg(0x0002090b)          port 2 pin 1, mode 2
     *     [0x40085100] |= 0x80000000;   start, then poll until it clears
     *     [0x40085100] = 0x0001b082;    the PHY configuration word
     *
     * Called rather than transcribed: it is mask ROM, it takes no arguments,
     * and it touches nothing else. */
    ROM_USB_PHY_INIT();

    /* Unconditionally, as the vendor does at FIRM 0xd66d98 - it calls ROM
     * 0x2564 straight, with no is-it-already-on check. */
    ROM_CLK_ENABLE(USB_MODULE);
    /* The vendor's delay(2000) here is ROM 0xb730, a raw three-instruction
     * spin, so about 30us at 192 MHz - not a millisecond delay. 2 ms is
     * generous and costs nothing on this path. */
    udelay(2000);                       /* FIRM 0xd66d9e */

    ROM_CLK_SET_SRC(USB_CLOCK, USB_CLOCK_SRC);
    ROM_CLK_APPLY(USB_CLOCK);

    ROM_USB_PHY(1);                     /* FIRM 0xd66db4 via thunk 0x80db54 */
    USB_SYS_ENABLE |= USB_SYS_ENABLE_BIT;   /* FIRM 0xd7ed88 */

    ROM_GPIO_CFG1(USB_PIN_DP);          /* FIRM 0xd66dbe */
    ROM_GPIO_CFG1(USB_PIN_DM);          /* FIRM 0xd66dc4 */
}

/* Teardown, FIRM 0xd66c3a: stop the clock, then drop the module. */
static void __attribute__((section(".icode"), noinline))
usb_hw_disable(void)
{
    ROM_CLK_STOP(USB_CLOCK);
    ROM_CLK_DISABLE(USB_MODULE);
}


/* --- the Rockbox side ------------------------------------------------------ */

void usb_init_device(void)
{
    usb_exposed = false;
}

/* usb.c's non-usbstack slave mode has already unmounted the disk, reset
 * storage and called storage_enable(false) by the time this runs, so the card
 * is ours to hand over. */
/* The controller registers, for the probe below. The ROM's USB code reaches
 * 0x40040001 (0x5204), 0x4004000b (0x5572), 0x4004000e and 0x40040012
 * (0x5118..0x5136) - the same page pcm-yp3box.c writes for audio DMA, which
 * is worth knowing but is not in play here: nothing plays during USB mode. */
#define USBC(o)             REG8(0x40040000u + (o))

#ifdef ROCKBOX_HAS_LOGF
static void usb_log_state(const char *when)
{
    logf("usb %s: mod23=%d clk sys40000000=%08lx", when,
         ROM_CLK_IS_ON(USB_MODULE), (unsigned long)USB_SYS_ENABLE);
    logf("usb %s: 00=%02x 01=%02x 0b=%02x 0e=%02x 12=%04x", when,
         USBC(0x00), USBC(0x01), USBC(0x0b), USBC(0x0e),
         REG16(0x40040012));
    logf("usb %s: phy 40085100=%08lx", when,
         (unsigned long)REG32(0x40085100));
}

/* Does the register file answer at all? The failure signatures differ:
 * a module that is off reads back zero, a module that is on with no committed
 * clock holds values but never transfers. Write a pattern into the endpoint
 * index register, read it back, put it back. */
static void usb_probe_regs(void)
{
    uint8_t saved = USBC(0x0e);

    USBC(0x0e) = 0x55;
    logf("usb probe: wrote 55 to 0e, read %02x", USBC(0x0e));
    USBC(0x0e) = 0x2a;
    logf("usb probe: wrote 2a to 0e, read %02x", USBC(0x0e));
    USBC(0x0e) = saved;
}
#else
/* Both exist only to feed the log; without it they would be register pokes
 * for nothing. */
static inline void usb_log_state(const char *when) { (void)when; }
static inline void usb_probe_regs(void) { }
#endif

void usb_enable(bool on)
{
    logf("usb_enable(%d)", on);

    if (on) {
        if (usb_exposed)
            return;

        msc_lba = 0;
        msc_blocks_left = 0;
        MSC_STATE = MSC_ST_IDLE;

        usb_log_state("pre");
        usb_hw_enable();
        usb_log_state("post-clk");
        usb_probe_regs();

        usb_exposed = true;
        ROM_USB_SET_USERFN(&usb_msc_ops);
        ROM_USB_ATTACH();
        usb_log_state("post-attach");
        logf("usb: vtor=%08lx irq41=%08lx irq42=%08lx",
             (unsigned long)REG32(0xE000ED08),
             (unsigned long)REG32(0x00800000 + (41 + 16) * 4),
             (unsigned long)REG32(0x00800000 + (42 + 16) * 4));
        logf("usb: iser1=%08lx userfn=%08lx",
             (unsigned long)REG32(0xE000E104),
             (unsigned long)REG32(0x00801068));

        /* The watchdog is the only reset the boot ROM has - there is no
         * SYSRESETREQ literal anywhere in it - so if the device restarts a
         * few seconds into USB mode, this is what did it. system_init()
         * disables it once at boot; log what it looks like now, and put it
         * back down in case something on the USB path re-armed it. */
        logf("usb: wdog ctrl=%08lx load=%08lx",
             (unsigned long)WDOG_CTRL, (unsigned long)WDOG_LOAD);
        WDOG_CTRL = 0;
    } else {
        if (!usb_exposed)
            return;

        usb_exposed = false;
        ROM_USB_SET_USERFN(NULL);
        usb_hw_disable();
    }
}

void usb_attach(void)
{
    usb_enable(true);
}

/* The PMU reports VBUS; power_input_status() already decodes the bits the
 * vendor's usb manager watches. This cannot tell a charger from a host - the
 * device has one USB input and one status bit - so a charger looks like a
 * connection until the host fails to enumerate. */
int usb_detect(void)
{
    /* usb_tick() calls this every tick, from the tick interrupt, and each
     * call is a PMU mailbox transaction. At HZ that is 100 transactions a
     * second taken out of an interrupt, contending with the power thread for
     * one register pair. Sample at 10 Hz and hand back the last answer in
     * between; usb.c debounces over 200 ms anyway. */
    static long next_poll;
    static int detected = USB_EXTRACTED;

    if (TIME_AFTER(current_tick, next_poll)) {
        next_poll = current_tick + HZ / 10;
        detected = (power_input_status() != POWER_INPUT_NONE)
                 ? USB_INSERTED : USB_EXTRACTED;
    }

    return detected;
}
