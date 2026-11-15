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
#include "yp3-buildstamp.h"

#ifndef YP3_RIG_SECONDS
#define YP3_RIG_SECONDS 0
#endif
/* YP3_RIG_USB=1 lets a rig build see VBUS after all, so a self-rebooting
 * cycle can exercise Rockbox's own mass storage - and read the card's
 * blackbox files through it - instead of hiding the cable. */
#ifndef YP3_RIG_USB
#define YP3_RIG_USB 0
#endif
/* The card is exposed READ-ONLY unless this is 1. The write path below has
 * never written a correct sector on hardware: on 2026-11-09 a single host
 * rw mount stored a WRITE(10) CBW over FAT1 sector 0 and broke the card
 * (yp3-0gn.11, root cause yp3-0gn.1). Desktops automount rw, so a writable
 * default is a data-loss default. Flip it only for a build that is testing
 * the write fix. */
#ifndef YP3_USB_WRITABLE
#define YP3_USB_WRITABLE 0
#endif
/* YP3_USB_WRITE_FIX=0 builds the old WRITE(10) path, which never arms the
 * data-out receive and hangs the host on the first write (see the comment
 * above usb_msc_write_start). It exists only so the diagnosis can be checked
 * on hardware with the same breadcrumbs; 1 is the default and the fix. */
#ifndef YP3_USB_WRITE_FIX
#define YP3_USB_WRITE_FIX 1
#endif
/* YP3_USB_WRITE_DMA=1 receives WRITE(10) data with the MUSB DMA, a run at a
 * time, instead of the ROM's one-64-byte-packet pump. See usb_msc_dma_cb. */
#ifndef YP3_USB_WRITE_DMA
#define YP3_USB_WRITE_DMA 1
#endif
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

/* The mass-storage hook runs once per 64-byte packet on a write and once per
 * sector on a read, inside the USB interrupt. From flash that cost 148 us a
 * packet on top of the SD time (2026-11-15 [L]: 357 us in the hook per
 * data-out packet, 209 of it the amortised SD write) - every instruction
 * fetched from flash is its own SPI transaction. So the hot path lives in
 * SRAM, like every other per-transfer path on this port: 228 us a packet
 * afterwards, and a 2 MB write went from 32 s to 11.8 s [L]. It costs about
 * 960 bytes of SRAM. */
#define USBHOT __attribute__((section(".icode"), noinline))

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

/* The CSW lives at MSC_BASE+0x228: signature, tag, dCSWDataResidue, status.
 * The residue is the field this driver forgot, and it cost a whole round of
 * "our side works but the host disagrees".
 *
 * The ROM seeds it from the CBW when the command arrives (0x63d6):
 *
 *     [base+0x22c] = [base+0x20c]      CSW tag     = CBW tag
 *     [base+0x230] = [base+0x210]      CSW residue = dCBWDataTransferLength
 *
 * and every handler is expected to DECREMENT it as it moves data. The ROM's
 * own read pump does exactly that at 0x5c64:
 *
 *     r3 = [base+0x230]; r3 -= bytes_sent; [base+0x230] = r3
 *
 * The CSW builder at 0x5e70 writes only the signature and the status - it
 * never recomputes the residue - so a handler that does not decrement sends
 * "I transferred nothing" alongside a full data phase. The host sees data it
 * was told did not arrive and fails the command with DID_ERROR, which is
 * exactly what usb-storage reported while our own counters read
 * hooks=202 chunks=202 sderr=0. */
#define MSC_RESIDUE         MSC32(0x230)

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

/* The CBW as the ROM stores it (0x63c6 reads 31 bytes to +0x208): length at
 * +0x210, flags at +0x214 (bit 7 = data-in). The ROM's own WRITE(10) rejects
 * a data-in flag at 0x5f80 and a length that is not count * blocksize at
 * 0x601e; the write hook delegates both so the ROM fails them. */
#define MSC_CBW_BUF         ((void *)(MSC_BASE + 0x208))
#define MSC_CBW_LEN         MSC32(0x210)
#define MSC_CBW_FLAGS       MSC8(0x214)

/* MUSB indexed RXCOUNT. 0x6454 sets INDEX (0x4004000e) to 1 on entry, so
 * inside the hook this is endpoint 1's. Read for the breadcrumb only. */
#define USB_RXCOUNT         REG16(0x40040018)


/* --- breadcrumbs ------------------------------------------------------------
 *
 * The hook runs inside the USB interrupt, and a write that went wrong there
 * left NOTHING in the ring: the log of 2026-11-07 ends at post-connect. So
 * every stage of a command writes a stage word here BEFORE it does the step,
 * in .persist (app.lds), where it survives the pinhole reset the hang needs.
 * tools/50_read_logring.sh decodes it straight out of SRAM, and the next boot
 * logs the previous run's copy (usb_init_device), so it reaches the card too.
 *
 * Plain stores, no logf: a stage word costs a few cycles, a logf from flash
 * costs about a millisecond, and a 512 KB write is 8192 packets. The per-
 * command logf lines are capped (MSC_LOG_CMDS). */
#define USBLOG_MAGIC        0x05b00000u
#define USBLOG_MASK         0xffff0000u

enum {
    UL_BOOT        = 0x00,  /* cleared at boot after reporting the last run */
    UL_ATTACHED    = 0x01,  /* usb_enable(true) finished */
    UL_DETACHED    = 0x02,  /* usb_enable(false) */
    UL_HOOK        = 0x03,  /* hook entered for 0x28/0x2a */
    UL_REJECT      = 0x04,  /* delegated to the ROM's own handler */

    UL_R_CBW       = 0x10,  /* READ(10) accepted */
    UL_R_SD        = 0x11,  /* calling sd_read_sectors */
    UL_R_SD_DONE   = 0x12,  /* ... returned */
    UL_R_SEND      = 0x13,  /* calling ROM 0x5930 */
    UL_R_SENT      = 0x14,  /* ... returned, back to the ROM */

    UL_W_CBW       = 0x20,  /* WRITE(10) accepted, first pass */
    UL_W_ARM       = 0x21,  /* calling ROM 0x5944 (ack + arm the receive) */
    UL_W_ARMED     = 0x22,  /* ... returned, back to the ROM, waiting */
    UL_W_PKT       = 0x23,  /* a data-out packet arrived (0x6454 re-dispatch) */
    UL_W_SD        = 0x24,  /* calling sd_write_sectors */
    UL_W_SD_DONE   = 0x25,  /* ... returned, rc in sd_rc */
    UL_W_CSW       = 0x26,  /* calling ROM 0x5e70 */
    UL_W_IDLE      = 0x27,  /* CSW queued, command complete */
    UL_W_RUNAWAY   = 0x28,  /* packet with no command in flight */
    UL_W_OLD_WAIT  = 0x29,  /* OLD path: first pass returned without arming */
};

struct yp3_usblog {
    uint32_t stage;     /* USBLOG_MAGIC | UL_*, written BEFORE each step */
    uint32_t seq;       /* bumps with every stage write: is it moving? */
    uint32_t op;        /* opcode | MSC_STATE << 8, last hook entry */
    uint32_t lba;       /* current command's first LBA */
    uint32_t blocks;    /* current command's block count */
    uint32_t left;      /* data-out bytes still to arrive (write) */
    uint32_t pkts;      /* data-out packets taken, this command */
    uint32_t secs;      /* sectors written, this command */
    uint32_t sd_calls;  /* sd_write_sectors calls since attach */
    uint32_t sd_errs;   /* ... that returned non-zero */
    uint32_t sd_rc;     /* last return code (read or write) */
    uint32_t sd_lba;    /* last LBA handed to the SD driver */
    uint32_t wr_begun;  /* WRITE(10) commands accepted since attach */
    uint32_t wr_done;   /* ... finished with a CSW */
    uint32_t rx;        /* RXCOUNT << 16 | armed length, last packet */
    uint32_t tick;      /* current_tick at the last stage write */
};
extern volatile struct yp3_usblog yp3_usblog;     /* app.lds, in .persist */

static inline void usblog_stage(unsigned stage)
{
    yp3_usblog.stage = USBLOG_MAGIC | stage;
    yp3_usblog.seq++;
    yp3_usblog.tick = (uint32_t)current_tick;
}

#ifdef ROCKBOX_HAS_LOGF
static void usblog_report(const char *when)
{
    volatile struct yp3_usblog *u = &yp3_usblog;

    logf("usblog %s: stage=%02lx seq=%lu op=%04lx t=%lu", when,
         (unsigned long)(u->stage & ~USBLOG_MASK), (unsigned long)u->seq,
         (unsigned long)u->op, (unsigned long)u->tick);
    logf("usblog %s: lba=%lu n=%lu left=%lu pkts=%lu secs=%lu rx=%08lx", when,
         (unsigned long)u->lba, (unsigned long)u->blocks,
         (unsigned long)u->left, (unsigned long)u->pkts,
         (unsigned long)u->secs, (unsigned long)u->rx);
    logf("usblog %s: wr=%lu/%lu sd=%lu err=%lu rc=%ld sdlba=%lu", when,
         (unsigned long)u->wr_done, (unsigned long)u->wr_begun,
         (unsigned long)u->sd_calls, (unsigned long)u->sd_errs,
         (long)u->sd_rc, (unsigned long)u->sd_lba);
}
#else
static inline void usblog_report(const char *when) { (void)when; }
#endif

/* Per-command logf lines, capped so a long copy cannot push the interesting
 * first commands out of a 6 KB ring. The stage words keep counting past it. */
#define MSC_LOG_CMDS        16
static unsigned msc_wr_logged;

/* Boot ROM entry points, thumb bit included. Local to this file because
 * nothing else has any business calling them. */
#define ROM_USB_SET_USERFN  ((void (*)(const void *))0x5915u)
#define ROM_USB_ATTACH      ((void (*)(void))0x58f3u)
/* send one data-in chunk from a buffer; ROM 0x5930, used by the ROM's own
 * multi-chunk pump at 0x5c4c */
#define ROM_MSC_SEND        ((void (*)(const void *, unsigned))0x5931u)
/* Arm a data-out receive of `len` bytes, ROM 0x5944: [ctx+0x3a] = 64,
 * [ctx+0x48] = len, then 0x5044, which in device mode ends in 0x4fac,
 * RXCSR |= 0x10 (FlushFIFO) - i.e. it ACKS the packet now sitting in the
 * endpoint FIFO, freeing it for the next one. The ROM's own WRITE(10) calls
 * it at 0x6034 (first pass) and 0x5f5e (after every packet). */
#define ROM_MSC_RECV        ((void (*)(void *, unsigned))0x5945u)
/* Send the CSW with bCSWStatus = status, ROM 0x5e70: sets MSC_STATE = 0,
 * queues 13 bytes from +0x228 through 0x5920, and re-arms the next CBW
 * receive with 0x5944(+0x208, 31). Residue is NOT recomputed. */
#define ROM_MSC_CSW         ((void (*)(unsigned))0x5e71u)
/* Set sense key / ASC, ROM 0x5a7c(key, asc): e.g. (5, 0x20) at 0x62d0. */
#define ROM_MSC_SENSE       ((void (*)(unsigned, unsigned))0x5a7du)
/* Bulk max packet at full speed. The ROM's data-out pump moves exactly this
 * much per packet (0x5eb4: len = min(remaining, 64)), and the link is full
 * speed [L]. A high-speed link would deliver 512-byte packets and this path
 * would read 64 of them; the breadcrumb records RXCOUNT to catch that. */
#define MSC_PACKET          64u
/* the USB PHY/controller enable the vendor reaches through SRAM thunk
 * 0x80db54, which is one branch to this */
#define ROM_USB_PHY         ((void (*)(unsigned))0x3ad5u)
/* Soft-connect. usb_attach does NOT do this - it ends in ROM 0x5204, which is
 * the soft-DISCONNECT, so that the host sees a clean attach afterwards. The
 * pull-up is a separate call, and without it D+ is never driven and the host
 * never sees a device at all:
 *
 *     0x7f4c  [0x40040060] |= 1;          DEVCTL SESSION
 *             b 0x51f0                    [0x40040001] |= 0x40; delay(3)
 *     0x7f60  [0x40040060] &= ~1;
 *             b 0x5204                    [0x40040001] &= ~0x40; delay(3)
 *
 * 0x40040001 is POWER and 0x40 is SOFTCONN, which is what the log said: the
 * register read back 0x20 (high-speed enable, no soft-connect) through every
 * attach. The ROM's own download mode reaches 0x7f4c through the thunk chain
 * 0x590c -> 0x55b8 -> 0x51f0, so this is the known-good enumeration path. */
#define ROM_USB_CONNECT     ((void (*)(void))0x7f4du)
#define ROM_USB_DISCONNECT  ((void (*)(void))0x7f61u)
/* ROM 0x3c2c, once called ROM_USB_PHY_INIT here, is the PMU mailbox bus
 * init: 0x40085100 is the mailbox's bus configuration, not a PHY. It now
 * goes through yp3_pmu_bus_init() in power-yp3box.c. See usb_hw_enable(). */
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
static USBHOT int usb_msc_ready(void)
{
    tCardInfo *card = card_get_info_target(0);

    return (card && card->initialized > 0 && card->numblocks != 0) ? 1 : 0;
}

/* +0x0c, write protect. 1 means protected: ROM 0x5fb0 turns that into sense
 * 0x27, WRITE PROTECTED. */
static USBHOT int usb_msc_write_protected(void)
{
    /* A rig-USB build is read-only by the same switch as any other: it used
     * to be forced read-only, until writes were proven on 2026-11-15 and the
     * rig became the way to change the card without moving it. */
    return YP3_USB_WRITABLE ? 0 : 1;
}

/* The ROM's MODE SENSE never reports write protection: 0x1a at ROM 0x59e4
 * returns eight zero bytes and 0x5a at 0x59fc a bare 8-byte header, so the
 * host mounts rw and only learns about it when WRITE(10) fails (sense 2/0x27,
 * ROM 0x5fb0) - safe, but every write is an I/O error. Answer both the way
 * the ROM does, same lengths and layout, with WP set: byte 2 bit 7 of the
 * 6-byte form's header, byte 3 bit 7 of the 10-byte form's. Returns 1 when
 * handled, which the dispatcher at ROM 0x6328 takes as "done". */
static USBHOT int usb_msc_mode_sense_wp(unsigned opcode)
{
    uint8_t *b = (uint8_t *)MSC_BUF;
    unsigned i;

    if ((opcode != 0x1a && opcode != 0x5a) || !usb_msc_write_protected())
        return 0;

    for (i = 0; i < 8; i++)
        b[i] = 0;
    if (opcode == 0x1a) {
        b[2] = 0x80;                    /* device-specific parameter: WP */
    } else {
        b[1] = 0x06;                    /* mode data length, as ROM 0x5a04 */
        b[3] = 0x80;
    }
    MSC_XFER_LEN = 8;                   /* ROM 0x59e8 / 0x59fe */
    return 1;
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

/* Counters, not logf: the hook runs in the USB interrupt (ROM 0x631c is
 * reached from the IRQ 41/42 handlers). logf has no lock, so a line from here
 * can interleave with one a thread was half-way through - garbled text, not a
 * hang - and it formats from flash, about a millisecond a line. So the hot
 * path writes stage words (yp3_usblog, above) and the only logf lines are one
 * per WRITE(10) command, capped at MSC_LOG_CMDS. The counters are reported
 * from usb_enable(false), in thread context.
 *
 * The host's READ(10) for 8 sectors at LBA 0 comes back DID_ERROR, and there
 * are three candidate explanations that these numbers separate cleanly:
 *
 *   hooks == 0            the dispatcher never reaches us - table or opcode
 *   hooks 1, chunks 1     the ROM did not re-dispatch for chunk 2, so the
 *                         host got 512 bytes of a 4096-byte transfer and
 *                         timed out: the multi-chunk protocol is wrong
 *   sderrs > 0            storage_enable(false) left the card unusable, so
 *                         the hook is fine and the medium is not
 *
 * A fourth stays unmeasured for now: sd_read_sectors is polled, and running
 * it inside the USB interrupt may hold the handler long enough for the host
 * to time out. This target has no microsecond counter to time it with, and
 * chunks==1 would point at it anyway. */
static uint32_t msc_hooks, msc_chunks, msc_rejects, msc_sderrs, msc_runaways;
static uint32_t msc_last_lba;
/* Wall time in USB mode, so throughput is a measured number rather than an
 * argument about which of two plausible bottlenecks matters more. The link
 * enumerates at FULL speed - the kernel says so, and the ROM's own download
 * mode does the same - which caps it near 1 MB/s before our code does
 * anything. Against that, every 512 bytes here costs one polled CMD17 with a
 * FIFO drained a word at a time, from inside the USB interrupt. If the
 * measured rate lands near the link cap the SD path is not worth rewriting;
 * if it lands an order of magnitude below, it is. */
static long msc_t0;

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
static USBHOT bool usb_msc_begin(void)
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

/* The read path is a per-chunk state machine. usb_msc_hook only calls
 * usb_msc_begin() when the state is IDLE and otherwise falls straight through
 * to usb_msc_read_chunk, so a re-dispatch that arrives with the command
 * already finished must find "nothing left" and send the CSW - a counter
 * decremented past zero there once streamed sectors for ever, which was the
 * `cat` that never returned.
 *
 * The send does not copy: 0x4e1c -> 0x53b8 points the MUSB DMA at the buffer
 * (TXCSR |= 0x9400, AUTOSET | DMAReqMode | DMAReqEnab), so the bytes leave
 * memory packet by packet and a buffer may not be touched until its
 * completion has really arrived. Two buffers alternated WITHOUT the
 * early-completion test hid a corrupted last packet for 13 clean 8 MB reads
 * [L] and then a read hung, because the channel was re-armed under a running
 * transfer. */

/* What the MUSB DMA channel looks like each time the data-in completion
 * re-enters the hook (yp3-0gn read corruption / read hang, 2026-11-15).
 * Channel 1: INTR 0x40040200, CNTL +0x204, ADDR +0x208, COUNT +0x20c (ROM
 * 0x53b8 literals 0x5460/0x5464, IRQ 42 handler 0x54e4). The first
 * continuation's values are kept as the reference; any later entry that
 * differs is counted in `odd` with its values, so "the completion came
 * before the channel had finished" is a number. `stall_*` is filled from
 * thread context (usb_detect) when a read chunk has been out for 3 s with no
 * completion. Plain .bss, read post-mortem by nm address: .bss survives the
 * reset. Not static, so the symbol is easy to find.
 *
 * Logf builds only: tools/usbdma_read.py reads it, and a daily build pays
 * nothing for it. The early-completion test itself is not part of this. */
#ifdef ROCKBOX_HAS_LOGF
#define YP3_USB_STATS 1
#else
#define YP3_USB_STATS 0
#endif

#define USB_DMA_CNTL        REG16(0x40040204)
#define USB_DMA_ADDR        REG32(0x40040208)
#define USB_DMA_COUNT       REG32(0x4004020c)
#define USB_TXCSR_IDX       REG16(0x40040012)
#define USB_INDEX           REG8(0x4004000e)

/* A chunk has been handed to ROM_MSC_SEND and its completion not yet taken. */
static bool msc_rd_inflight;

#if YP3_USB_STATS
struct yp3_usbdma {
    uint32_t n, odd;
    uint32_t ref_cntl, ref_adv, ref_count, ref_txcsr;
    uint32_t odd_cntl, odd_adv, odd_count, odd_txcsr, odd_lba;
    uint32_t last_cntl, last_adv, last_count, last_txcsr;
    uint32_t stalls, stall_cntl, stall_addr, stall_count, stall_txcsr,
             stall_state, stall_tick, stall_buf;
    uint32_t early;         /* completions refused: the channel was not done */
    uint32_t fifo_busy;     /* channel done, but a packet still in the FIFO */
    /* SD time per sector as the USB path sees it, in SysTick counts / 256
     * (the tick's reload is one 10 ms period, so a call longer than that
     * reads short; max says whether any came close). Where a USB transfer's
     * time goes is otherwise an argument, not a number. */
    uint32_t rd_n, rd_cyc, rd_max, wr_n, wr_cyc, wr_max, syst_rvr;
    uint32_t wr_polls;      /* busy polls the SD driver made during writes */
    /* A data-out packet's cost, split: time inside the hook, and the gap
     * from leaving the hook to being entered for the next packet (the ROM,
     * the bus and the host). SysTick counts / 256, as above. */
    uint32_t pk_n, pk_in, pk_gap, pk_gap_max;
    /* DMA receive: runs completed, completions that were not one (channel
     * still enabled or bytes left), hook entries during a DMA write, the
     * longest wait for the next packet in spins, waits that gave up. */
    uint32_t wd_n, wd_odd, wd_spur, wd_wait_max, wd_wait_to;
    uint32_t wd_odd_cntl, wd_odd_count, wd_odd_rxcsr;
} yp3_usbdma;
static void *msc_rd_prev;
static unsigned msc_rd_sent;        /* bytes in the run on the wire */
#define USBSTAT(x)          do { x; } while (0)
/* Time a region: USBSTAT_T0 before it, USBSTAT_T1 after, adding to the
 * struct's <f>_n, <f>_cyc and <f>_max. */
#define USBSTAT_T0(t)       uint32_t t = usb_syst()
#define USBSTAT_T1(t, f)    do { uint32_t dt_ = usb_syst_since(t);          \
                                 yp3_usbdma.f##_n++;                       \
                                 yp3_usbdma.f##_cyc += dt_ >> 8;           \
                                 if (dt_ > yp3_usbdma.f##_max)             \
                                     yp3_usbdma.f##_max = dt_;             \
                                 yp3_usbdma.syst_rvr =                     \
                                     REG32(0xE000E014) & 0xffffffu; } while (0)

static inline uint32_t usb_syst(void) { return REG32(0xE000E018) & 0xffffffu; }
static inline uint32_t usb_syst_since(uint32_t t0)
{
    uint32_t t1 = usb_syst(), rvr = REG32(0xE000E014) & 0xffffffu;
    return (t0 >= t1) ? t0 - t1 : t0 + rvr + 1 - t1;   /* it counts DOWN */
}

static USBHOT void usb_dma_note(void)
{
    struct yp3_usbdma *d = &yp3_usbdma;
    uint32_t cntl = USB_DMA_CNTL, adv = USB_DMA_ADDR - (uint32_t)msc_rd_prev,
             count = USB_DMA_COUNT, txcsr = USB_TXCSR_IDX;

    d->last_cntl = cntl; d->last_adv = adv;
    d->last_count = count; d->last_txcsr = txcsr;
    if (d->n++ == 0) {
        d->ref_cntl = cntl; d->ref_adv = adv;
        d->ref_count = count; d->ref_txcsr = txcsr;
    }
    /* Measured 2026-11-15 [L], 3,500 completions: the normal picture is
     * cntl 021e (enable clear), address advanced 512, count 0, TXCSR 9404
     * (TxPktRdy and FIFONotEmpty both clear - everything already sent).
     * Anything else is the event being hunted. */
    if ((cntl & 1) || adv != msc_rd_sent || count != 0 || (txcsr & 3)) {
        d->odd++;
        d->odd_cntl = cntl; d->odd_adv = adv;
        d->odd_count = count; d->odd_txcsr = txcsr;
        d->odd_lba = msc_lba;
    }
}
#else
#define USBSTAT(x)          do { } while (0)
#define USBSTAT_T0(t)       do { } while (0)
#define USBSTAT_T1(t, f)    do { } while (0)
#endif /* YP3_USB_STATS */

/* READS GO OUT IN RUNS OF UP TO EIGHT SECTORS, WITH THE NEXT RUN READ FROM
 * THE CARD WHILE THE CURRENT ONE IS ON THE WIRE (yp3-0gn.18).
 *
 * One sector per chunk cost 430 us of SD time and then ~560 us of USB time,
 * strictly one after the other: 517 KB/s measured 2026-11-15 [L] on a link
 * good for about twice that. Two things change:
 *
 * - The chunk is MSC_RD_RUN sectors. ROM_MSC_SEND (0x5930) cannot do that:
 *   it stores 512 in the endpoint context's per-call limit ([ctx+0x3a], ctx =
 *   *(0x804634) = 0x804658 in the stock capture) and 0x4e1c then moves
 *   min(limit, len). So usb_msc_send() stores its own limit and enters
 *   0x4e1c directly, which is all 0x5930 does. 0x53b8 takes the length as a
 *   32-bit DMA count; the context's remaining-length field is 16 bits and
 *   ends up 0. One CMD18 for the run also costs less than eight CMD17s.
 *
 * - There are two run buffers. A run is handed to the DMA and the hook then
 *   reads the following run into the other buffer before it returns; the
 *   completion interrupt is pending by then and is taken at once. The early
 *   completion is still refused first (below), so the channel is never
 *   re-armed while it is moving.
 *
 * THE BUFFERS ARE THE CODEC BUFFER. Two runs are 8 KB, and the arena has no
 * 8 KB to give (docs/CODEC-MEMORY.md). codecbuf is 110 KB that nothing uses
 * in USB mode: usb.c waits for the audio thread's SYS_USB_CONNECTED ack
 * before usb_enable(true), that ack is sent after audio_stop_playback(), and
 * every later codec load copies the whole image in again (lc-yp3box.c), so
 * nothing in it has to survive. It is SRAM and word-aligned, which is what
 * the SD IDMAC and the MUSB DMA both need. */
#define MSC_RD_RUN          8u              /* sectors; SD_MULTI_MAX */
extern unsigned char codecbuf[];
#define MSC_RD_BUF(i)       ((void *)((((uint32_t)codecbuf + 7u) & ~7u)       \
                                      + (i) * (MSC_RD_RUN * 512u)))
#define ROM_EP_CTX          REG32(0x00804634)       /* ROM 0x4e94 */
#define ROM_EP_SEND         ((void (*)(const void *, unsigned, unsigned))0x4e1du)

static unsigned msc_rd_cur;         /* which buffer went out last */
static unsigned msc_rd_ahead;       /* sectors waiting in the other one */

static USBHOT void usb_msc_send(const void *buf, unsigned len)
{
    REG16(ROM_EP_CTX + 0x3a) = len;         /* what 0x5930 sets to 512 */
    ROM_EP_SEND(buf, len, 1);
}

/* Read the next run off the card into buffer `i`. Returns its length in
 * sectors, 0 when the command has no more. */
static USBHOT unsigned usb_msc_read_run(unsigned i)
{
    unsigned n = msc_blocks_left < MSC_RD_RUN ? msc_blocks_left : MSC_RD_RUN;
    void *buf = MSC_RD_BUF(i);

    if (n == 0)
        return 0;

    usblog_stage(UL_R_SD);
    {
        USBSTAT_T0(t0);
        yp3_usblog.sd_rc = (uint32_t)sd_read_sectors(IF_MD(0,) msc_lba, n, buf);
        USBSTAT_T1(t0, rd);
        USBSTAT(yp3_usbdma.rd_n += n - 1);  /* the mean is per sector */
    }
    usblog_stage(UL_R_SD_DONE);
    if (yp3_usblog.sd_rc != 0) {
        msc_sderrs++;
        /* Nothing here can fail the command: by this point the data phase
         * is in flight and the only ways out are "send the next chunk" and
         * "send the CSW". A zeroed run keeps the transfer counted and the
         * bus alive; wedging it would need the host to time out. The log
         * line is the real report. */
        logf("usb: read error at lba %lu", (unsigned long)msc_lba);
        memset(buf, 0, n * 512u);
    }

    msc_last_lba = msc_lba + n - 1;
    msc_chunks += n;
    msc_lba += n;
    msc_blocks_left -= n;
    return n;
}

static USBHOT void usb_msc_read_chunk(void)
{
    unsigned n;

    /* THE COMPLETION CAN COME EARLY. Two things re-dispatch a data-in
     * command: the DMA channel's interrupt (IRQ 42, ROM 0x54e4) and endpoint
     * 1's own TX interrupt (IRQ 41, ROM 0x5694 -> table +8), and the second
     * one sometimes fires while the channel is still moving this chunk.
     * Measured 2026-11-15 [L]: 280 of 135,969 completions arrived with
     * CNTL 021f (still enabled), 64 bytes moved and 448 to go, TxPktRdy set.
     * Taking one as real read the next sector under a running transfer -
     * which is the corrupted last packet - and then re-armed a channel that
     * had not finished, which is the likeliest cause of the read that hung.
     * So an early one is refused: the channel is still running with its
     * interrupt enabled, and its real completion follows. */
    if (msc_rd_inflight) {
        USBSTAT(usb_dma_note());
        if ((USB_DMA_CNTL & 1) || USB_DMA_COUNT != 0) {
            USBSTAT(yp3_usbdma.early++);
            return;
        }
        USBSTAT(if (USB_TXCSR_IDX & 3) yp3_usbdma.fifo_busy++);
        msc_rd_inflight = false;
    }

    /* The run to send: the one read ahead, or - first chunk of a command -
     * one read now. */
    if (msc_rd_ahead) {
        msc_rd_cur ^= 1;
        n = msc_rd_ahead;
        msc_rd_ahead = 0;
    } else {
        n = usb_msc_read_run(msc_rd_cur);
    }

    /* The data phase is over and its last chunk has really gone: send the
     * CSW from here (ROM 0x5e70, as the write path does). The old way was to
     * mark the last chunk MSC_ST_LAST and let ROM 0x6380 send the CSW at its
     * completion - but 0x6380 does that without calling the hook, so an
     * early completion on the last chunk would have put a CSW on the wire in
     * the middle of the data, and nothing here could refuse it. The state
     * therefore stays DATA_IN until this point. Run 2026-11-15 [L]: 36 of 36
     * reads of an 8 MB file clean where 5 of 10 had been bad, and the
     * READ(10) DID_ERROR that used to follow every attach is gone ([I] that
     * the misplaced CSW was its cause). */
    if (n == 0) {
        ROM_MSC_CSW(0);
        return;
    }

    /* Account for the run, or the CSW will claim it never went. */
    MSC_RESIDUE -= n * 512u;

    MSC_STATE = MSC_ST_DATA_IN;         /* never LAST: see above */
    MSC_XFER_LEN = n * 512u;
    USBSTAT(msc_rd_prev = MSC_RD_BUF(msc_rd_cur));
    USBSTAT(msc_rd_sent = n * 512u);
    usblog_stage(UL_R_SEND);
    usb_msc_send(MSC_RD_BUF(msc_rd_cur), n * 512u);
    msc_rd_inflight = true;
    usblog_stage(UL_R_SENT);

    /* And while that is on the wire, fetch the one after it. */
    msc_rd_ahead = usb_msc_read_run(msc_rd_cur ^ 1);
}

/* A run of sectors through the SD driver, with its breadcrumbs. Shared by
 * both write paths so the old one can be compared on hardware like for like. */
static USBHOT int usb_msc_write_run(uint32_t lba, unsigned n, const void *buf)
{
    int rc;

    yp3_usblog.sd_lba = lba;
    yp3_usblog.sd_calls++;
    usblog_stage(UL_W_SD);
    {
        extern uint32_t yp3_sd_ready_polls;
        uint32_t p0 = yp3_sd_ready_polls;
        USBSTAT_T0(t0);
        rc = sd_write_sectors(IF_MD(0,) lba, n, buf);
        USBSTAT_T1(t0, wr);
        USBSTAT(yp3_usbdma.wr_n += n - 1);  /* the mean is per sector */
        USBSTAT(yp3_usbdma.wr_polls += yp3_sd_ready_polls - p0);
        (void)p0;
    }
    yp3_usblog.sd_rc = (uint32_t)rc;
    usblog_stage(UL_W_SD_DONE);
    if (rc != 0) {
        yp3_usblog.sd_errs++;
        msc_sderrs++;
    }
    return rc;
}

#if YP3_USB_WRITE_FIX
/* ==== THE FIX (YP3_USB_WRITE_FIX) ==========================================
 *
 * WRITE(10), transcribed from the ROM's own data-out pump rather than
 * mirrored from the read path. The old version assumed "0x6454 re-arms the
 * receive into the staging buffer before it re-dispatches, so a whole block
 * is waiting in MSC_BUF". The ROM says otherwise, and the old path is wrong in
 * three independent ways, all [C]:
 *
 * 1. NOTHING ARMED THE RECEIVE. 0x6454 does not arm anything: it copies
 *    [ctx+0x48] bytes OUT of the endpoint FIFO with 0x5378 (a PIO read of
 *    0x40040024) and re-dispatches. The arm/ack is 0x5944, which ends in
 *    RXCSR |= FlushFIFO and so releases the FIFO for the next packet. The
 *    ROM's WRITE(10) calls it on the first pass (0x6034) and after every
 *    packet (0x5f5e). The old hook called it nowhere, so the CBW's packet was
 *    never acknowledged, the controller NAKed every data-out packet, and the
 *    host sat on its 30 s SCSI timeout - "hangs on the USB screen, silently".
 *    The READ path never noticed because the CSW (0x5e70) ends by arming the
 *    next CBW, which acks the old one.
 *
 * 2. A RE-DISPATCH IS ONE 64-BYTE PACKET, NOT A 512-BYTE BLOCK. The ROM pump
 *    0x5eb4 moves min(remaining, 64) per call, and 0x6454 reads exactly the
 *    length the last 0x5944 armed. With nothing armed that was 31 - the CBW's
 *    length, left in [ctx+0x48] - so even a delivered packet would have
 *    become a 512-byte sector of 31 new bytes and stale data.
 *
 * 3. NO CSW AT THE END. 0x6454 switches on the state byte and only handles 0
 *    (a new CBW, 0x63b8) and 1 (data, re-dispatch); anything else returns
 *    without doing anything (0x6466). Unlike 0x6380 on the data-in side,
 *    state 3 does NOT send a CSW for data-out. The ROM's pump sends it itself
 *    (0x5f46, 0x5e70(0)); the old hook set state 3 and stopped.
 *
 * The ROM's 0x60f8/0x6050 (vendor command 0xc0, subcommands 1, 2, 9 - the
 * smartlink_flash write and write_mem path) use the same 64-byte 0x5944 loop,
 * so this protocol is the one every flash of this device has exercised [L].
 * The vendor's own hook (SRAM 0x813550) uses 512-byte DMA receives through
 * FIRM 0xd7eda4 and replaces the ROM's DMA callback (table 0x8004e0 +0x20 ->
 * 0x813371) to skip the PIO copy; that is faster and is NOT what this does -
 * it needs the callback table patched and a DMA path nobody here has run.
 *
 * Packets are gathered into a run of up to MSC_RD_RUN sectors in the third
 * 4 KB of the codec buffer (see usb_msc_read_chunk for why that is free and
 * why the SD IDMAC may read it) and committed with ONE sd_write_sectors call
 * - one CMD25 instead of eight CMD24s, each of which waited out its own
 * card-busy period (1,670 us a sector measured 2026-11-15 [L]). The next
 * packet is armed after the write, the same order as 0x5eb4 (write op, then
 * 0x5944). Nothing here waits: everything between packets is the host's
 * problem. */
#define MSC_WR_BUF          ((uint8_t *)MSC_RD_BUF(2))
static uint32_t msc_wr_bytes;           /* data-out bytes still to arrive */
static uint32_t msc_wr_fill;            /* bytes of MSC_WR_BUF filled */
static bool     msc_wr_err;             /* a sector failed this command */
static unsigned msc_wr_armed;           /* length the last 0x5944 asked for */

static USBHOT void usb_msc_write_arm(void)
{
    unsigned n = msc_wr_bytes < MSC_PACKET ? msc_wr_bytes : MSC_PACKET;

    msc_wr_armed = n;
    yp3_usblog.left = msc_wr_bytes;
    yp3_usblog.rx = (yp3_usblog.rx & 0xffff0000u) | n;
    usblog_stage(UL_W_ARM);
    ROM_MSC_RECV(MSC_BUF, n);
    usblog_stage(UL_W_ARMED);
}

/* First pass, from the CBW handler 0x63b8 with the state byte 0: what the
 * ROM's WRITE(10) does at 0x602e..0x6034 - state 1, arm the first packet. */
#if YP3_USB_WRITE_DMA
static USBHOT void usb_msc_write_dma_start(void);
#endif
static USBHOT void usb_msc_write_start(void)
{
    msc_wr_bytes = msc_blocks_left * 512u;
    msc_wr_fill = 0;
    msc_wr_err = false;
    MSC_STATE = MSC_ST_DATA_OUT;
#if YP3_USB_WRITE_DMA
    usb_msc_write_dma_start();
#else
    usb_msc_write_arm();
#endif
}

/* Every later pass, from 0x6454 with the state byte 1: the packet armed last
 * time is in MSC_BUF. */
static USBHOT void usb_msc_write_packet(void)
{
    const uint32_t *src = (const uint32_t *)MSC_BUF;
    uint32_t *dst;
    unsigned n, i;

    if (msc_wr_bytes == 0) {
        /* A packet with no write in flight. Ack it and go back to waiting
         * for a CBW - what the vendor's TX callback does after its CSW
         * (SRAM 0x8133a8: state 0, 0x5944(+0x208, 31)). [I]: not seen. */
        msc_runaways++;
        usblog_stage(UL_W_RUNAWAY);
        MSC_STATE = MSC_ST_IDLE;
        ROM_MSC_RECV(MSC_CBW_BUF, 31);
        return;
    }

    n = msc_wr_armed;                       /* 0x6454 copied exactly this */
    yp3_usblog.rx = ((uint32_t)USB_RXCOUNT << 16) | n;
    yp3_usblog.pkts++;
    usblog_stage(UL_W_PKT);

    /* n is 64 or a multiple of 4 below it, and both buffers are aligned:
     * MSC_BUF is 0x800df0. Open-coded - this is the per-packet path. */
    dst = (uint32_t *)(MSC_WR_BUF + msc_wr_fill);
    for (i = 0; i < n / 4; i++)
        dst[i] = src[i];

    msc_wr_fill  += n;
    msc_wr_bytes -= n;
    MSC_RESIDUE  -= n;                      /* 0x5f34: residue -= len */

    /* A full run, or the end of the command (its length is a whole number
     * of sectors - the hook delegated anything else). */
    if (msc_wr_fill >= MSC_RD_RUN * 512u || msc_wr_bytes == 0) {
        unsigned secs = msc_wr_fill / 512u;

        if (secs && usb_msc_write_run(msc_lba, secs, MSC_WR_BUF) != 0)
            msc_wr_err = true;
        msc_last_lba = msc_lba + secs - 1;
        msc_chunks += secs;
        msc_lba += secs;
        msc_blocks_left -= secs;
        msc_wr_fill = 0;
        yp3_usblog.secs += secs;
    }

    if (msc_wr_bytes == 0) {
        /* 0x5f42..0x5f46: the data phase is over, send the CSW. Unlike the
         * read side a failure CAN be reported here, because the data phase
         * has ended: MEDIUM ERROR / WRITE ERROR, then status 1. */
        yp3_usblog.left = 0;
        usblog_stage(UL_W_CSW);
        if (msc_wr_err)
            ROM_MSC_SENSE(3, 0x0c);
        ROM_MSC_CSW(msc_wr_err ? 1 : 0);
        yp3_usblog.wr_done++;
        usblog_stage(UL_W_IDLE);
        if (msc_wr_logged <= MSC_LOG_CMDS)
            logf("msc: W done secs=%lu pkts=%lu err=%d rc=%ld",
                 (unsigned long)yp3_usblog.secs,
                 (unsigned long)yp3_usblog.pkts, msc_wr_err,
                 (long)yp3_usblog.sd_rc);
        return;
    }

    usb_msc_write_arm();                    /* 0x5f5e */
}

#if YP3_USB_WRITE_DMA
/* ==== WRITE(10) BY DMA (yp3-0gn.18) ========================================
 *
 * The 0x5944 pump above takes one 64-byte packet per interrupt and the host's
 * next OUT is NAKed until the hook has returned and re-armed: 206 us from
 * leaving the hook to the next packet, measured 2026-11-16 [L], which is
 * 1.65 ms of every 2.15 ms sector once the SD side was batched. The vendor
 * does not use that pump. Its hook (SRAM 0x813550) receives with the MUSB
 * DMA, and this is that sequence:
 *
 *   first pass, 0x8135b8..0x8135f0
 *       state = 1; INDEX = 1; RXCSR &= ~1          ack the CBW's packet
 *       dma_start(buf, min(512, left), ep 1, rx, mode 1)
 *   dma_start, FIRM 0xd7eda4 with r3 = 0 (receive), [sp] = 1
 *       INDEX = ep; RXCSR = (RXCSR & ~0x800) | 0xa000   AUTOCLEAR|DMAReqEnab
 *       ctx[0x4a] = 0                               "last DMA was a receive"
 *       ADDR (0x40040208) = buf; COUNT (0x4004020c) = len
 *       CNTL (0x40040204) = (CNTL & ~0xf6) | 4 | 8 | ep << 4 | 1
 *   completion: DMA interrupt, ROM 0x54e4 -> [*(0x80464c) + 0x20]
 *       the ROM's entry there is 0x58cc: ctx[0x4a] ? 0x6380 : 0x6454, and
 *       0x6454 would PIO-read a FIFO the DMA has already emptied. The vendor
 *       replaces the entry (0x8004e0 + 0x20 -> 0x813371), which keeps 0x6380
 *       for a transmit and for a receive goes straight back to its hook.
 *   later passes, 0x8135f6..0x813738
 *       write the sector; residue -= n; if none left, CSW
 *       else: while (!(RXCSR & 1)) ;   INDEX = 1;   dma_start(next)
 *
 * The table is SRAM (0x8004e0, read off this device in ROM mode 2026-11-16:
 * +0x20 = 0x58cd), so the entry is replaced the same way. ROM 0x53b8 is the
 * same routine as 0xd7eda4 except that it never sets the mode bit (it ORs
 * mode << 4 and then clears 0xf0), so the start is written out here rather
 * than called.
 *
 * Three departures from the vendor, each marked [C] until run:
 * - a run is up to MSC_RD_RUN sectors, not 512 bytes, into the codec buffer
 *   (the read side already sends 4 KB per DMA [L]);
 * - the wait for RxPktRdy is bounded. The vendor's is not, and it runs in
 *   the interrupt: a host that stopped mid-command would hang the device.
 *   On timeout the channel is started anyway and takes the packet when it
 *   comes;
 * - the CSW goes through ROM 0x5e70 as everywhere else in this file, whose
 *   re-arm of the CBW receive (0x5944 -> 0x5044) clears AUTOCLEAR and
 *   DMAReqEnab again. */
#define ROM_USB_CB_TABLE_ENTRY REG32(REG32(0x0080464c) + 0x20)  /* ROM 0x5514 */
#define ROM_DATA_IN_DONE    ((void (*)(void))0x6381u)
#define USB_RXCSR_IDX       REG16(0x40040016)
#define MSC_WR_WAIT_SPINS   200000u         /* tens of ms; a packet is 50 us */

static USBHOT void usb_msc_write_dma_arm(void)
{
    unsigned n = msc_wr_bytes < MSC_RD_RUN * 512u ? msc_wr_bytes
                                                   : MSC_RD_RUN * 512u;

    msc_wr_armed = n;
    yp3_usblog.left = msc_wr_bytes;
    usblog_stage(UL_W_ARM);
    USB_INDEX = 1;
    USB_RXCSR_IDX = (USB_RXCSR_IDX & ~0x0800u) | 0xa000u;
    REG8(ROM_EP_CTX + 0x4a) = 0;
    USB_DMA_ADDR  = (uint32_t)MSC_WR_BUF;
    USB_DMA_COUNT = n;
    USB_DMA_CNTL  = (USB_DMA_CNTL & ~0x00f6u) | 0x04u | 0x08u | (1u << 4) | 1u;
    usblog_stage(UL_W_ARMED);
}

/* The DMA channel's completion, in place of ROM 0x58cc. */
static USBHOT void usb_msc_dma_cb(void)
{
    unsigned n, secs, index;

    if (REG8(ROM_EP_CTX + 0x4a)) {          /* a transmit: the ROM's own */
        ROM_DATA_IN_DONE();
        return;
    }
    if (MSC_STATE != MSC_ST_DATA_OUT || msc_wr_bytes == 0) {
        USBSTAT(yp3_usbdma.wd_spur++);
        return;
    }
    index = USB_INDEX;
    USB_INDEX = 1;
    if ((USB_DMA_CNTL & 1) || USB_DMA_COUNT != 0) {
        /* Not a completion. Leave the channel to finish; say so. */
        USBSTAT(yp3_usbdma.wd_odd++);
        USBSTAT(yp3_usbdma.wd_odd_cntl = USB_DMA_CNTL);
        USBSTAT(yp3_usbdma.wd_odd_count = USB_DMA_COUNT);
        USBSTAT(yp3_usbdma.wd_odd_rxcsr = USB_RXCSR_IDX);
        USB_INDEX = index;
        return;
    }
    USBSTAT(yp3_usbdma.wd_n++);

    n = msc_wr_armed;
    secs = n / 512u;
    yp3_usblog.pkts += n / MSC_PACKET;
    usblog_stage(UL_W_PKT);
    msc_wr_bytes -= n;
    MSC_RESIDUE  -= n;

    if (usb_msc_write_run(msc_lba, secs, MSC_WR_BUF) != 0)
        msc_wr_err = true;
    msc_last_lba = msc_lba + secs - 1;
    msc_chunks += secs;
    msc_lba += secs;
    msc_blocks_left -= secs;
    yp3_usblog.secs += secs;

    if (msc_wr_bytes == 0) {
        yp3_usblog.left = 0;
        usblog_stage(UL_W_CSW);
        if (msc_wr_err)
            ROM_MSC_SENSE(3, 0x0c);
        ROM_MSC_CSW(msc_wr_err ? 1 : 0);
        yp3_usblog.wr_done++;
        usblog_stage(UL_W_IDLE);
    } else {
        unsigned spins = 0;

        USB_INDEX = 1;
        while (!(USB_RXCSR_IDX & 1) && spins < MSC_WR_WAIT_SPINS)
            spins++;
        USBSTAT(if (spins > yp3_usbdma.wd_wait_max)
                    yp3_usbdma.wd_wait_max = spins);
        USBSTAT(if (spins >= MSC_WR_WAIT_SPINS) yp3_usbdma.wd_wait_to++);
        usb_msc_write_dma_arm();
    }
    USB_INDEX = index;
}

static USBHOT void usb_msc_write_dma_start(void)
{
    ROM_USB_CB_TABLE_ENTRY = (uint32_t)usb_msc_dma_cb | 1u;
    USB_INDEX = 1;
    USB_RXCSR_IDX &= ~1u;                   /* 0x8135ce: ack the CBW */
    usb_msc_write_dma_arm();
}
#endif /* YP3_USB_WRITE_DMA */
/* ==== end of THE FIX ====================================================== */
#else
/* The OLD path, kept only behind YP3_USB_WRITE_FIX=0 so the diagnosis can be
 * confirmed on hardware with the same breadcrumbs. Predicted result: stage
 * W_OLD_WAIT, pkts=0, and nothing after it until the host gives up. */
static USBHOT void usb_msc_write_start(void)
{
    MSC_STATE = MSC_ST_DATA_OUT;
    usblog_stage(UL_W_OLD_WAIT);
}

static USBHOT void usb_msc_write_packet(void)
{
    if (msc_blocks_left == 0) {         /* see usb_msc_read_chunk */
        msc_runaways++;
        usblog_stage(UL_W_RUNAWAY);
        MSC_STATE = MSC_ST_LAST;
        return;
    }

    yp3_usblog.rx = (uint32_t)USB_RXCOUNT << 16;
    yp3_usblog.pkts++;
    usblog_stage(UL_W_PKT);

    if (usb_msc_write_run(msc_lba, 1, MSC_BUF) != 0)
        logf("usb: write error at lba %lu", (unsigned long)msc_lba);

    msc_lba++;
    msc_blocks_left--;
    yp3_usblog.secs++;

    MSC_RESIDUE -= 512;

    MSC_STATE = msc_blocks_left ? MSC_ST_DATA_OUT : MSC_ST_LAST;
}
#endif /* YP3_USB_WRITE_FIX */

static USBHOT int usb_msc_hook(unsigned opcode)
{
    if (usb_msc_mode_sense_wp(opcode))
        return 1;

    if (opcode != 0x28 && opcode != 0x2a)
        return 0;                       /* the ROM's own handlers are fine */

    if (!usb_exposed)
        return 0;

    /* The hook sees WRITE(10) before the ROM checks +0x0c, so honour write
     * protection here: hand it back and the ROM fails it as sense 0x27. */
    if (opcode == 0x2a && usb_msc_write_protected())
        return 0;

    msc_hooks++;
    yp3_usblog.op = opcode | ((uint32_t)MSC_STATE << 8);
    usblog_stage(UL_HOOK);

    if (MSC_STATE == MSC_ST_IDLE) {
        if (!usb_msc_begin()
                /* The ROM's WRITE(10) checks these two (0x5f80, 0x601e);
                 * delegate so it fails them with the right sense. */
                || (opcode == 0x2a && ((MSC_CBW_FLAGS & 0x80)
                        || MSC_CBW_LEN != msc_blocks_left * 512u))) {
            msc_rejects++;
            usblog_stage(UL_REJECT);
            return 0;                   /* let the ROM fail it properly */
        }

        if (opcode == 0x2a) {
            yp3_usblog.lba = msc_lba;
            yp3_usblog.blocks = msc_blocks_left;
            yp3_usblog.pkts = 0;
            yp3_usblog.secs = 0;
            yp3_usblog.wr_begun++;
            usblog_stage(UL_W_CBW);
            if (++msc_wr_logged <= MSC_LOG_CMDS)
                logf("msc: W lba=%lu n=%lu", (unsigned long)msc_lba,
                     (unsigned long)msc_blocks_left);
            else if (msc_wr_logged == MSC_LOG_CMDS + 1)
                logf("msc: further writes counted in yp3_usblog only");
            /* The first packet has not arrived: arm it and return. 0x63b8
             * then sees state 1 and returns too (0x6428..0x6442). */
            usb_msc_write_start();
            return 1;
        }

        usblog_stage(UL_R_CBW);
        MSC_STATE = MSC_ST_DATA_IN;
        msc_rd_inflight = false;
        msc_rd_ahead = 0;
        msc_rd_cur = 0;
    }

    if (opcode == 0x28) {
        usb_msc_read_chunk();
#if YP3_USB_WRITE_FIX && YP3_USB_WRITE_DMA
    } else if (1) {
        /* The data comes through usb_msc_dma_cb, not through here: this
         * is 0x6454 after an endpoint interrupt nobody asked for. */
        USBSTAT(yp3_usbdma.wd_spur++);
#endif
    } else {
#if YP3_USB_WRITE_FIX && YP3_USB_STATS
        static uint32_t t_exit;
        static bool have_exit;
        uint32_t t_in = usb_syst();

        if (have_exit && msc_wr_bytes != 0) {
            uint32_t gap = (t_exit >= t_in) ? t_exit - t_in
                         : t_exit + (REG32(0xE000E014) & 0xffffffu) + 1 - t_in;
            yp3_usbdma.pk_gap += gap >> 8;
            if (gap > yp3_usbdma.pk_gap_max)
                yp3_usbdma.pk_gap_max = gap;
        }
        usb_msc_write_packet();
        yp3_usbdma.pk_n++;
        yp3_usbdma.pk_in += usb_syst_since(t_in) >> 8;
        t_exit = usb_syst();
        have_exit = (msc_wr_bytes != 0);
#else
        usb_msc_write_packet();
#endif
    }

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
    const void *inquiry;                                /* +0x1c */
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
    /* NOT a function. ROM 0x599c is the INQUIRY handler and it treats +0x1c
     * as a pointer to 36 bytes of INQUIRY response, which it memcpys into
     * the staging buffer:
     *
     *     r3 = [0x801068]; r3 = [r3 + 0x1c];
     *     if (r3 == 0) r3 = 0x800588;      the ROM's own default
     *     [0x800dee] = 36; copy 36 bytes r3 -> 0x800df0
     *
     * docs/USB.md called this slot unused and the vendor's a "return 0"; the
     * vendor's is 0x81ac43, a data address, and ours was usb_msc_stub. The
     * ROM duly sent 36 bytes of that function's Thumb code as the INQUIRY
     * response, which the host read as vendor "ppGP", product "1N F F0H 4x"
     * and peripheral-qualifier 1 - no medium, so no block device.
     *
     * NULL takes the ROM's default, which is the SMTLINK/DEVICE response
     * that download mode enumerates with. Point it at our own 36 bytes when
     * there is a reason to; there is no reason yet. */
    .inquiry         = NULL,
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
     * moves under us. The third, 0x3c2c, was taken to be USB-specific:
     *
     *     gpio_cfg(0x00020100)
     *     gpio_cfg(0x0002090b)          the vendor's copy uses 0x00020908
     *     [0x40085100] |= 0x80000000;   reset, then poll until it clears
     *     [0x40085100] = 0x0001b082;    the vendor writes 0x0001b080
     *
     * It is not. 0x40085100 is the PMU MAILBOX's bus configuration, one word
     * below its command register, and 0x0001b082 runs that bus at half the
     * vendor's rate - every PMU transaction from then on cost ~3.2 ms, with
     * interrupts masked [L]. Whether USB needs this call at all is [U]; it
     * stays because every working USB session so far has had it, and
     * yp3_pmu_bus_init() puts the vendor's word back straight after. */
    yp3_pmu_bus_init();

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

/* Teardown, FIRM 0xd66c3a: stop the clock, then drop the module.
 *
 * We stop the clock and LEAVE THE MODULE ON, and the vendor does the same:
 * 0x23 is on in probe/clk80_work.bin, the gate capture taken with its firmware
 * running. Stopping the clock is enough to quiesce the controller.
 *
 * The reason this comment used to give was wrong. It said 0x23 gates "the
 * audio block at 0x40040000" as well as USB, from a probe that walked nine
 * candidates and found registers at 0x40040300 holding a value only once 0x23
 * was enabled. Those registers are the USB mass-storage controller's, not
 * audio's - the probe found the USB gate and was reading USB registers back.
 * Audio is module 0x25 at 0x40009000 and is unaffected by anything in this
 * file (docs/AUDIO.md). Leaving the module on is still right; it is just USB's
 * own business. */
static void __attribute__((section(".icode"), noinline))
usb_hw_disable(void)
{
    ROM_CLK_STOP(USB_CLOCK);
}


/* --- the Rockbox side ------------------------------------------------------ */

void usb_init_device(void)
{
    usb_exposed = false;

    /* Whatever the last run's USB session left in .persist - after a hang
     * and a pinhole reset, this is the line that says where it stopped. A
     * power-on leaves garbage, which the magic rejects. */
    if ((yp3_usblog.stage & USBLOG_MASK) == USBLOG_MAGIC
            && (yp3_usblog.stage & ~USBLOG_MASK) != UL_BOOT)
        usblog_report("prev");
    memset((void *)&yp3_usblog, 0, sizeof(yp3_usblog));
    usblog_stage(UL_BOOT);
}

/* usb.c's non-usbstack slave mode has already unmounted the disk, reset
 * storage and called storage_enable(false) by the time this runs, so the card
 * is ours to hand over. */
/* The controller registers, for the probe below. The ROM's USB code reaches
 * 0x40040001 (0x5204), 0x4004000b (0x5572), 0x4004000e and 0x40040012
 * (0x5118..0x5136). pcm-yp3box.c used to write this same page believing it was
 * audio DMA; it no longer does, and this page belongs entirely to USB. */
#define USBC(o)             REG8(0x40040000u + (o))

#ifdef ROCKBOX_HAS_LOGF
static void usb_log_state(const char *when)
{
    logf("usb %s: mod23=%d clk sys40000000=%08lx", when,
         ROM_CLK_IS_ON(USB_MODULE), (unsigned long)USB_SYS_ENABLE);
    /* 01 is POWER: 0x20 is high-speed enable, 0x40 is SOFTCONN - the D+
     * pull-up, and the difference between a device the host can see and one
     * it cannot. 60 is DEVCTL; its bit 0 is SESSION. */
    logf("usb %s: 00=%02x 01=%02x 0b=%02x 0e=%02x 12=%04x 60=%02x", when,
         USBC(0x00), USBC(0x01), USBC(0x0b), USBC(0x0e),
         REG16(0x40040012), USBC(0x60));
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
        msc_t0 = current_tick;
        msc_wr_logged = 0;
        memset((void *)&yp3_usblog, 0, sizeof(yp3_usblog));
        usblog_stage(UL_BOOT);

        usb_log_state("pre");
        usb_hw_enable();
        usb_log_state("post-clk");
        usb_probe_regs();

        /* Quiesce the ROM's own USB interrupts before re-attaching.
         *
         * A rig boot is entered by `exec`, which the ROM dispatches from
         * INSIDE its USB interrupt (0x627a -> 0x61d4), so the ROM's stack is
         * mid-command and the controller may hold latched events. Attach
         * (0x58f2) enables IRQ 41 and 42 (0x5532..0x5550); the first run of
         * this path never returned from it while the tick kept running,
         * which is the shape of an interrupt that fires and never clears.
         * So mask both lines, drop anything pending in the NVIC, and read the
         * MUSB interrupt latches, which clear on read: INTRTX +0x02,
         * INTRRX +0x04 (16-bit), INTRUSB +0x0a (8-bit). A normal boot has
         * nothing latched here, so this is a no-op for it. */
        {
            uint16_t itx, irx;
            uint8_t iusb;

            REG32(0xE000E184) = (1u << (41 - 32)) | (1u << (42 - 32)); /* ICER1 */
            REG32(0xE000E284) = (1u << (41 - 32)) | (1u << (42 - 32)); /* ICPR1 */
            itx = REG16(0x40040002);
            irx = REG16(0x40040004);
            iusb = USBC(0x0a);
            logf("usb: latched tx=%04x rx=%04x usb=%02x, irq41/42 masked",
                 itx, irx, iusb);
            (void)itx; (void)irx; (void)iusb;   /* reads ARE the clear */
        }

        /* And endpoint 0 itself. Measured 2026-11-15 on an exec boot [L]:
         * attach never returned, and the stall watch put the thread in ROM
         * 0x4d06 with lr 0x4d3d - the first call of the endpoint sweep at
         * 0x4d34, `INDEX = 0; CSR |= 8; while (CSR & 1) ;`. On endpoint 0
         * bit 0 of the CSR is RxPktRdy, and nothing in that loop can clear
         * it: the exec request that started this image is still sitting in
         * the FIFO, because the ROM's handler for it never returned to
         * acknowledge it. Acknowledge it here (ServicedRxPktRdy, bit 6), and
         * flush (bit 8) if that is not enough. Bounded; a normal boot has
         * bit 0 clear and skips all of it. Run 2026-11-15 [L]: "ep0 csr0 was
         * 0001, now 0000 (0 spins)", and attach has returned on every exec
         * boot since. */
        {
            uint8_t idx = USBC(0x0e);
            uint16_t csr0;
            unsigned spins = 0;

            USBC(0x0e) = 0;
            csr0 = REG16(0x40040012);
            if (csr0 & 1) {
                REG16(0x40040012) = 0x0040;
                while ((REG16(0x40040012) & 1) && ++spins < 100000)
                    ;
                if (REG16(0x40040012) & 1) {
                    REG16(0x40040012) = 0x0100;
                    spins = 0;
                    while ((REG16(0x40040012) & 1) && ++spins < 100000)
                        ;
                }
                logf("usb: ep0 csr0 was %04x, now %04x (%u spins)", csr0,
                     REG16(0x40040012), spins);
            }
            USBC(0x0e) = idx;
        }

        usb_exposed = true;
        ROM_USB_SET_USERFN(&usb_msc_ops);
        logf("usb: attach...");
        ROM_USB_ATTACH();
        usb_log_state("post-attach");
        /* Only now is there anything for the host to talk to: attach has
         * registered the interrupts and left the pull-up down. */
        ROM_USB_CONNECT();
        usb_log_state("post-connect");
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
        usblog_stage(UL_ATTACHED);
        logf("usb: write path %s", !YP3_USB_WRITE_FIX ? "OLD (no arm)"
             : YP3_USB_WRITE_DMA ? "DMA receive" : "0x5944 pump");
    } else {
        if (!usb_exposed)
            return;

        logf("msc: hooks=%lu chunks=%lu rej=%lu sderr=%lu run=%lu",
             (unsigned long)msc_hooks, (unsigned long)msc_chunks,
             (unsigned long)msc_rejects, (unsigned long)msc_sderrs,
             (unsigned long)msc_runaways);
        {
            long dt = current_tick - msc_t0;
            /* KB/s: chunks are 512 bytes, and HZ ticks make a second. */
            unsigned long kbs = dt > 0
                ? (unsigned long)((uint64_t)msc_chunks * 512 * HZ / dt / 1024)
                : 0;
            logf("msc: lastlba=%lu left=%lu state=%d",
                 (unsigned long)msc_last_lba, (unsigned long)msc_blocks_left,
                 MSC_STATE);
            logf("msc: ticks=%ld hz=%d -> %lu KB/s", dt, HZ, kbs);
        }
        usblog_report("detach");
        usblog_stage(UL_DETACHED);
        msc_hooks = msc_chunks = msc_rejects = msc_sderrs = 0;
        msc_runaways = 0;

        usb_exposed = false;
        /* Drop the pull-up before the clock goes away, so the host sees a
         * disconnect rather than a device that stopped answering. */
        ROM_USB_DISCONNECT();
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

    /* A rig build always has the cable in - it is how the host flashes and
     * reads the ring, all through the boot ROM, never through Rockbox. Left
     * alone, VBUS sends every rig boot straight into mass-storage mode and
     * the UI and playback the rig exists to exercise never run. */
    if (YP3_RIG_SECONDS && !YP3_RIG_USB)
        return USB_EXTRACTED;

    if (TIME_AFTER(current_tick, next_poll)) {
        next_poll = current_tick + HZ / 10;
#if YP3_USB_STATS
        /* A read chunk handed to the ROM 3 s ago and no completion since:
         * take the channel's registers once, from here, because the hook
         * will never run again to do it. */
        if (yp3_usblog.stage == (USBLOG_MAGIC | UL_R_SENT)
                && MSC_STATE != MSC_ST_IDLE     /* idle = the CSW went out */
                && TIME_AFTER(current_tick, (long)yp3_usblog.tick + 3 * HZ)
                && yp3_usbdma.stall_tick != yp3_usblog.tick) {
            struct yp3_usbdma *d = &yp3_usbdma;
            int old = disable_irq_save();
            uint8_t idx = USB_INDEX;
            USB_INDEX = 1;
            d->stall_txcsr = USB_TXCSR_IDX;
            USB_INDEX = idx;
            d->stall_cntl = USB_DMA_CNTL;
            d->stall_addr = USB_DMA_ADDR;
            d->stall_count = USB_DMA_COUNT;
            restore_irq(old);
            d->stall_state = MSC_STATE;
            d->stall_buf = (uint32_t)msc_rd_prev;
            d->stall_tick = yp3_usblog.tick;
            d->stalls++;
            logf("usb: READ STALL cntl=%04lx addr=%08lx cnt=%lu txcsr=%04lx st=%lu",
                 (unsigned long)d->stall_cntl, (unsigned long)d->stall_addr,
                 (unsigned long)d->stall_count, (unsigned long)d->stall_txcsr,
                 (unsigned long)d->stall_state);
        }
#endif
        detected = (power_input_status() != POWER_INPUT_NONE)
                 ? USB_INSERTED : USB_EXTRACTED;
    }

    return detected;
}
