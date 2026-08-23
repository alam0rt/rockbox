/* SD driver for the SL6801.
 *
 * Reversed from the vendor bootloader's sdmmc_wrap_init (SRAM 0x8224d4) and the
 * SD HAL that lives in the boot ROM. The controller is NOT an STM32 SDIO block
 * despite the vendor's ST-style "HAL_SD_Init_new" naming - the register map
 * below comes from the ROM accessors themselves:
 *
 *   0x000 CTRL      bits 0-2 self-clearing reset (ROM 0x3d58)
 *                   bit  2   + bit 25 enable internal DMA (ROM 0x46f4)
 *   0x004 CMD       bit 31 = start/busy (ROM 0x40a2)
 *   0x008 ARG       (ROM 0x40a2)
 *   0x00c BLKSIZE   (ROM 0x40b6)
 *   0x010 DATALEN   (ROM 0x40b6)
 *   0x014 DTIMER    (ROM 0x40b6)
 *   0x034 STATUS    write-1-to-clear (ROM 0x419a)
 *   0x038 RESP0..3  at 0x38/0x3c/0x40/0x44 (ROM 0x4186: base+(i+14)*4)
 *   0x048 FIFOSTA   bit 2 = RX empty, bit 3 = TX full (ROM 0x4602/0x4610)
 *                   bits 11:16 = index of the responding command (ROM 0x4190)
 *   0x04c CLKCFG    (ROM 0x3d6a)
 *   0x200 FIFO      read and write data port (ROM 0x4602/0x4610)
 *
 * The previous revision of this file assumed the documented STM32 layout, which
 * put ARG at 0x08 and CMD at 0x0c - it would have written the argument into the
 * command register. It could never have worked.
 *
 * Commands go through the ROM's own send/wait helpers rather than open-coded
 * register writes: ROM 0x40c4 owns the table mapping a command index to its
 * controller-specific command word (response length, data direction and so on),
 * and reimplementing that table would be guesswork for no gain.
 *
 * Polled, no DMA and no interrupts. The goal is a correct read, not throughput.
 */
#include "config.h"
#include "system.h"
#include "sd.h"
#include "storage.h"
#include "sdmmc.h"
#include "breadcrumb.h"
#include <string.h>
#include <stdint.h>

#define SD_BASE     0x40003000u
#define R(o)        (*(volatile uint32_t *)(SD_BASE + (o)))

#define SD_CTRL     0x000
#define SD_BLKSIZE  0x00c
#define SD_DATALEN  0x010
#define SD_DTIMER   0x014
#define SD_REG18    0x018
#define SD_DCTRL    0x02c
#define SD_STA      0x034
#define SD_FIFOSTA  0x048
#define SD_CLKCFG   0x04c
#define SD_FIFO     0x200

/* STATUS bits, from the ROM's error decoding (0x419a, 0x4bc0, 0x4794) */
#define STA_DONE      (1u << 2)    /* command/response complete   */
#define STA_DATAEND   (1u << 3)    /* data block finished (ROM 0x488c) */
#define STA_RXREADY   (1u << 5)    /* RX FIFO has a burst ready   */
#define STA_CRCFAIL   (1u << 6)
#define STA_TIMEOUT   (1u << 8)
#define STA_ERRMASK   0x300u       /* the pair the ROM treats as fatal */
#define STA_ALLERR    0xbfc2u      /* every error bit, no DONE    */
#define STA_ALLFLAGS  0xbfc6u      /* clear-all                   */

/* Clock/module ids: the vendor resets module 0x24 and drives clock 0x11. */
#define SD_MODULE   0x24u
#define SD_CLOCK    0x11u
#define SD_DIV_ID   64u            /* identification-speed divider (vendor: 0x40) */

/* Boot ROM SD HAL. All of these take a handle whose first word is the base. */
#define ROM_SD_CMD      ((void     (*)(void *, unsigned, uint32_t))0x40c5u)
#define ROM_SD_RESP     ((uint32_t (*)(void *, unsigned))0x4187u)
#define ROM_SD_RESPCMD  ((uint32_t (*)(void *))0x4191u)
#define ROM_SD_WAIT     ((uint32_t (*)(void *, unsigned))0x4bc1u)
/* One wait per response type. The error masks genuinely differ, because the
 * response formats differ - this is not redundancy in the ROM:
 *   CMD0  0x4234  no error check at all, just a bounded wait for STA bit 2
 *   R7    0x425a  mask 0xbfc2
 *   R3    0x429a  mask 0xbf80  <- excludes bit 6 (CRC): R3 carries NO CRC
 *   R2    0x43c4  mask 0xbfc2
 *   R6    0x4400  mask 0xbfc2, checks the response index, returns RCA
 * Using one generic wait for all of them rejected a perfectly good ACMD41 on a
 * CRC error that does not exist in an R3 response. */
#define ROM_SD_WAIT_CMD0 ((uint32_t (*)(void *))0x4235u)
#define ROM_SD_WAIT_R7   ((uint32_t (*)(void *))0x425bu)
#define ROM_SD_WAIT_R3   ((uint32_t (*)(void *))0x429bu)
#define ROM_SD_WAIT_R2   ((uint32_t (*)(void *))0x43c5u)
#define ROM_SD_WAIT_R6   ((uint32_t (*)(void *, unsigned, uint16_t *))0x4401u)
#define ROM_SD_WAIT_IDX  ((uint32_t (*)(void *, unsigned))0x419bu)
#define ROM_SD_SELECT    ((uint32_t (*)(void *, uint32_t))0x4283u)
#define ROM_SD_RESET    ((void     (*)(void *))0x3d59u)
#define ROM_SD_CFG      ((void (*)(void *, uint32_t, uint32_t, uint32_t, \
                                   uint32_t, uint32_t, uint32_t))0x3d6bu)
#define ROM_SD_RD512    ((uint32_t (*)(void *, void *))0x4795u)
#define ROM_FIFO_WR     ((void (*)(void *, const uint32_t *))0x4611u)
#define ROM_UDELAY      ((void (*)(unsigned))0xb765u)
#define ROM_CLK_STOP    ((void (*)(unsigned))0x2a7du)
#define ROM_CLK_DIV     ((void (*)(unsigned, unsigned))0x2d59u)
#define ROM_CLK_APPLY   ((void (*)(unsigned))0x27a1u)
#define ROM_CLK_ENABLE  ((void (*)(unsigned))0x2565u)
#define ROM_CLK_DISABLE ((void (*)(unsigned))0x2571u)
#define ROM_GPIO_CFG1   ((void (*)(unsigned))0x7adu)

/* Every ROM entry point used here dereferences only handle[0] (verified by
 * reading each one). The vendor's handle is 0xcc bytes and some ROM paths we do
 * NOT call reach as far as handle[0x8c] for a callback pointer, so the struct is
 * NOT call (ROM 0x3e04/0x3e12) reach handle[0x8c] for a callback pointer. Those
 * are never invoked from here, so one word is sufficient; giving the struct the
 * vendor's full 0xcc bytes overflows the LOW region by 64 bytes. */
static struct { volatile uint32_t *base; } sdh;

/* CLK, CMD, DAT0-3 on port 3 pins 4..9, alt mode 11 (see docs/SD.md). */
static const unsigned sd_pins[] = {
    0x0003258bu, 0x00032d8bu, 0x0003358bu,
    0x00033d8bu, 0x0003458bu, 0x00034d8bu,
};

static bool     sd_ok;
static uint32_t sd_rca;
static uint32_t sd_cardtype;         /* 1 = SDv2 byte-addressed, 2 = SDHC/SDXC */
static uint32_t sd_numblocks;
static tCardInfo sd_card;

/* Do NOT use the ROM's udelay (0xb764) here. It spins until a 64-bit counter at
 * 0x40082100 advances past a target, and that timer is not running under
 * Rockbox - the ROM starts it during its own boot and we never do. The first
 * ROM_UDELAY(5) call hung the device between breadcrumbs 1 and 2, which the
 * watchdog then turned into a boot loop. The LCD driver already busy-loops for
 * the same reason.
 *
 * SPIN_PER_US is deliberately generous: over-delaying during card
 * identification costs nothing, under-delaying breaks it. */
/* Clock changes and delay loops run from SRAM. Executed from XIP flash, a
 * clk_apply that perturbs the flash controller stalls the CPU mid-loop, and a
 * calibrated delay is not calibrated at all when every iteration waits on a SPI
 * fetch. The LCD hit both; SD has the same shapes. */
#define SDICODE __attribute__((section(".icode"), noinline))

#define SPIN_PER_US 80u

static void SDICODE sd_busywait(uint32_t n) { volatile uint32_t i; for (i = 0; i < n; i++) ; }
static void SDICODE sd_udelay(unsigned us) { sd_busywait(us * SPIN_PER_US); }
static void SDICODE sd_mdelay(unsigned ms) { while (ms--) sd_udelay(1000); }

/* clk_stop / set divider / settle / apply - vendor callback at SRAM 0x8223c8 */
static void SDICODE sd_module_reset(void)
{
    ROM_CLK_DISABLE(SD_MODULE);
    sd_udelay(5);
    ROM_CLK_ENABLE(SD_MODULE);
}

static void SDICODE sd_set_clock(unsigned div)
{
    ROM_CLK_STOP(SD_CLOCK);
    ROM_CLK_DIV(SD_CLOCK, div);
    sd_mdelay(100);
    ROM_CLK_APPLY(SD_CLOCK);
}



/* Faithful port of the vendor's own R1 wait (SRAM 0x828796), used for CMD55.
 *
 * NOT ROM 0x419a. That routine looks like a generic index-checking wait, but its
 * tail returns 0 only when RESP0 bit 31 is SET and 41 otherwise:
 *
 *     ldr r3,[r3,#56]   ; RESP0
 *     cmp r3,#0
 *     ite lt
 *     movlt r0,#0       ; success only if bit31 set
 *     movge r0,#41
 *
 * CMD55's R1 normally has bit 31 clear, so 0x419a reports failure on a perfectly
 * good response - observed as "CMD55 rejected" with ACMD41 already fixed. The
 * vendor never uses 0x419a here; it uses this, which checks the flags and the
 * response index and then simply returns 0.
 */
static uint32_t SDICODE sd_wait_r1_idx(unsigned idx)
{
    uint32_t sta;

    while (!(R(SD_STA) & STA_DONE))
        ;

    if (R(SD_STA) & STA_ERRMASK)  { return 32; }
    sta = R(SD_STA);
    if (sta & STA_TIMEOUT) { R(SD_STA) = STA_TIMEOUT; return 3; }
    sta = R(SD_STA) & STA_CRCFAIL;
    if (sta)               { R(SD_STA) = STA_CRCFAIL; return 1; }

    if (ROM_SD_RESPCMD(&sdh) != idx)
        return 16;

    R(SD_STA) = STA_DONE | STA_ERRMASK;
    return 0;
}

/* CMD0, CMD8 and the ACMD41 negotiation - vendor sd_power_on at 0x8288c4. */
static uint32_t sd_power_on(void)
{
    uint32_t arg41 = 0x00100000u;    /* OCR voltage window */
    uint32_t ocr;
    int tries;

    sd_cardtype = 0;
    R(SD_STA) = STA_ALLFLAGS;

    ROM_SD_CMD(&sdh, 0, 0);                  /* GO_IDLE_STATE */
    if (ROM_SD_WAIT_CMD0(&sdh))
        return 2;

    ROM_SD_CMD(&sdh, 8, 0x1aau);             /* SEND_IF_COND */
    if (ROM_SD_WAIT_R7(&sdh) == 0) {
        sd_cardtype = 1;                     /* v2.00 or later */
        arg41 |= 0x40000000u;                /* host supports high capacity */
    }
    arg41 |= 0x80000000u;

    /* The vendor allows 500ms; ACMD41 is polled until the card clears busy. */
    for (tries = 0; tries < 600; tries++) {   /* vendor allows 500 ms */
        ROM_SD_CMD(&sdh, 55, 0);             /* APP_CMD */
        if (sd_wait_r1_idx(55))
            return 4;

        ROM_SD_CMD(&sdh, 41, arg41);         /* SD_SEND_OP_COND, replies R3 */
        if (ROM_SD_WAIT_R3(&sdh))
            return 5;

        ocr = ROM_SD_RESP(&sdh, 0);
        if (ocr & 0x80000000u) {             /* power-up complete */
            if (ocr & 0x40000000u)
                sd_cardtype = 2;             /* CCS set: block addressed */
            return 0;
        }
        sd_mdelay(1);
    }
    return 27;                               /* vendor's timeout code */
}

/* CMD2 / CMD3 / CMD9 - vendor 0x828a20. */
static uint32_t sd_identify(void)
{
    uint32_t csd[4];
    uint32_t csize;

    uint16_t rca16 = 0;

    ROM_SD_CMD(&sdh, 2, 0);                  /* ALL_SEND_CID, replies R2 */
    if (ROM_SD_WAIT_R2(&sdh))
        return 6;

    ROM_SD_CMD(&sdh, 3, 0);                  /* SEND_RELATIVE_ADDR, replies R6 */
    if (ROM_SD_WAIT_R6(&sdh, 3, &rca16))
        return 7;
    sd_rca = rca16;

    ROM_SD_CMD(&sdh, 9, sd_rca << 16);       /* SEND_CSD, replies R2 */
    if (ROM_SD_WAIT_R2(&sdh))
        return 8;

    /* For a long (R2) response, index 3 is the MOST significant word:
     *   RESP3 = CSD[127:96], RESP2 = CSD[95:64], RESP1 = [63:32], RESP0 = [31:0]
     *
     * This was originally assumed the other way round, because for SHORT
     * responses index 0 does hold the response - ROM 0x4400 reads it for CMD3's
     * RCA and that worked, which made the assumption look confirmed.
     *
     * Verified against a real card: RESP3 = 0x400e0032 gives CSD_STRUCTURE 1,
     * TAAC 0x0e, TRAN_SPEED 0x32, and RESP2 = 0x5b590000 gives CCC 0x5b5 and
     * READ_BL_LEN 9 - every field where it belongs. The vendor agrees: it stores
     * resp(3) at the LOWEST offset of its CSD array (SRAM 0x828a20). */
    csd[3] = ROM_SD_RESP(&sdh, 3);
    csd[2] = ROM_SD_RESP(&sdh, 2);
    csd[1] = ROM_SD_RESP(&sdh, 1);
    csd[0] = ROM_SD_RESP(&sdh, 0);

    sd_card.csd[0] = csd[3];     /* Rockbox stores csd[0] as the most
                                  * significant word, matching the vendor's
                                  * own layout at SRAM 0x828a20 */
    sd_card.csd[1] = csd[2];
    sd_card.csd[2] = csd[1];
    sd_card.csd[3] = csd[0];

    /* Record the four response words raw. sd_numblocks came out exactly 0 on the
     * first successful init, which in the v1.0 branch is what happens when
     * READ_BL_LEN decodes small enough that blocklen/512 truncates to zero - i.e.
     * the v2.0 test below did not match and the word order is not what was
     * assumed. Decode these offline rather than guessing the layout again. */
    BC_SLOT(46) = csd[3];                    /* RESP0 */
    BC_SLOT(47) = csd[2];                    /* RESP1 */
    BC_SLOT(48) = csd[1];                    /* RESP2 */
    BC_SLOT(49) = csd[0];                    /* RESP3 */

    if ((csd[3] >> 30) == 1) {               /* CSD version 2.0 */
        csize = ((csd[2] & 0x3fu) << 16) | (csd[1] >> 16);
        sd_numblocks = (csize + 1) * 1024u;
    } else {                                 /* version 1.0 */
        uint32_t mult, blocklen;
        csize    = ((csd[2] & 0x3ffu) << 2) | (csd[1] >> 30);
        mult     = 1u << (((csd[1] >> 15) & 7u) + 2u);
        blocklen = 1u << ((csd[2] >> 16) & 0xfu);
        sd_numblocks = (csize + 1) * mult * (blocklen / 512u);
    }
    return 0;
}

void sd_enable(bool on) { (void)on; }
bool sd_removable(IF_MD_NONVOID(int drive)) { IF_MD((void)drive;) return true; }
bool sd_present(IF_MD_NONVOID(int drive))   { IF_MD((void)drive;) return sd_ok; }
long sd_last_disk_activity(void) { return 0; }
int  sd_event(long id, intptr_t data) { (void)id; (void)data; return 0; }
/* sdmmc.h declares this as returning tCardInfo *. It was previously defined here
 * as void(int, void *), which only compiled because sdmmc.h was not included -
 * any caller would have gone wrong. */
tCardInfo *card_get_info_target(int card_no)
{
    (void)card_no;
    return &sd_card;
}

int sd_init(void)
{
    unsigned i;
    uint32_t err;

    BC_SLOT(32) = 1;

    /* Module reset pulse, exactly as sdmmc_wrap_init does it. Runs from SRAM:
     * ROM clk_enable spins on an acknowledge bit and must not be executed from
     * flash. */
    sd_module_reset();

    for (i = 0; i < sizeof(sd_pins) / sizeof(sd_pins[0]); i++)
        ROM_GPIO_CFG1(sd_pins[i]);

    sdh.base = (volatile uint32_t *)SD_BASE;

    sd_set_clock(SD_DIV_ID);
    BC_SLOT(32) = 2;

    ROM_SD_RESET(&sdh);
    /* Vendor constants from HAL_SD_Init_new (0x828b54): the trailing three are
     * stack arguments landing in CLKCFG and the 0x64 register. */
    ROM_SD_CFG(&sdh, 0, 0, 0x20000000u, 8, 7, 0xfffu);

    R(SD_REG18) &= ~1u;
    R(SD_DTIMER) = 0xffffff40u;
    R(SD_CTRL)   = 0x40000000u;

    {   /* Is the ROM's timer running? Costs nothing to find out. */
        volatile uint32_t *tmr = (volatile uint32_t *)0x40082104u;
        uint32_t t0 = *tmr;
        sd_busywait(20000);
        BC_SLOT(43) = *tmr - t0;     /* 0 = frozen, confirming the hang above */
    }
    BC_SLOT(33) = R(SD_STA);
    BC_SLOT(34) = R(SD_CLKCFG);
    BC_SLOT(32) = 3;

    err = sd_power_on();
    BC_SLOT(35) = err ? (0xE0000000u | err) : sd_cardtype;
    if (err)
        return -1;

    sd_udelay(30);
    BC_SLOT(32) = 4;

    err = sd_identify();
    BC_SLOT(36) = err ? (0xE0000000u | err) : sd_rca;
    BC_SLOT(37) = sd_numblocks;
    if (err)
        return -2;

    /* ROM 0x4282 sends CMD7 and waits, in one call. */
    if (ROM_SD_SELECT(&sdh, sd_rca << 16)) { BC_SLOT(38) = 0xE0000007u; return -3; }

    ROM_SD_CMD(&sdh, 16, 512);               /* SET_BLOCKLEN */
    if (ROM_SD_WAIT(&sdh, 16)) { BC_SLOT(38) = 0xE0000010u; return -4; }

    sd_card.initialized = 1;
    sd_card.numblocks   = sd_numblocks;
    sd_card.blocksize   = 512;
    sd_card.rca         = sd_rca;
    sd_card.sd2plus     = (sd_cardtype >= 1);

    BC_SLOT(32) = 5;
    BC_SLOT(38) = 0x5D000000u;               /* init complete */
    sd_ok = true;
    return 0;
}

/* Bounded equivalents of ROM 0x4bc0 and 0x4794.
 *
 * Both ROM routines spin with NO timeout: 0x4bc0 waits forever for STA bit 2 and
 * 0x4794 waits forever for the RX burst flag. That is fine for the vendor, whose
 * watchdog would reset the device, but here it hung disk_mount_all with no way to
 * tell a hang from a failure. These mirror the ROM's checks exactly - including
 * the R1 error mask 0xfdffe008 from the pool at ROM 0x4cb0 - but give up rather
 * than spin.
 */
/* From SRAM this is a real bound, not a flash-fetch crawl. 4 million iterations
 * per wait, executed from flash, across the reads a FAT mount performs, is
 * minutes - which looks exactly like a hang. */
#define SD_SPIN 400000u

static uint32_t SDICODE sd_wait_data(unsigned idx)
{
    uint32_t sta, t = 0;

    while (!(R(SD_STA) & STA_DONE))
        if (++t > SD_SPIN)
            return 0x7f;                     /* our own code: wait timed out */

    sta = R(SD_STA);
    if (sta & STA_ERRMASK) { R(SD_STA) = STA_ERRMASK; return 32; }
    if (sta & STA_TIMEOUT) { R(SD_STA) = STA_TIMEOUT; return 3; }
    if (sta & STA_CRCFAIL) { R(SD_STA) = STA_CRCFAIL; return 1; }

    if (ROM_SD_RESPCMD(&sdh) != idx)
        return 16;

    R(SD_STA) = STA_DONE | STA_ERRMASK;
    if (ROM_SD_RESP(&sdh, 0) & 0xfdffe008u)  /* R1 error bits, ROM pool 0x4cb0 */
        return 33;
    return 0;
}

static uint32_t SDICODE sd_fifo_word(uint32_t *out)
{
    uint32_t t = 0;
    while (R(SD_FIFOSTA) & (1u << 2))        /* bit 2 set = RX empty (ROM 0x4602) */
        if (++t > SD_SPIN)
            return 0x7e;
    *out = R(SD_FIFO);
    return 0;
}

static uint32_t SDICODE sd_read_fifo512(void *buf)
{
    uint8_t *p = buf, *end = p + 512;
    uint32_t t, w, err;

    while (p < end) {
        uint32_t sta;
        t = 0;
        for (;;) {
            sta = R(SD_STA);
            if (sta & STA_ERRMASK) { R(SD_STA) = STA_ERRMASK; return 32; }
            if (sta & STA_RXREADY) break;
            if (++t > SD_SPIN) return 0x7d;
        }
        /* ROM 0x4794 drains eight words per burst, then clears the flag. */
        for (t = 0; t < 8 && p < end; t++) {
            if ((err = sd_fifo_word(&w)) != 0)
                return err;
            memcpy(p, &w, 4);                /* buf is not guaranteed aligned */
            p += 4;
        }
        R(SD_STA) = STA_RXREADY;
    }
    return 0;
}

/* Byte address for standard-capacity cards, block address for SDHC/SDXC -
 * the same test the ROM makes at 0x474e. */
static uint32_t sd_addr(sector_t sector)
{
    return (sd_cardtype == 2) ? (uint32_t)sector : (uint32_t)(sector << 9);
}

static void sd_setup_data(uint32_t len)
{
    R(SD_DTIMER)  = 0xffffff40u;
    R(SD_BLKSIZE) = 512;
    R(SD_DATALEN) = len;
}

int SDICODE sd_read_sectors(IF_MD(int drive,) sector_t start, int count, void *buf)
{
    uint8_t *p = buf;


    IF_MD((void)drive;)
    if (!sd_ok)
        return -1;

    while (count--) {
        uint32_t err;

        R(SD_STA) = STA_ALLFLAGS;
        R(SD_DCTRL) |= 8u;                       /* vendor does this first, 0x8227f8 */
        sd_setup_data(512);

        ROM_SD_CMD(&sdh, 17, sd_addr(start));    /* READ_SINGLE_BLOCK */
        if ((err = sd_wait_data(17)) != 0) {
            BC_SLOT(39) = 0xE0001100u | (err & 0xff);
            return -2;
        }
        if ((err = sd_read_fifo512(p)) != 0) {
            BC_SLOT(39) = 0xE0001200u | (err & 0xff);
            return -3;
        }
        p += 512;
        start++;
    }
    BC_SLOT(39) = 0x5D000001u;
    return 0;
}

/* Wait for the card to finish PROGRAMMING the block it was just given.
 *
 * This is ROM 0x4898, and leaving it out is why writes failed. A card takes
 * milliseconds to commit a block and rejects commands while it is busy, so the
 * next thing the filesystem asked for went to a card that was not listening.
 * The ROM polls CMD13 (SEND_STATUS) up to 200 times at 1 ms, and takes bit 8 of
 * the R1 status - READY_FOR_DATA - as the all clear.
 */
static uint32_t SDICODE sd_wait_ready(void)
{
    unsigned tries;

    for (tries = 0; tries < 200; tries++) {
        uint32_t t = 0;

        ROM_SD_CMD(&sdh, 13, sd_rca << 16);
        while (!(R(SD_STA) & STA_DONE))
            if (++t > SD_SPIN)
                return 0x7c;
        R(SD_STA) = STA_DONE | STA_ERRMASK;    /* ROM writes 0x304 here */

        if (ROM_SD_RESP(&sdh, 0) & (1u << 8))
            return 0;
        sd_mdelay(1);
    }
    return 0x29;                               /* the ROM's own timeout code */
}

/* The data half of a block write, modelled on ROM 0x483a. Two things this does
 * that simply pushing 128 words does not: it checks the data error bits between
 * words, and it waits for DATAEND and clears it afterwards. The command-response
 * wait is NOT a substitute - it reports on the command, not on the data. */
static uint32_t SDICODE sd_write_fifo512(const void *buf)
{
    const uint8_t *p = buf, *end = p + 512;
    uint32_t t;

    while (p < end) {
        uint32_t w, sta;

        t = 0;
        for (;;) {
            sta = R(SD_STA);
            if (sta & STA_ERRMASK) { R(SD_STA) = STA_ERRMASK; return 32; }
            if (sta & (1u << 4)) break;        /* TX ready, ROM 0x4850 */
            if (++t > SD_SPIN) return 0x7b;
        }
        memcpy(&w, p, 4);                      /* buf is not guaranteed aligned */
        ROM_FIFO_WR(&sdh, &w);
        p += 4;
    }

    t = 0;
    while (!(R(SD_STA) & STA_DATAEND))
        if (++t > SD_SPIN)
            return 0x7a;
    R(SD_STA) = STA_DATAEND;                   /* ROM 0x4892 writes 8 */

    if (R(SD_STA) & STA_ERRMASK) { R(SD_STA) = STA_ERRMASK; return 32; }
    return 0;
}

/* SDICODE for the same reason the read path is: this is a per-transfer hot loop
 * spinning on FIFO flags, and run from XIP flash every iteration waits on a SPI
 * fetch. It was the one data path still left in flash. */
int SDICODE sd_write_sectors(IF_MD(int drive,) sector_t start, int count, const void *buf)
{
    const uint8_t *p = buf;

    IF_MD((void)drive;)
    if (!sd_ok)
        return -1;

    while (count--) {
        uint32_t err;

        if ((err = sd_wait_ready()) != 0) {     /* card still busy from before */
            BC_SLOT(40) = 0xE0002400u | (err & 0xff);
            return -4;
        }

        R(SD_STA) = STA_ALLFLAGS;
        R(SD_DCTRL) |= 8u;
        sd_setup_data(512);

        ROM_SD_CMD(&sdh, 24, sd_addr(start));    /* WRITE_BLOCK */
        if ((err = sd_wait_data(24)) != 0) {
            BC_SLOT(40) = 0xE0002100u | (err & 0xff);
            return -2;
        }
        if ((err = sd_write_fifo512(p)) != 0) {
            BC_SLOT(40) = 0xE0002200u | (err & 0xff);
            return -3;
        }
        p += 512;
        start++;
    }

    /* Do not return while the card is still committing the last block: the very
     * next read would be issued into a busy card. */
    if (sd_wait_ready() != 0) {
        BC_SLOT(40) = 0xE0002500u;
        return -5;
    }

    BC_SLOT(40) = 0x5D000002u;
    return 0;
}
