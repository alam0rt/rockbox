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
#include <string.h>
#include <stdint.h>
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

/* Reads and writes that FAIL, only. Silence is the useful signal here: a file
 * that reads short with nothing logged puts the fault above this driver, in
 * the FAT layer, and a burst of these puts it below. Logging every transfer
 * would wrap the 6 KB ring long before the interesting one. */
#define SD_LOG_FAIL(op, lba, n, rc) \
    logf("sd: " op " fail lba=%lu n=%d rc=%d", (unsigned long)(lba), (n), (rc))

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
#define SD_DIV_ID   64u
/* Identification-speed divider (vendor: 0x40). */

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

/* ROM entry points only dereference handle[0]. */
static struct { volatile uint32_t *base; } sdh;

/* CLK, CMD, DAT0-3 on port 3 pins 4..9, alternate function 11. */
static const unsigned sd_pins[] = {
    0x0003258bu, 0x00032d8bu, 0x0003358bu,
    0x00033d8bu, 0x0003458bu, 0x00034d8bu,
};

static bool     sd_ok;
static uint32_t sd_rca;
static uint32_t sd_cardtype;         /* 1 = SDv2, 2 = SDHC/SDXC */
static uint32_t sd_numblocks;
static tCardInfo sd_card;

/* Do not use the ROM delay: its timer is not running once Rockbox owns the
 * device. A generous local delay is harmless during card identification. */
/* Clock changes and delay loops run from SRAM. Executed from XIP flash, a
 * clk_apply that perturbs the flash controller stalls the CPU mid-loop, and a
 * calibrated delay is not calibrated at all when every iteration waits on a SPI
 * fetch. The LCD hit both; SD has the same shapes. */
#define SDICODE __attribute__((section(".icode"), noinline))

#define SPIN_PER_US 80u
/* From SRAM this is a real bound, not a flash-fetch crawl. 4 million iterations
 * per wait, executed from flash, across the reads a FAT mount performs, is
 * minutes - which looks exactly like a hang. */
#define SD_SPIN 400000u

static void SDICODE sd_busywait(uint32_t n)
{
    volatile uint32_t i;
    for (i = 0; i < n; i++)
        ;
}
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



/* Faithful port of the vendor's R1 wait at SRAM 0x828796, used for CMD55. */
/* ROM 0x419a performs a generic index check and rejects a valid R1 response. */
/* Check the flags and response index exactly as the vendor does. */
/* The response flag handling below mirrors the ROM's return codes. */
static uint32_t SDICODE sd_wait_r1_idx(unsigned idx)
{
    uint32_t sta, t = 0;

    /* Bounded like every other wait in this driver. This one was not, and it
     * is the wait CMD55 uses - so a card that stopped answering APP_CMD hung
     * the storage thread here for ever, with no console to say so. The card
     * failing to answer is exactly what "sd: no card (power_on=4)" reports,
     * which means we have been one unlucky timing difference away from a
     * silent hang on the USB-eject path. */
    while (!(R(SD_STA) & STA_DONE))
        if (++t > SD_SPIN)
            return 0x7f;                     /* same code sd_wait_data uses */

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
    uint32_t ocr, err;
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
        err = sd_wait_r1_idx(55);
        if (err) {
            /* "power_on=4" collapsed four different faults into one number:
             * 0x7f the wait timing out (no clock, or no card at all), 32 a
             * controller error flag, 3 a response timeout, 1 a CRC failure -
             * which is a signal-integrity answer, not a missing-card one - and
             * 16 the card answering with the wrong index. They want different
             * fixes, so say which. */
            logf("sd: CMD55 fail sub=%lu try=%d type=%lu",
                 (unsigned long)err, tries, (unsigned long)sd_cardtype);
            return 4;
        }

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

    ROM_SD_CMD(&sdh, 3, 0);                  /* SEND_RELATIVE_ADDR */
    if (ROM_SD_WAIT_R6(&sdh, 3, &rca16))
        return 7;
    sd_rca = rca16;

    ROM_SD_CMD(&sdh, 9, sd_rca << 16);       /* SEND_CSD, replies R2 */
    if (ROM_SD_WAIT_R2(&sdh))
        return 8;

    /* For a long (R2) response, index 3 is the MOST significant word:
     * RESP3 = CSD[127:96], RESP2 = CSD[95:64], RESP1 = [63:32],
     * RESP0 = [31:0]
     *
     * This was originally assumed the other way round, because for SHORT
     * responses index 0 does hold the response - ROM 0x4400 reads it for CMD3's
     * RCA and that worked, which made the assumption look confirmed.
     *
     * Verified against a real card: RESP3 = 0x400e0032 gives CSD_STRUCTURE 1,
     * TAAC 0x0e, TRAN_SPEED 0x32, and RESP2 = 0x5b590000 gives CCC 0x5b5 and
     * READ_BL_LEN 9 - every field belongs where expected. The vendor stores
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
bool sd_present(IF_MD_NONVOID(int drive))
{
    IF_MD((void)drive;)
    return sd_ok;
}
long sd_last_disk_activity(void) { return 0; }
int  sd_event(long id, intptr_t data) { (void)id; (void)data; return 0; }
/* sdmmc.h declares this as returning tCardInfo *. Keep the target definition
 * consistent with that declaration. */
tCardInfo *card_get_info_target(int card_no)
{
    (void)card_no;
    return &sd_card;
}

int sd_init(void)
{
    unsigned i;
    uint32_t err;


    /* Module reset pulse, exactly as sdmmc_wrap_init does it. Runs from SRAM:
     * ROM clk_enable spins on an acknowledge bit and must not be executed from
     * flash. */
    sd_module_reset();

    for (i = 0; i < sizeof(sd_pins) / sizeof(sd_pins[0]); i++)
        ROM_GPIO_CFG1(sd_pins[i]);

    sdh.base = (volatile uint32_t *)SD_BASE;

    sd_set_clock(SD_DIV_ID);

    ROM_SD_RESET(&sdh);
    /* Vendor constants from HAL_SD_Init_new (0x828b54): the trailing three are
     * stack arguments landing in CLKCFG and the 0x64 register. */
    ROM_SD_CFG(&sdh, 0, 0, 0x20000000u, 8, 7, 0xfffu);

    R(SD_REG18) &= ~1u;
    R(SD_DTIMER) = 0xffffff40u;
    R(SD_CTRL)   = 0x40000000u;


    err = sd_power_on();
    if (err) {
        /* No card in the slot, or one that will not identify.
         *
         * This is NOT fatal here and must not be reported as an init failure.
         * apps/main.c turns a non-zero storage_init() into panicf("ata: %d"),
         * which is how booting this device with the card removed ends in a
         * panic screen rather than a usable player. The firmware runs from
         * flash (XIP), so everything except content is still available, and
         * Rockbox already has a graceful path for a device with no mountable
         * volume - disk_mount_all() returning <= 0 shows the info screen and
         * offers USB.
         *
         * So report success with no medium: sd_ok stays false, which is what
         * sd_read_sectors and sd_write_sectors already gate on, and the card
         * info stays zeroed so nothing believes there is a volume. */
        sd_ok = false;
        memset(&sd_card, 0, sizeof(sd_card));
        logf("sd: no card (power_on=%lu)", (unsigned long)err);
        return 0;
    }

    sd_udelay(30);

    err = sd_identify();
    if (err)
        return -2;

    /* ROM 0x4282 sends CMD7 and waits, in one call. */
    if (ROM_SD_SELECT(&sdh, sd_rca << 16))
        return -3;

    ROM_SD_CMD(&sdh, 16, 512);               /* SET_BLOCKLEN */
    if (ROM_SD_WAIT(&sdh, 16))
        return -4;

    sd_card.initialized = 1;
    sd_card.numblocks   = sd_numblocks;
    sd_card.blocksize   = 512;
    sd_card.rca         = sd_rca;
    sd_card.sd2plus     = (sd_cardtype >= 1);

    sd_ok = true;
    return 0;
}

/* Bounded equivalents of ROM 0x4bc0 and 0x4794.
 *
 * Both ROM routines can spin forever. These bounded equivalents return an error
 * instead, so disk_mount_all cannot hang without reporting a failure.
 */
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
    if (ROM_SD_RESP(&sdh, 0) & 0xfdffe008u)
        return 33;
    return 0;
}

static uint32_t SDICODE sd_fifo_word(uint32_t *out)
{
    uint32_t t = 0;
    /* Bit 2 set means the RX FIFO is empty. */
    while (R(SD_FIFOSTA) & (1u << 2))
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

/* Wait for the controller to finish the block it just handed us, and clear
 * the flag. The write path has always done this - ROM 0x488c spins on the same
 * bit and ROM 0x4892 writes 8 to clear it - and the read path never did.
 *
 * That asymmetry is what broke playback. One CMD17 per sector, with the next
 * command's STA reset, DCTRL and DATALEN written while the controller was
 * still closing out the previous block, desynchronised the data path after a
 * few sectors: "sd: read fifo fail lba=66243 n=61 rc=125" is sector four of a
 * 64-sector request timing out with the FIFO never going ready again. Small
 * reads - a directory entry, an ID3 tag - are one block and never hit it,
 * which is why the browser worked and only the audio buffer fill failed. */
/* Defined below with the write path, which has always used it; the read path
 * needs it too, for the recovery in sd_read_sectors. */
static uint32_t SDICODE sd_wait_ready(void);

static uint32_t SDICODE sd_wait_dataend(void)
{
    uint32_t t = 0;

    while (!(R(SD_STA) & STA_DATAEND))
        if (++t > SD_SPIN)
            return 0x79;
    R(SD_STA) = STA_DATAEND;
    return 0;
}

int SDICODE sd_read_sectors(IF_MD(int drive,)
                            sector_t start, int count, void *buf)
{
    uint8_t *p = buf;


    IF_MD((void)drive;)
    if (!sd_ok)
        return -1;

    while (count--) {
        uint32_t err;

        R(SD_STA) = STA_ALLFLAGS;
        /* The controller needs data mode enabled for every single-block read. */
        R(SD_DCTRL) |= 8u;
        sd_setup_data(512);

        ROM_SD_CMD(&sdh, 17, sd_addr(start));    /* READ_SINGLE_BLOCK */
        if ((err = sd_wait_data(17)) != 0) {
            SD_LOG_FAIL("read cmd", start, count + 1, (int)err);
            goto fail;
        }
        if ((err = sd_read_fifo512(p)) != 0) {
            SD_LOG_FAIL("read fifo", start, count + 1, (int)err);
            goto fail;
        }
        if ((err = sd_wait_dataend()) != 0) {
            SD_LOG_FAIL("read end", start, count + 1, (int)err);
            goto fail;
        }
        p += 512;
        start++;
        continue;

fail:
        /* Recover the CARD, not just the controller.
         *
         * Clearing the controller flags leaves a card that is still in a data
         * state exactly as stuck as it was, so the storage layer's retry
         * re-issues CMD17 into the same condition and fails identically. That
         * is visible in the log as the same LBA failing seven times with the
         * same code and nothing changing in between.
         *
         * So do what the write path has always done and the read path never
         * did: abort any transfer the card thinks is running, then wait for it
         * to report ready again. Both results are ignored deliberately - this
         * is a best-effort cleanup on a path that has already failed, and the
         * caller's retry is what decides the outcome. */
        R(SD_STA) = STA_ALLFLAGS;
        ROM_SD_CMD(&sdh, 12, 0);            /* STOP_TRANSMISSION */
        (void)sd_wait_data(12);
        R(SD_STA) = STA_ALLFLAGS;
        (void)sd_wait_ready();
        return (err == 0x7f) ? -2 : -3;
    }
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
 * words, waits for DATAEND, and clears the status afterwards.
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
        /* buf is not guaranteed to be aligned. */
        memcpy(&w, p, 4);
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

/* The write path is placed in SRAM because it spins on FIFO flags. */
int SDICODE sd_write_sectors(IF_MD(int drive,)
                             sector_t start, int count, const void *buf)
{
    const uint8_t *p = buf;

    IF_MD((void)drive;)
    if (!sd_ok)
        return -1;

    while (count--) {
        uint32_t err;

        /* The card may still be busy from the previous block. */
        if ((err = sd_wait_ready()) != 0)
            return -4;

        R(SD_STA) = STA_ALLFLAGS;
        R(SD_DCTRL) |= 8u;
        sd_setup_data(512);

        ROM_SD_CMD(&sdh, 24, sd_addr(start));    /* WRITE_BLOCK */
        if ((err = sd_wait_data(24)) != 0)
            return -2;
        if ((err = sd_write_fifo512(p)) != 0)
            return -3;
        p += 512;
        start++;
    }

    /* Do not return while the card is committing the last block. */
    if (sd_wait_ready() != 0)
        return -5;

    return 0;
}
