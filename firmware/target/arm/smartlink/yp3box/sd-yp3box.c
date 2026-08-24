/* SD driver for the SL6801.
 *
 * The controller at 0x40003000 is a **DesignWare Mobile Storage Host**
 * (`dw_mmc`) with a partly remapped register file. That identification is the
 * whole reason this file can now drive the card at full rate, and it came from
 * matching offsets and magic constants against the published IP rather than
 * from any new hardware cycle - the same move CLAUDE.md records for the MUSB
 * block at 0x40040000. What matches at the standard offset:
 *
 *   0x014 TMOUT    [31:8] data timeout, [7:0] response timeout
 *   0x018 CTYPE    bit 0 = card 0 is 4-bit; [31:16] would be 8-bit
 *   0x048 STATUS   bit 2 FIFO_EMPTY, bit 3 FIFO_FULL, [16:11] response index
 *   0x04c FIFOTH   [30:28] DMA msize, [27:16] RX watermark, [11:0] TX
 *   0x080 BMOD / 0x084 PLDMND / 0x088 DBADDR / 0x090 IDINTEN   internal DMA
 *   0x200 FIFO
 *
 * and what this part moved:
 *
 *   0x004 CMD      (std 0x2c) bit 31 start
 *   0x008 ARG      (std 0x28)
 *   0x00c BLKSIZ   (std 0x1c)
 *   0x010 BYTCNT   (std 0x20)
 *   0x034 RINTSTS  (std 0x44) write-1-to-clear
 *   0x038 RESP0..3 (std 0x30)
 *
 * The CMD word is the stock dw_mmc encoding, which is what ROM 0x40c4's
 * command table has been handing out all along:
 *
 *   [5:0] index  6 response_expect  7 response_long  8 check_response_crc
 *   9 data_expected  10 write  12 send_auto_stop  13 wait_prvdata_complete
 *   14 stop_abort_cmd  15 send_initialization  21 update_clock_only  31 start
 *
 * so CMD17 is 0x2351, CMD24 0x2758, and the vendor's own send wrapper at SRAM
 * 0x823092 emits CMD18 as 0x3352 and CMD25 as 0x3759 - the same words with bit
 * 12 set - whenever its handle asks for hardware auto-stop. That is where the
 * multi-block paths below come from.
 *
 * RINTSTS is the standard interrupt set too, which is why the masks this file
 * inherited from the ROM line up exactly: bit 2 CD, 3 DTO, 4 TXDR, 5 RXDR,
 * 6 RCRC, 8 RTO, 9 DRTO, 14 ACD, and 0xbfc2 is precisely "every error".
 *
 * Two things this identification corrected, both of which had cost cycles:
 *
 *   - 0x04c is FIFOTH, not a clock register. It had been called CLKCFG, and
 *     the note that "the card rate lives in the controller, not the module
 *     divider" followed from that name. It does not: the card clock IS the
 *     module clock on tree id 0x11, and the reason two earlier attempts at a
 *     higher divider produced an unreadable card was the memcpy-per-word FIFO
 *     drain overrunning, not the divider. Both are fixed here.
 *   - 0x018 bit 0 is CTYPE, the 4-bit bus enable. This file cleared it at
 *     init and never set it again, so every transfer this port has ever done
 *     ran one bit wide. The vendor sets it from HAL_SD_Init_new (SRAM
 *     0x828bec) after an ACMD6, and that is the 4x this driver was missing.
 *
 * Commands still go through the ROM's send/wait helpers wherever the ROM has
 * the right word for them: ROM 0x40c4 owns the index-to-command-word table and
 * ROM 0x419a..0x4400 own one wait per response type, and the masks genuinely
 * differ per type. The two auto-stop words the ROM table does not carry are
 * sent through ROM 0x40a2, which is the raw "write ARG, wait, write CMD" the
 * vendor's own wrapper calls.
 *
 * Polled, no interrupts. The controller has an internal DMA engine (BMOD at
 * 0x80, descriptor base at 0x88) and the vendor uses it; at 4-bit/24 MHz a
 * block arrives in 43 us and the drain below costs a few, so the CPU is not
 * the limit and the descriptor machinery is not worth its risk yet.
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
#define SD_CTYPE    0x018      /* bit 0: card 0 runs a 4-bit data bus */
#define SD_XFERCTL  0x02c      /* bit 3 data transfer, bit 14 multi-block */
#define SD_STA      0x034
#define SD_FIFOSTA  0x048
#define SD_FIFOTH   0x04c
#define SD_FIFO     0x200

#define CTYPE_4BIT      1u
#define XFER_DATA       (1u << 3)
#define XFER_MULTI      (1u << 14)
#define FIFOSTA_EMPTY   (1u << 2)
#define FIFOSTA_FULL    (1u << 3)

/* STATUS bits, from the ROM's error decoding (0x419a, 0x4bc0, 0x4794) */
#define STA_DONE      (1u << 2)    /* command/response complete   */
#define STA_DATAEND   (1u << 3)    /* data block finished (ROM 0x488c) */
#define STA_RXREADY   (1u << 5)    /* RX FIFO has a burst ready   */
#define STA_CRCFAIL   (1u << 6)
#define STA_TIMEOUT   (1u << 8)
#define STA_AUTOCMD   (1u << 14)   /* ACD: the auto-stop CMD12 finished */
#define STA_ERRMASK   0x300u       /* the pair the ROM treats as fatal */
#define STA_ALLERR    0xbfc2u      /* every error bit, no DONE    */
/* The vendor's clear-all is 0xbfc6, which predates this driver using the
 * hardware auto-stop: it leaves ACD latched. Clearing it too keeps a stale
 * bit from being read as the next transfer's auto-stop completing. */
#define STA_ALLFLAGS  (0xbfc6u | STA_AUTOCMD)

/* Clock/module ids: the vendor resets module 0x24 and drives clock 0x11. */
#define SD_MODULE   0x24u
#define SD_CLOCK    0x11u
/* The SOURCE for that clock. The vendor's own SD bring-up (FIRM 0xd67f14) sets
 * it every time, immediately before starting the clock:
 *
 *     d67f26  movs r1, #10          ; src 0x0a
 *     d67f28  movs r0, #17          ; clk 0x11
 *     d67f2a  bl   0x8050f2         ; clk_set_source
 *     d67f2e  movs r0, #17
 *     d67f30  bl   0x8051ec         ; clk_start
 *
 * We never did, which is why the card comes back after a boot but not after a
 * USB eject: at boot the source is whatever the ROM left while loading from
 * the card, and it is right. The ROM's own BOT target drives the card during
 * mass storage and does not put it back, so our re-init restored the divider
 * onto a source that is no longer feeding anything. A stopped source is a
 * stopped clock, the card never sees CMD0 or CMD55, and sd_power_on reports
 * "no card". CLAUDE.md has the rule this broke: a source number is itself a
 * clock id, so selecting one is a thing you must do, not a thing you inherit. */
/* FIFOTH for the transfer phase, as the ROM's own post-identification switch
 * writes it (ROM 0x46de) and as the vendor's handle carries it (SRAM 0x822570:
 * msize field 0x10000000, RX watermark 3, TX watermark 0xc). ROM_SD_CFG builds
 * the same three fields at init from its arg3/arg5/arg4, which is what leaves
 * 0x20070008 behind - msize 2, RX watermark 7, TX watermark 8.
 *
 * This is a FIFO threshold register, not a clock. It was called CLKCFG here,
 * and that name is why the divider was believed not to matter. */
#define SD_FIFOTH_XFER 0x1003000cu

#define SD_CLOCK_SRC 0x0au
#define SD_DIV_ID   64u
/* Identification-speed divider (vendor: 0x40).
 *
 * sd_set_clock runs first from sd_init, before sd_power_on, so identification
 * happens here. The SD spec caps identification at 400 kHz and allows 25 MHz
 * for transfer, and until the ladder below existed every data read on this
 * device - the FAT mount, every track, and every sector the ROM's BOT target
 * moves during USB mass storage - ran at the identification rate. That is the
 * suspect for "USB access is very slow", not the audio/USB module sharing.
 *
 * Measured, and the guess would have been wrong:
 *
 *     sd: clk=375000 Hz src0xa=24000000 Hz div=64
 *
 * so the source is 24 MHz and 64 lands exactly on 375 kHz - the identification
 * rate, inside the spec's 400 kHz cap. That also settles that selecting source
 * 0x0a is right: it is the source the vendor's divider was chosen against.
 *
 * Raising it is now the speed ladder at the end of sd_init, and the history
 * is worth keeping because two hardware cycles went into it.
 *
 * 12 MHz was tried first, unverified: the card went unreadable the instant the
 * clock changed and the device booted to what looked like a blank volume.
 * 6 MHz was tried next, with a check that raised the clock, read sector 0 and
 * fell back if the read returned an error. The card was still blank.
 *
 * So the check was worthless, and the reason is the important part: at the
 * wrong clock this controller does not report an error, it returns data. A
 * read of sector 0 "succeeded" and handed back bytes that were not sector 0.
 * Verifying the return code of a read proves nothing on hardware that fails
 * this way; only verifying the CONTENT does.
 *
 * Worse, the failure destroys the evidence. The black box is written to the
 * card, so a clock that makes the card unwritable produces no log of having
 * done so - the two boots that broke this way left nothing behind at all.
 * That is why the ladder ends at a rung equal to identification: falling all
 * the way back is a reachable outcome, not an exceptional one.
 *
 * What the ladder can NOT do is go above 24 MHz. Source 0x0a is 24 MHz and the
 * SD default-speed ceiling is 25 MHz, so the divider is already at the last
 * useful rung; 50 MHz needs CMD6 SWITCH_FUNC to put the card in high speed,
 * and neither command table in the vendor's firmware has a CMD6 word with the
 * data bit set (ROM 0x40c4 emits 0x2546, the vendor's wrapper 0x2446, both no
 * data phase). The vendor never runs this card in high speed, so there is no
 * vendor code to read for it - which by this port's rules puts it behind a
 * hardware cycle, not in this change. The width below is the 4x that is
 * actually documented in the vendor's own init. */

/* Bus width. The vendor decides this at SRAM 0x828bec and it is the single
 * biggest thing this driver was leaving on the table:
 *
 *     if (bus_widths_supports_4bit) {
 *         send_cmd(handle, 6, 2);        ; 0x823092 -> CMD word 0x2446
 *         wait_r1_idx(handle, 6);        ; 0x828796, ported as sd_wait_r1_idx
 *         base[0x18] |= 1;               ; CTYPE: card 0 is 4-bit
 *     } else {
 *         base[0x18] &= ~1;
 *     }
 *
 * Note the ORDER - the card is told first, the controller second - and note
 * that the vendor's CMD6 word is 0x2446 rather than the ROM table's 0x2546.
 * The difference is bit 8, check_response_crc. That is the same class of
 * choice as picking the right response wait: use the word the vendor uses for
 * that command, not the one that looks equivalent.
 *
 * The vendor reaches the 4-bit test by reading the SCR with ACMD51 and
 * checking SD_BUS_WIDTHS. This driver does not, for two reasons. Its ACMD51
 * path sets BYTCNT to 512 for an 8-byte register (SRAM 0x828cbc) and would
 * stall on any card that answered it, so it is not code to copy. And 4-bit is
 * mandatory for every SD memory card, so the interesting question is not
 * whether the card claims it but whether this board's traces carry it - which
 * only a content check can answer, and there is already one. */
#define SD_CMDW_ACMD6      0x2446u   /* SET_BUS_WIDTH, R1, no data phase   */
#define SD_CMDW_READ_MULTI 0x3352u   /* CMD18 + send_auto_stop (bit 12)    */
#define SD_CMDW_WRITE_MULTI 0x3759u  /* CMD25 + send_auto_stop             */

/* Sectors per multi-block command. Rockbox's buffering asks for runs far
 * larger than this; the cap keeps one command's data phase, and so one
 * software timeout, to something a stalled card cannot stretch into minutes. */
#define SD_MULTI_MAX 128

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
/* Raw send: takes the BASE, not the handle, and a {arg, cmdword} pair. It is
 * what the vendor's own send wrapper at SRAM 0x823092 tail-calls once it has
 * picked a command word, and it is the way in for the two auto-stop words the
 * ROM's table (0x40c4) does not carry. */
#define ROM_SD_SEND_RAW ((void (*)(volatile uint32_t *, const uint32_t *))0x40a3u)
#define ROM_UDELAY      ((void (*)(unsigned))0xb765u)
#define ROM_CLK_STOP    ((void (*)(unsigned))0x2a7du)
#define ROM_CLK_DIV     ((void (*)(unsigned, unsigned))0x2d59u)
#define ROM_CLK_APPLY   ((void (*)(unsigned))0x27a1u)
#define ROM_CLK_SRC     ((void (*)(unsigned, unsigned))0x3119u)
/* The vendor's source-select wrapper. sl6801-regs.h carries the canonical copy
 * and the reasoning; this file keeps its own ROM bindings rather than including
 * that header, so the wrapper is repeated here with it. Never call ROM_CLK_SRC
 * directly: a source number is itself a clock id, and 8 and 9 have to be
 * started before they can be selected. 0x0a is neither, but the guard stays so
 * that changing SD_CLOCK_SRC later cannot quietly reintroduce the trap. */
#define ROM_CLK_SET_SRC(id, src) do {                       \
        if ((src) == 8 || (src) == 9) ROM_CLK_APPLY(src);   \
        ROM_CLK_SRC((id), (src));                           \
    } while (0)
#define ROM_CLK_ENABLE  ((void (*)(unsigned))0x2565u)
#define ROM_CLK_DISABLE ((void (*)(unsigned))0x2571u)
#define ROM_GPIO_CFG1   ((void (*)(unsigned))0x7adu)
#define ROM_CLK_FREQ    ((uint32_t (*)(unsigned))0x3851u)  /* docs/ROM-API.md */

/* ROM entry points only dereference handle[0]. */
static struct { volatile uint32_t *base; } sdh;

/* CLK, CMD, DAT0-3 on port 3 pins 4..9, alternate function 11. */
static const unsigned sd_pins[] = {
    0x0003258bu, 0x00032d8bu, 0x0003358bu,
    0x00033d8bu, 0x0003458bu, 0x00034d8bu,
};

static uint32_t sd_fifoth_id;        /* FIFOTH as identification left it */
static bool     sd_ok;
static bool     sd_wide;             /* CTYPE bit 0: the bus is 4 data lines */
static bool     sd_multi;            /* CMD18/CMD25 with hardware auto-stop */
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
    /* Source before divider and apply, which is the vendor's order. Through
     * the wrapper, never ROM_CLK_SRC directly - see sl6801-regs.h. */
    ROM_CLK_SET_SRC(SD_CLOCK, SD_CLOCK_SRC);
    ROM_CLK_DIV(SD_CLOCK, div);
    sd_mdelay(100);
    ROM_CLK_APPLY(SD_CLOCK);

    /* get_clock_freq computes from the dividers and does not look at the gate,
     * so this reports a configured rate rather than a proven-running one -
     * which is exactly what is wanted here. */
    logf("sd: clk=%lu Hz src%#x=%lu Hz div=%u",
         (unsigned long)ROM_CLK_FREQ(SD_CLOCK), SD_CLOCK_SRC,
         (unsigned long)ROM_CLK_FREQ(SD_CLOCK_SRC), div);
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

int SDICODE sd_read_sectors(IF_MD(int drive,) sector_t start, int count,
                            void *buf);

/* Two sectors, for the post-identification checks. .bss, not const: it is a
 * DMA-free PIO read, but keeping card buffers out of flash is the standing
 * rule in this port. Two, because proving a multi-block transfer needs the
 * SECOND block to land in the right place, which one sector cannot show. */
static uint8_t sd_probe_buf[1024];

/* Does this bus width and rate actually work? Content, never a return code.
 *
 * Sector 0's boot signature is the one byte pair every partitioned card is
 * guaranteed to end its first sector with, so garbage is recognisable as
 * garbage. The seven sectors after it are read for errors only - one good
 * sector proves the rate can be reached, a short burst proves it can be
 * sustained, and sustained is what the audio buffer fill needs. */
static bool SDICODE sd_verify(void)
{
    int i;

    for (i = 0; i < 8; i++) {
        if (sd_read_sectors(IF_MD(0,) (sector_t)i, 1, sd_probe_buf) != 0)
            return false;
        if (i == 0 && (sd_probe_buf[510] != 0x55 || sd_probe_buf[511] != 0xaa))
            return false;
    }
    return true;
}

static uint32_t SDICODE sd_sum(const uint8_t *p)
{
    uint32_t i, sum = 0;

    for (i = 0; i < 512; i++)
        sum = (sum << 1) + (sum >> 31) + p[i];
    return sum;
}

/* Does the hardware auto-stop path work? Same rule: content.
 *
 * A multi-block read can be wrong in a way a single-block read cannot - the
 * command succeeds, the first block is perfect, and every block after it is
 * short, duplicated or absent because BYTCNT, the block boundary or the
 * auto-stop is wrong. So read sector 1 on its own, remember what it looked
 * like, then read sectors 0 and 1 as one transfer and require BOTH halves:
 * the boot signature at the end of the first block, and the second block
 * matching the sector that was already read by itself. */
static bool SDICODE sd_verify_multi(void)
{
    uint32_t sum1;

    if (sd_read_sectors(IF_MD(0,) (sector_t)1, 1, sd_probe_buf) != 0)
        return false;
    sum1 = sd_sum(sd_probe_buf);

    sd_multi = true;                     /* the path under test */
    if (sd_read_sectors(IF_MD(0,) (sector_t)0, 2, sd_probe_buf) != 0)
        return false;
    /* sd_read_sectors falls back to single blocks and clears sd_multi when a
     * multi-block transfer fails, and then RETURNS SUCCESS with correct data.
     * That is right for a running player and wrong for a test: the content
     * check below would pass on bytes the path under test never carried. So
     * ask whether the path is still enabled, not just whether the read
     * worked. */
    if (!sd_multi)
        return false;
    if (sd_probe_buf[510] != 0x55 || sd_probe_buf[511] != 0xaa)
        return false;
    return sd_sum(sd_probe_buf + 512) == sum1;
}

/* ACMD6, then CTYPE - the vendor's order at SRAM 0x828bec, and the command
 * word is the vendor's 0x2446 rather than the ROM table's 0x2546. Returns the
 * card's own refusal, so a card that will not widen falls back rather than
 * being driven four bits wide while it answers on one. */
static uint32_t SDICODE sd_set_bus_width(bool wide)
{
    uint32_t cmd[2], err;

    R(SD_STA) = STA_ALLFLAGS;
    ROM_SD_CMD(&sdh, 55, sd_rca << 16);          /* APP_CMD */
    if ((err = sd_wait_r1_idx(55)) != 0)
        return err;

    cmd[0] = wide ? 2u : 0u;                     /* 2 = 4 bit, 0 = 1 bit */
    cmd[1] = SD_CMDW_ACMD6;
    ROM_SD_SEND_RAW(sdh.base, cmd);
    if ((err = sd_wait_r1_idx(6)) != 0)
        return err;

    if (wide)
        R(SD_CTYPE) |= CTYPE_4BIT;
    else
        R(SD_CTYPE) &= ~CTYPE_4BIT;
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
    sd_fifoth_id = R(SD_FIFOTH);

    /* One data line until the card has been asked for four - the vendor's
     * HAL_SD_Init_new clears CTYPE here too, and identification is single
     * ended on DAT0 whatever the card ends up doing. */
    sd_wide  = false;
    sd_multi = false;
    R(SD_CTYPE) &= ~CTYPE_4BIT;
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

    /* Identification is over. Two things change now, and the bigger one is
     * not the clock:
     *
     *   - the bus goes from one data line to four (CTYPE, after an ACMD6),
     *     which the vendor does at SRAM 0x828bec and this driver never did;
     *   - the module clock leaves the 375 kHz identification divider.
     *
     * ROM 0x4690 is the vendor's own post-identification switch, and the line
     * in it that is not about the internal DMA is
     *
     *     0x46de:  str r2, [r3, #76]     ; FIFOTH = 0x1003000c
     *
     * against the 0x20070008 ROM_SD_CFG leaves at init. That register is the
     * FIFO threshold, not a clock - see the head of this file - so it is set
     * once here and the divider does the rate.
     *
     * Two earlier attempts raised the divider blind and shipped a card that
     * read as an empty volume, so this walks a ladder from fastest to slowest
     * and keeps the first rung that DEMONSTRABLY reads. Every rung is verified
     * by content, not by a return code - at the wrong rate this controller
     * returns data rather than an error, which is exactly how a silently
     * unreadable card got shipped twice.
     *
     * The width is the outer loop and the divider the inner one, so a board
     * whose DAT1-3 will not carry 24 MHz still gets 24 MHz on DAT0 rather
     * than falling back to 375 kHz for want of three traces.
     *
     * Every width change is issued at the identification divider, never at
     * the rate that just failed: ACMD6 sent into a clock this card cannot
     * follow fails for the wrong reason, and the ladder would then blame the
     * width. */
    {
        static const unsigned divs[] = { 1u, 2u, 8u };
        unsigned w;

        R(SD_FIFOTH) = SD_FIFOTH_XFER;

        for (w = 0; w < 2; w++) {
            bool wide = (w == 0);

            sd_set_clock(SD_DIV_ID);
            if ((err = sd_set_bus_width(wide)) != 0) {
                logf("sd: width %d refused (%lu)", wide ? 4 : 1,
                     (unsigned long)err);
                continue;
            }

            for (i = 0; i < sizeof(divs) / sizeof(divs[0]); i++) {
                sd_set_clock(divs[i]);
                if (!sd_verify()) {
                    logf("sd: rate bad width=%d div=%u", wide ? 4 : 1, divs[i]);
                    continue;
                }
                sd_wide  = wide;
                /* Only now, on a bus that is proven to read, is it worth
                 * asking whether the hardware auto-stop works. It sets
                 * sd_multi itself so the path can be exercised. */
                sd_multi = sd_verify_multi();
                if (!sd_multi)
                    logf("sd: multi-block bad, one CMD17 per sector");
                logf("sd: %d-bit div=%u multi=%d", wide ? 4 : 1, divs[i],
                     sd_multi);
                return 0;
            }

            /* Nothing at this width held up. Put the clock somewhere the card
             * answers before the next ACMD6 goes out. */
            sd_set_clock(SD_DIV_ID);
        }

        /* Neither width held up at any rate: back to the state that
         * identified the card, which is known to work and is merely slow. */
        sd_multi = false;
        sd_wide  = false;
        (void)sd_set_bus_width(false);
        R(SD_FIFOTH) = sd_fifoth_id;
        sd_set_clock(SD_DIV_ID);
        logf("sd: no rate held, 1-bit div=%u", SD_DIV_ID);
    }

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

/* Drain `len` bytes out of the RX FIFO. `len` is always a multiple of 32.
 *
 * Shaped like ROM 0x4794, which was written by the people who built the
 * controller: one error-and-watermark check per eight-word burst, and inside
 * the burst nothing but a poll on FIFO_EMPTY and a store. What it must NOT do
 * is call anything - `memcpy` for every four bytes was the previous version of
 * this loop, and memcpy lives in flash, so the drain was taking an XIP fetch
 * per word. An overrun FIFO drops words silently, so the symptom was garbage
 * data rather than an error, and it is what made a raised clock look like a
 * broken clock.
 *
 * ROM 0x4794 is no longer used, for two reasons: it drains exactly 512 bytes,
 * which a multi-block transfer is not, and its waits are unbounded, which is
 * the thing this driver spent a hardware cycle removing everywhere else.
 *
 * The unaligned case stores bytes rather than reaching for memcpy. Rockbox
 * hands this word-aligned buffers in practice; the path exists so that a
 * caller that does not cannot corrupt anything. */
static uint32_t SDICODE sd_read_fifo(void *buf, uint32_t len)
{
    uint8_t *p = buf;
    uint32_t left = len;
    bool aligned = (((uintptr_t)buf & 3u) == 0);

    while (left) {
        uint32_t sta, t = 0, n;

        for (;;) {
            sta = R(SD_STA);
            if (sta & STA_ERRMASK) { R(SD_STA) = STA_ERRMASK; return 32; }
            if (sta & STA_RXREADY) break;
            if (++t > SD_SPIN) return 0x7d;
        }

        for (n = 0; n < 8 && left; n++, left -= 4, p += 4) {
            uint32_t w;

            t = 0;
            /* FIFOSTA bit 2 set means the RX FIFO is empty. */
            while (R(SD_FIFOSTA) & FIFOSTA_EMPTY)
                if (++t > SD_SPIN)
                    return 0x7e;
            w = R(SD_FIFO);

            if (aligned) {
                *(uint32_t *)p = w;
            } else {
                p[0] = (uint8_t)w;
                p[1] = (uint8_t)(w >> 8);
                p[2] = (uint8_t)(w >> 16);
                p[3] = (uint8_t)(w >> 24);
            }
        }
        R(SD_STA) = STA_RXREADY;             /* ROM clears it after the burst */
    }
    return 0;
}

/* Byte address for standard-capacity cards, block address for SDHC/SDXC -
 * the same test the ROM makes at 0x474e. */
static uint32_t sd_addr(sector_t sector)
{
    return (sd_cardtype == 2) ? (uint32_t)sector : (uint32_t)(sector << 9);
}

/* BLKSIZ stays 512 and BYTCNT carries the whole transfer - which is exactly
 * what the vendor's data setup at SRAM 0x823106 does, and why its multi-block
 * caller passes `nblocks << 9`. */
static void SDICODE sd_setup_data(uint32_t len)
{
    R(SD_DTIMER)  = 0xffffff40u;
    R(SD_BLKSIZE) = 512;
    R(SD_DATALEN) = len;
}

/* The transfer-control register the vendor touches before every data command:
 * bit 3 for a data transfer at SRAM 0x8227fa, bit 14 as well for a multi-block
 * one at 0x82283a. The vendor only ever ORs, because its own operating-mode
 * switch (ROM 0x4690) rewrites the register wholesale first; here bit 14 is
 * set and cleared explicitly, so a single-block read that follows a multi
 * cannot inherit it. */
static void SDICODE sd_xfer_ctl(bool multi)
{
    uint32_t v = R(SD_XFERCTL) | XFER_DATA;

    if (multi)
        v |= XFER_MULTI;
    else
        v &= ~XFER_MULTI;
    R(SD_XFERCTL) = v;
}

/* The auto-stop CMD12 the controller issues for us finishes after the data
 * does, and latches ACD. Leaving it latched is harmless today but would make
 * the next transfer's ACD unreadable. */
static uint32_t SDICODE sd_wait_autostop(void)
{
    uint32_t t = 0;

    while (!(R(SD_STA) & STA_AUTOCMD))
        if (++t > SD_SPIN)
            return 0x78;
    R(SD_STA) = STA_AUTOCMD;
    return 0;
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

/* Recover the CARD, not just the controller.
 *
 * Clearing the controller flags leaves a card that is still in a data state
 * exactly as stuck as it was, so the storage layer's retry re-issues the same
 * command into the same condition and fails identically. That is visible in
 * the log as the same LBA failing seven times with the same code and nothing
 * changing in between.
 *
 * So abort any transfer the card thinks is running, then wait for it to report
 * ready again. Both results are ignored deliberately - this is a best-effort
 * cleanup on a path that has already failed, and the caller's retry is what
 * decides the outcome. */
static void SDICODE sd_recover(void)
{
    R(SD_STA) = STA_ALLFLAGS;
    ROM_SD_CMD(&sdh, 12, 0);                 /* STOP_TRANSMISSION */
    (void)sd_wait_data(12);
    R(SD_STA) = STA_ALLFLAGS;
    (void)sd_wait_ready();
}

static uint32_t SDICODE sd_read_one(sector_t start, void *buf)
{
    uint32_t err;

    R(SD_STA) = STA_ALLFLAGS;
    sd_xfer_ctl(false);
    sd_setup_data(512);

    ROM_SD_CMD(&sdh, 17, sd_addr(start));    /* READ_SINGLE_BLOCK */
    if ((err = sd_wait_data(17)) != 0)
        return err;
    if ((err = sd_read_fifo(buf, 512)) != 0)
        return err;
    return sd_wait_dataend();
}

/* CMD18 with the controller's own auto-stop.
 *
 * One command for the whole run instead of one per sector. The command word
 * is 0x3352 - the ROM table's CMD18 with bit 12, send_auto_stop - which is
 * exactly what the vendor's send wrapper at SRAM 0x823092 emits when its
 * handle asks for hardware stop, and the vendor correspondingly skips its own
 * CMD12 in that mode (SRAM 0x82289a bics #2). The ROM's table has no entry
 * with bit 12 set, so this goes out through the raw send the vendor's wrapper
 * itself calls. */
static uint32_t SDICODE sd_read_multi(sector_t start, int count, void *buf)
{
    uint32_t cmd[2], err;

    R(SD_STA) = STA_ALLFLAGS;
    sd_xfer_ctl(true);
    sd_setup_data((uint32_t)count * 512u);

    cmd[0] = sd_addr(start);
    cmd[1] = SD_CMDW_READ_MULTI;
    ROM_SD_SEND_RAW(sdh.base, cmd);

    if ((err = sd_wait_data(18)) != 0)
        return err;
    if ((err = sd_read_fifo(buf, (uint32_t)count * 512u)) != 0)
        return err;
    if ((err = sd_wait_dataend()) != 0)
        return err;
    return sd_wait_autostop();
}

int SDICODE sd_read_sectors(IF_MD(int drive,)
                            sector_t start, int count, void *buf)
{
    uint8_t *p = buf;

    IF_MD((void)drive;)
    if (!sd_ok)
        return -1;

    while (count > 0) {
        uint32_t err;
        int n = 1;

        if (sd_multi && count > 1) {
            n = (count > SD_MULTI_MAX) ? SD_MULTI_MAX : count;
            err = sd_read_multi(start, n, p);
            if (err) {
                /* Do not fail the request: the single-block path is still
                 * there and is known to work, since the ladder proved it
                 * before it ever tried this one. Turn multi-block off for the
                 * rest of this boot rather than paying the failure on every
                 * subsequent run, and say so once - a stream of these would
                 * mean the auto-stop path is wrong, which is a different bug
                 * from a card going away. */
                SD_LOG_FAIL("read multi", start, n, (int)err);
                logf("sd: multi-block off, one CMD17 per sector from here");
                sd_multi = false;
                sd_recover();
                continue;
            }
        } else {
            err = sd_read_one(start, p);
            if (err) {
                SD_LOG_FAIL("read", start, count, (int)err);
                sd_recover();
                return (err == 0x7f) ? -2 : -3;
            }
        }

        p     += (uint32_t)n * 512u;
        start += n;
        count -= n;
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

/* The data half of a write, modelled on ROM 0x483a. Two things this does that
 * simply pushing words does not: it checks the data error bits between bursts,
 * waits for DATAEND, and clears the status afterwards. The command wait is NOT
 * a substitute - it reports on the command, not on the data.
 *
 * Same shape as the read drain and for the same reason: neither the per-word
 * memcpy nor the ROM's per-word push (0x4610, an unbounded spin behind a call)
 * belongs inside a FIFO loop on an XIP build. */
static uint32_t SDICODE sd_write_fifo(const void *buf, uint32_t len)
{
    const uint8_t *p = buf;
    uint32_t left = len, t;
    bool aligned = (((uintptr_t)buf & 3u) == 0);

    while (left) {
        uint32_t sta, n;

        t = 0;
        for (;;) {
            sta = R(SD_STA);
            if (sta & STA_ERRMASK) { R(SD_STA) = STA_ERRMASK; return 32; }
            if (sta & (1u << 4)) break;        /* TXDR, ROM 0x4850 */
            if (++t > SD_SPIN) return 0x7b;
        }

        for (n = 0; n < 8 && left; n++, left -= 4, p += 4) {
            uint32_t w;

            if (aligned)
                w = *(const uint32_t *)p;
            else
                w = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);

            t = 0;
            /* FIFOSTA bit 3 set means the TX FIFO is full. */
            while (R(SD_FIFOSTA) & FIFOSTA_FULL)
                if (++t > SD_SPIN)
                    return 0x7b;
            R(SD_FIFO) = w;
        }
    }

    t = 0;
    while (!(R(SD_STA) & STA_DATAEND))
        if (++t > SD_SPIN)
            return 0x7a;
    R(SD_STA) = STA_DATAEND;                   /* ROM 0x4892 writes 8 */

    if (R(SD_STA) & STA_ERRMASK) { R(SD_STA) = STA_ERRMASK; return 32; }
    return 0;
}

static uint32_t SDICODE sd_write_one(sector_t start, const void *buf)
{
    uint32_t err;

    R(SD_STA) = STA_ALLFLAGS;
    sd_xfer_ctl(false);
    sd_setup_data(512);

    ROM_SD_CMD(&sdh, 24, sd_addr(start));    /* WRITE_BLOCK */
    if ((err = sd_wait_data(24)) != 0)
        return err;
    return sd_write_fifo(buf, 512);
}

/* CMD25 with the controller's own auto-stop - 0x3759, the ROM table's CMD25
 * with bit 12, and the word the vendor's wrapper emits at SRAM 0x8230f4. */
static uint32_t SDICODE sd_write_multi(sector_t start, int count,
                                       const void *buf)
{
    uint32_t cmd[2], err;

    R(SD_STA) = STA_ALLFLAGS;
    sd_xfer_ctl(true);
    sd_setup_data((uint32_t)count * 512u);

    cmd[0] = sd_addr(start);
    cmd[1] = SD_CMDW_WRITE_MULTI;
    ROM_SD_SEND_RAW(sdh.base, cmd);

    if ((err = sd_wait_data(25)) != 0)
        return err;
    if ((err = sd_write_fifo(buf, (uint32_t)count * 512u)) != 0)
        return err;
    return sd_wait_autostop();
}

/* The write path is placed in SRAM because it spins on FIFO flags.
 *
 * Multi-block writes ride on the same sd_multi the read ladder proved. That is
 * an inference rather than its own check - the command word, the transfer
 * register and the auto-stop are the same three mechanisms, only the direction
 * bit differs - and it is deliberately not verified by writing to the card,
 * because a verification write has to put bytes somewhere and there is nowhere
 * on a user's card that is safe to spend. A card that cannot do it falls back
 * on the first failure, exactly as the read path does. */
int SDICODE sd_write_sectors(IF_MD(int drive,)
                             sector_t start, int count, const void *buf)
{
    const uint8_t *p = buf;

    IF_MD((void)drive;)
    if (!sd_ok)
        return -1;

    while (count > 0) {
        uint32_t err;
        int n = 1;

        /* The card may still be committing the previous block. */
        if (sd_wait_ready() != 0)
            return -4;

        if (sd_multi && count > 1) {
            n = (count > SD_MULTI_MAX) ? SD_MULTI_MAX : count;
            err = sd_write_multi(start, n, p);
            if (err) {
                SD_LOG_FAIL("write multi", start, n, (int)err);
                logf("sd: multi-block off, one CMD24 per sector from here");
                sd_multi = false;
                sd_recover();
                continue;
            }
        } else {
            err = sd_write_one(start, p);
            if (err) {
                SD_LOG_FAIL("write", start, count, (int)err);
                sd_recover();
                return (err == 0x7f) ? -2 : -3;
            }
        }

        p     += (uint32_t)n * 512u;
        start += n;
        count -= n;
    }

    /* Do not return while the card is committing the last block. */
    if (sd_wait_ready() != 0)
        return -5;

    return 0;
}
