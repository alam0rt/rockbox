/*
 * PCM playback for the Smartlink SL6801.
 *
 * Three things have to be brought up, and they are independent:
 *
 *   the audio block at 0x40009000   module 0x25, clock 0x13, fed by a
 *                                   dedicated PLL (24.576 or 22.579 MHz)
 *                                   committed on clock id 0
 *   the general DMA engine          0x40001000, module 0x21, peripheral
 *                                   request line 1, writing into the TX FIFO
 *                                   at 0x4000920c
 *   IRQ 60 + channel                the DMA channel's own completion interrupt
 *
 * Everything here is transcribed from the vendor's audio HAL; docs/AUDIO.md
 * and docs/DMA.md carry the derivation and the evidence tags.
 *
 * WHAT THIS FILE USED TO BE. Every revision before this one drove 0x40040300,
 * 0x40040200 and IRQ 43 behind module 0x23. All four belong to the USB
 * mass-storage controller. The register map came from FIRM 0xd66fe4, which
 * writes a control word, sets two enable bits and hooks an interrupt - it
 * reads exactly like audio_start, and it is inside driver_usbd_msc_param_init
 * (string FIRM 0x00ce3e0f, loaded two instructions into the enclosing
 * function). The nine-module probe that "found the audio gate" found the USB
 * gate and was reading USB registers back the whole time. Nothing in the
 * vendor's audio path touches 0x40040xxx at all.
 *
 * The Rockbox side is the software-volume sink path, which is what makes this
 * driver simple: with PCM_SINK_SWVOL the core hands us chunks out of its own
 * double buffer (pcm_sw_volume.c, PCM_PLAY_DBL_BUF_SAMPLES, max 4 KB), each at
 * a stable SRAM address. So "program a transfer, get an interrupt, ask for the
 * next" is the whole driver - which is also exactly what the vendor does
 * (SRAM 0x816d14 re-arms from its completion callback), so the two models
 * agree and no circular transfer is needed.
 */
#include "config.h"
#include "system.h"
#include "audio.h"
#include "pcm.h"
#include "pcm-internal.h"
#include "pcm_sink.h"
#include "sl6801-regs.h"
#include "pcm_mixer.h"
#include "kernel.h"
#include "blackbox.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

/* --- audio master clock ---------------------------------------------------
 *
 * The block runs off a dedicated PLL and the vendor picks between exactly two
 * settings by testing the requested rate against 8000. FIRM 0xd7d6a4, and again
 * at 0xd7d4d4:
 *
 *     r2 = 8000
 *     r3 = rate % r2
 *     pll = (r3 == 0) ? 24576000 : 22579000
 *     set_pll(pll)
 *     clk_apply(0)
 *
 * 24576000 is 512 * 48000 and 22579000 is the achievable approximation of
 * 512 * 44100, so this is the usual "48k family or 44.1k family" split.
 *
 * set_pll itself is FIRM's HAL at 0x80da34. It is NOT callable from here - it
 * lives in the loaded SRAM HAL at 0x800000-0x830000, which our own .data/.bss
 * overwrite - so it is reproduced. Both constant pairs are read straight out of
 * its literal pool (0x80da88..0x80da9c):
 *
 *     want 24576000:  mult = 0x3126 (12582), pre = 3
 *     otherwise:      mult = 0x186c2 (100034), pre = 2, achieves 22579000
 *
 * Runs from .icode: it is a clock change, and this port has paid for that rule
 * four times. */
#define ACLK(o) (*(volatile uint32_t *)(CLKCTRL_BASE + (o)))

unsigned long yp3_audio_mclk;   /* what the PLL actually achieved, in Hz */

static void __attribute__((section(".icode"), noinline))
yp3_audio_pll(unsigned long rate)
{
    int f48 = (rate % 8000u) == 0;
    unsigned mult = f48 ? 12582u : 100034u;
    unsigned pre  = f48 ? 3u : 2u;

    /* Bounded. The vendor spins here unbounded (SRAM 0x80da4e); on a device
     * with no console an unbounded wait is indistinguishable from a dead
     * board, and this function is reached from the block's own init. */
    {
        unsigned long spins = 0;
        while ((ACLK(0x14) & (1u << 11)) && ++spins < 1000000) ;
        if (spins >= 1000000)
            logf("pll: BUSY never cleared, 14=%08lx", (unsigned long)ACLK(0x14));
    }
    ACLK(0x10) = (ACLK(0x10) & 0xfc1f0000u) | 0x3000u | (pre << 5);
    ACLK(0x14) = (ACLK(0x14) & 0x3800u) | 0x80000000u | 0x400u | (mult << 14);
    yp3_audio_mclk = f48 ? 24576000ul : 22579000ul;
}

/* --- sample-rate code -----------------------------------------------------
 *
 * The PLL family is only half of it. The rate itself goes into TX_CTRL[11:8]
 * as a code, from the switch at FIRM 0xd7b224 (and the identical capture-side
 * one at 0xd7b4ba). Anything not in the table encodes as 0.
 *
 * The port declares one rate today; this is written out in full because the
 * table is the whole answer, not a guess, and adding a rate is a one-line
 * change to yp3_sampr[] plus the target's HW_SAMPR_CAPS once 44.1 kHz has
 * actually made a sound. */
static unsigned yp3_rate_code(unsigned long rate)
{
    switch (rate) {
    case 12000:  return 1;
    case 11025:  return 2;
    case 16000:  return 3;
    case 24000:  return 4;
    case 22050:  return 5;
    case 32000:  return 6;
    case 48000:  return 7;
    case 44100:  return 8;
    case 96000:  return 9;
    case 88200:  return 10;
    case 192000: return 11;
    case 176400: return 12;
    default:     return 0;
    }
}

/* --- bitfield write -------------------------------------------------------
 *
 * The vendor's own helper, FIRM 0xd7a3d8. It appears in this driver wherever
 * the vendor used it, so the transcription stays readable against the
 * disassembly rather than being expanded by hand into shifts. */
static inline void field_set(volatile uint32_t *reg, uint32_t mask, uint32_t v)
{
    unsigned shift = 0;
    while (!((mask >> shift) & 1u))
        shift++;
    *reg = (*reg & ~mask) | ((v << shift) & mask);
}

/* --- the audio block ------------------------------------------------------
 *
 * Cold init, from audio_hw_init at FIRM 0xd7d598. Runs once. The ASRC window
 * clear is included because the vendor does it before anything else touches
 * the block; playback never reads that RAM, but leaving a block half-set up
 * is how this port has lost cycles before.
 *
 * .icode: two module enables and two clock commits, and ROM clk_enable spins
 * on an acknowledge bit. */
static bool audio_block_up;

static void __attribute__((section(".icode"), noinline))
yp3_audio_block_init(void)
{
    volatile uint32_t d;
    uint32_t a;

    if (!ROM_CLK_IS_ON(AUDIO_MODULE))
        ROM_CLK_ENABLE(AUDIO_MODULE);
    ROM_CLK_APPLY(AUDIO_CLK);

    AUD_RX_CTRL |= 1u;                          /* FIRM 0xd7abf8 */
    for (d = 0; d < 200000u; d++) ;             /* the vendor's 20 ms */

    /* Capture volumes to maximum, then the ASRC coefficient RAM. FIRM 0xd7ac0e
     * loads 0x1ff into r2 and copies it into r1, so 0x1ff is BOTH the mask and
     * the value - this is a write of the field maximum, not a mute. Playback
     * does not use the capture side, but leaving a block half-initialised is
     * how this port has lost cycles before.
     *
     * The ASRC RAM is only addressable with module 0x20 and clock 0x2d up and
     * bit 2 of 0x40000020 set; the vendor drops all three again eight
     * instructions later (FIRM 0xd7ac08..0xd7ac6a). We drop the clock but
     * leave module 0x20 enabled: this port has no established ROM entry for
     * clk_disable, and an unnecessary module left on costs power, while a
     * wrong one turned off costs a boot. */
    field_set(&AUD_RX_VOL(0), AUD_VOL_MAX, AUD_VOL_MAX);
    field_set(&AUD_RX_VOL(1), AUD_VOL_MAX, AUD_VOL_MAX);
    field_set(&AUD_RX_VOL(2), AUD_VOL_MAX, AUD_VOL_MAX);

    if (!ROM_CLK_IS_ON(ASRC_MODULE))
        ROM_CLK_ENABLE(ASRC_MODULE);
    ROM_CLK_APPLY(ASRC_CLK);
    REG32(0x40000020) |= 4u;
    for (a = AUDIO_BASE + 0x400; a < AUDIO_BASE + 0x600; a += 4)
        REG32(a) = 0;
    REG32(0x40000020) &= ~4u;
    ROM_CLK_STOP(ASRC_CLK);

    field_set(&AUD_RX_CTRL, 0x000f0000u, 5);
    field_set(&AUD_RX_CTRL, 0x0000f000u, 5);
    field_set(&AUD_RX_MUTE(0), 0x10000u, 0);
    field_set(&AUD_RX_MUTE(1), 0x10000u, 0);
    field_set(&AUD_RX_MUTE(2), 0x10000u, 0);

    audio_block_up = true;
    logf("aud: init 00=%08lx play(0x200)=%08lx cap(0x100)=%08lx",
         (unsigned long)AUD(0x00), (unsigned long)AUD_TX_CTRL,
         (unsigned long)AUD_RX_CTRL);
}

/* --- the analogue output stage --------------------------------------------
 *
 * audio_analog_enable(mask) at FIRM 0xd7ac8c, with mask == 3: the stereo pair,
 * which is what /dev/audio0 selects (it issues ioctl 90 with 3 at FIRM
 * 0xd67488). The channel-2 branches and the gain ramp are the mask & 4 path
 * and are not reached with mask 3, so they are not reproduced.
 *
 * TWO DEPARTURES FROM THE VENDOR, both deliberate and both marked:
 *
 * 1. AUD(0x64) and AUD(0x68) are per-chip trim values the vendor fetches from
 *    an OTP/efuse accessor (FIRM 0xcf88ea -> 0xcf88cc -> 0xc481b0, over an
 *    object at SRAM 0x0081b090 that our .bss overwrites). We cannot read it,
 *    and writing a wrong trim is worse than leaving whatever the boot ROM put
 *    there - so these are logged and not written.
 *
 * 2. The analogue gain fields in AUD(0x30) come from the vendor's volume
 *    manager and are zero in a freshly calloc'd context. Whether 0 is minimum
 *    or unity is not established, and the field is only 5 bits wide, so it is
 *    a named constant here rather than a silent 0: if the first hardware run
 *    is silent with the digital volume at maximum, this is the one line to
 *    change, and the log line below reports what it actually holds. */
#define YP3_ANALOG_GAIN  0u             /* AUD(0x30)[20:16] and [28:24], [U] */

/* Which analogue channels to bring up. 3 is the stereo pair, which is what
 * /dev/audio0 selects; the self-test at the foot of this file varies it,
 * because there is a real question about whether this product's jack and
 * amplifier hang off channel 2 rather than 0 and 1 - analogue channel 0's
 * output pads are P1.10/P1.11, and P1.10 is this panel's reset line
 * (lcd-yp3box.c LCD_RESET_PIN, the same 0x15000 descriptor). A product cannot
 * route audio through its own LCD reset, so channel 0 is suspect `[I]`. */
static unsigned yp3_analog_mask = 3u;

/* The three channels' registers, indexed by channel:
 *   present bit  ch0/ch1 in AUD(0x30) bits 0 and 4, ch2 in AUD(0x2c) bit 0
 *   power reg    AUD(0x50), AUD(0x54), AUD(0x58)
 *   0x4c bit     0x20000, 0x10000, 0x400000
 *   settle bit   0x01, 0x02, 0x100
 *   bias field   0x0d, 0xd00, 0xd0000 in AUD(0x20)   (FIRM 0xd7ab90)
 *
 * EVIDENCE NOTE. The channel 0 and 1 sequence below is the one this driver has
 * always had, transcribed instruction by instruction from 0xd7ac8c. The
 * channel 2 arms are from the same function's mask & 4 branches as condensed
 * in docs/AUDIO.md, which is a weaker reading - they have never been written
 * out against the disassembly line by line. They are here to be TESTED, not
 * because they are known good. */
static uint32_t aud_pwr(unsigned ch)
{
    return AUDIO_BASE + (ch == 2 ? 0x58u : 0x50u + ch * 4u);
}

static void yp3_analog_enable(unsigned mask)
{
    unsigned ch;

    /* Already up? Bit 29 of each channel's power register reads back as the
     * state flag - the vendor's own gain ioctls test exactly this. Only the
     * channels being asked for count, or a mask change would be ignored. */
    {
        bool all_up = true;
        for (ch = 0; ch < 3; ch++)
            if ((mask & (1u << ch)) && !(REG32(aud_pwr(ch)) & (1u << 29)))
                all_up = false;
        if (all_up)
            return;
    }

    if (mask & 1u) AUD(0x30) |= 0x01u;  /* channel 0 present */
    if (mask & 2u) AUD(0x30) |= 0x10u;  /* channel 1 present */
    if (mask & 4u) AUD(0x2c) |= 0x01u;  /* channel 2 present */
    if (mask & 3u) AUD(0x40) |= 0x200u;
    if (mask & 1u) AUD(0x4c) |= 0x20000u;
    if (mask & 2u) AUD(0x4c) |= 0x10000u;
    if (mask & 4u) AUD(0x4c) |= 0x400000u;
    AUD(0x40)  = (AUD(0x40) & ~0x30u) | 0x20u;
    AUD(0x4c) |= 0x100000u;
    if (mask & 3u) AUD(0x40) |= 0x800u;
    for (ch = 0; ch < 3; ch++)          /* pre-charge */
        if (mask & (1u << ch))
            REG32(aud_pwr(ch)) |= 0x8000u;
    udelay(10000);
    for (ch = 0; ch < 3; ch++)
        if (mask & (1u << ch)) {
            REG32(aud_pwr(ch)) |= 0x10000000u;      /* enable */
            REG32(aud_pwr(ch)) |= 0x20000000u;      /* up     */
        }
    AUD(0x4c) |= 0x30u;
    AUD(0x4c) |= (mask & 3u) | ((mask & 4u) ? 0x100u : 0u);
    udelay(10000);
    for (ch = 0; ch < 3; ch++)          /* the vendor's mvn/lsr idiom */
        if (mask & (1u << ch))
            REG32(aud_pwr(ch)) |= 0xfffu;
    udelay(10000);
    AUD(0x4c) &= ~((mask & 3u) | ((mask & 4u) ? 0x100u : 0u));
    if (mask & 1u) field_set(&AUD(0x30), 0x001f0000u, YP3_ANALOG_GAIN);
    if (mask & 2u) field_set(&AUD(0x30), 0x1f000000u, YP3_ANALOG_GAIN);
    if (mask & 4u) field_set(&AUD(0x2c), 0x001f0000u, YP3_ANALOG_GAIN);

    /* audio_pa_bias, FIRM 0xd7ab90: 0x0d per analogue channel. */
    AUD(0x20) |= (mask & 1u ? 0x0du : 0u)
               | (mask & 2u ? 0xd00u : 0u)
               | (mask & 4u ? 0xd0000u : 0u);

    logf("aud: analog mask=%u 2c=%08lx 30=%08lx 4c=%08lx", mask,
         (unsigned long)AUD(0x2c), (unsigned long)AUD(0x30),
         (unsigned long)AUD(0x4c));
    logf("aud: analog 50=%08lx 54=%08lx 58=%08lx (bit29 = channel up)",
         (unsigned long)AUD(0x50), (unsigned long)AUD(0x54),
         (unsigned long)AUD(0x58));
    logf("aud: trim 64=%08lx 68=%08lx (not written, see comment)",
         (unsigned long)AUD(0x64), (unsigned long)AUD(0x68));
}

/* --- the playback half ----------------------------------------------------
 *
 * audio_tx_enable at FIRM 0xd7b0e0, taking the force == 0 path - the one
 * audio_open uses. Passing 1, which audio_route_apply does, forces the rate
 * code to 7 (48 kHz) and skips the channel-count write entirely, so the two
 * paths are not interchangeable.
 *
 * The four per-channel control words are the outcome of the vendor's loop at
 * 0xd7b360 for an enabled channel ((x & ~8) | 1) and a disabled one
 * (x & ~0xb). Its input is a bitmap assembled from six context bytes that the
 * vendor's route ioctls maintain; we want the stereo pair on and the other two
 * off, so the outcome is written directly rather than reproducing the state
 * machine. */
/* Start the PLAYBACK stream, transcribed from the vendor's own playback arm.
 *
 * The vendor function is FIRM 0xd7b0e0. It has exactly three call sites and
 * they do NOT agree on the force flag: audio_open passes 0 (0xd7d760),
 * audio_route_apply (0xd7c49c) and audio_rx_enable (0xd7b48a) both pass 1,
 * which forces the rate code to 7 and skips the 0x208 write. What follows is
 * the force == 0 path - audio_open's, the one a stream actually opens with -
 * in its order, through the base pointer at 0xd7b3d4 (= 0x40009000):
 *
 *   0xd7b0e4  base[0x00] |= 0x80000022
 *   0xd7b0f4  base[0x00] |= 0x40000000        a SECOND write, not one OR
 *   0xd7b0fe  base[0xfc] &= ~0x1f
 *   0xd7b194  base[0x200] &= ~2
 *   0xd7b1a0  base[0x200] |= 1
 *   0xd7b1a8  the four channel volumes, field 0x1ff
 *             (literals 0xd7b3dc..0xd7b3e8 = 0x40009228/258/288/2b8)
 *   0xd7b24c  base[0x200] = (old & ~0xf00) | (ratecode << 8)
 *   0xd7b25c  base[0x208] = stereo ? (old | 4) : (old & ~4), then | 0x80
 *   0xd7b376  per enabled channel: |= 2 (or |= 6), then |= 1
 *   0xd7ba00  base[0x410] |= 0x40000000       (from 0xd7b99c, the stream start)
 *
 * TWO THINGS THAT ARE NOT WHAT THE CAPTURE FILE DOES, and both were wrong here
 * because the two register files are identical in shape and the capture one
 * (0x100, driven by FIRM 0xd7b46c) was transcribed by mistake:
 *
 *   - playback never sets bit 1 of base[0x200]. The capture path builds
 *     `old | 2 | ratecode<<8` at 0xd7b528; the playback path builds
 *     `old | ratecode<<8` and relies on bit 0, set separately at 0xd7b1a0.
 *   - a playback channel word is 3, not 5. 0xd7b390..0xd7b398 ORs 2 (or 6)
 *     and then 1; the 5 came from the capture loop at 0xd7b56e.
 */
/* Readback tracing. The first hardware run of this path logged
 * `tx ctrl=00300800` at the END of this function - bits 0 and 1 clear, when
 * 0xd7b1a0 had written bit 0 - and there was no way to tell WHEN it was lost.
 * A write that never latches and a bit that clears itself two writes later are
 * different faults with the same footprint at the end. Only the self-test
 * turns this on; normal playback is not a place to log six lines. */
static bool yp3_tx_trace;
#define TXTRACE(tag) do { if (yp3_tx_trace) \
        logf("tx: %s 200=%08lx", tag, (unsigned long)AUD_TX_CTRL); } while (0)

static void yp3_tx_enable(unsigned long rate, bool stereo)
{
    unsigned n;
    uint32_t v;

    TXTRACE("entry");

    /* 0xd7b0e4 .. 0xd7b0fe - global, and ahead of everything else. */
    AUD(0x00) |= 0x80000022u;
    AUD(0x00) |= 0x40000000u;
    AUD(0xfc) &= ~0x1fu;

    /* FIRM 0xd7a3f0 and 0xd7a478, reached from 0xd7b0e0's head gated on
     * ctx[0xe6] bits 0 and 1 - one output channel each, and never implemented
     * here. Both halves of AUD(0x08) go in for stereo:
     *
     *     0xd7a404:  AUD(0x08) |= 0x2040 | 3            (bits 0,1,6,13)
     *     0xd7a48c:  AUD(0x08) |= 0x20400000 | 0x30000  (the same, << 16)
     *
     * 0xd7aa78 is the matching disable and clears 0x3fc0 and 3, which is what
     * identifies the two halves as one field per channel. */
    AUD(0x08) |= 0x2040u | 3u;
    AUD(0x08) |= 0x20400000u | 0x30000u;

    /* 0xd7b194 / 0xd7b1a0 - stop, then enable. Bit 0 is the enable for this
     * file; bit 1 stays clear. */
    AUD_TX_CTRL &= ~2u;
    TXTRACE("clr1");
    AUD_TX_CTRL |= 1u;
    TXTRACE("set0");

    /* 0xd7b1a8 - all four channel volumes, not three. */
    for (n = 0; n < 4; n++)
        field_set(&AUD_TX_VOL(n), AUD_VOL_MAX, AUD_VOL_MAX);
    TXTRACE("vols");

    /* 0xd7b24c - rate code into [11:8], everything else preserved. */
    AUD_TX_CTRL = (AUD_TX_CTRL & ~0xf00u)
                | ((uint32_t)yp3_rate_code(rate) << 8);
    TXTRACE("rate");

    /* 0xd7b25c - bit 2 is stereo, bit 7 is unconditional. */
    AUD_TX_FMT = (AUD_TX_FMT & ~4u) | (stereo ? 4u : 0u) | 0x80u;

    /* FIRM 0xd7ab90, called from 0xd7b356 immediately before the channel loop
     * and never implemented here at all. It is per-channel routing in
     * AUD(0x20), one nibble-ish field per channel:
     *
     *     channel 0 enabled -> |= 0x0d     else &= ~0x0d
     *     channel 1 enabled -> |= 0xd00    else &= ~0xd00
     *     channel 2 enabled -> |= 0xd0000  else &= ~0xd0000
     *
     * through the same 0x40009000 base (literal 0xd7abf4). Stereo is channels
     * 0 and 1, so the first two fields go in and the third comes out. Without
     * this the channel words below are set on channels that are not routed
     * anywhere, which is consistent with a transmitter that accepts every
     * register write and still never asks the DMA for a sample. */
    /* NOTE: yp3_analog_enable already ORs 0x0d0d into this register, so this is
     * not new - it is here because the vendor sets it at 0xd7ab90, called from
     * 0xd7b356 BEFORE the channel loop, and the analogue path runs after. */
    AUD(0x20) = (AUD(0x20) & ~0xd0d0du) | 0x0du | 0xd00u;

    /* 0xd7b390 - the stereo pair on, the other two off. The `| 6` variant is
     * selected by a context byte (ctx[0xe5]) this driver has no equivalent of;
     * the `& ~4 | 2` arm is the one taken when that byte is clear, which is
     * the freshly-initialised state. */
    AUD(AUD_TX_CH(0)) = (AUD(AUD_TX_CH(0)) & ~4u) | 3u;
    AUD(AUD_TX_CH(1)) = (AUD(AUD_TX_CH(1)) & ~4u) | 3u;
    AUD(AUD_TX_CH(2)) &= ~0xbu;
    AUD(AUD_TX_CH(3)) &= ~0xbu;

    /* 0xd7b41c: base[0x200] |= 2, at the exit of the four-iteration channel
     * loop (0xd7b410 `cmp r2, #4` / 0xd7b414 `bne 0xd7b362`). THIS IS THE RUN
     * BIT and it is the vendor's, not a guess.
     *
     * It was in this driver, was removed on the claim that "there is no such
     * write on the playback path", and the removal was wrong - the search that
     * produced that claim followed the channel loop's body and never followed
     * it to its exit. Hardware then settled it: with bit 1 clear the DMA
     * channel armed, GO stuck, and not one transfer ever completed; poking
     * bit 1 alone started it and 132 transfers completed at exactly the 44.1
     * kHz rate.
     *
     * Bit 0, which the vendor writes at 0xd7b1a4, does NOT read back on this
     * part - `tx: set0 200=00300000` immediately after the write. It is not
     * the enable it was documented as; bit 1 is the whole story. */
    AUD_TX_CTRL |= 2u;
    TXTRACE("run");

    /* The vendor's order: the analogue stage comes up AFTER this file is
     * configured and BEFORE the stream is started - 0xd7c4a2 runs 0xd7b46c,
     * then 0xd7c4a8 runs 0xd7ac8c, then 0xd7c4b6 runs 0xd7b99c. */
    yp3_analog_enable(yp3_analog_mask);

    /* 0xd7ba00, in the stream-start step. ONLY bit 30.
     *
     * An earlier version also wrote 0x44 into this register, taken from
     * 0xd7b55a - which is inside the CAPTURE enable's channel loop, not this
     * path. 0x410 is zeroed by yp3_audio_block_init along with the rest of the
     * 0x400..0x600 window, and the vendor's playback arm sets bit 30 and
     * nothing else, so that is what happens here. */
    v = AUD(0x410);
    AUD(0x410) = v | 0x40000000u;
    TXTRACE("r410");

    logf("aud: tx ctrl=%08lx fmt=%08lx code=%u vol=%08lx",
         (unsigned long)AUD_TX_CTRL, (unsigned long)AUD_TX_FMT,
         yp3_rate_code(rate), (unsigned long)AUD_TX_VOL(0));
    /* Read back what actually stuck: this file and the capture file are
     * identical in shape, so the only proof of which one is live is what the
     * registers hold after the writes. */
    logf("aud: ch0=%08lx ch1=%08lx ch2=%08lx r410=%08lx",
         (unsigned long)AUD(AUD_TX_CH(0)), (unsigned long)AUD(AUD_TX_CH(1)),
         (unsigned long)AUD(AUD_TX_CH(2)), (unsigned long)AUD(0x410));
    logf("aud: 00=%08lx r08=%08lx r20=%08lx fc=%08lx",
         (unsigned long)AUD(0x00), (unsigned long)AUD(0x08),
         (unsigned long)AUD(0x20), (unsigned long)AUD(0xfc));
}

static void yp3_tx_disable(void)
{
    AUD_TX_CTRL &= ~2u;
    AUD(AUD_TX_CH(0)) &= ~0xbu;
    AUD(AUD_TX_CH(1)) &= ~0xbu;
}

/* --- the DMA channel ------------------------------------------------------
 *
 * docs/DMA.md has the descriptor decode. Feeding audio_open's descriptor
 * (32-bit both sides, burst 4, burst length 16, destination request 1) through
 * dma_open produces the control word below; the same derivation reproduces
 * both LCD channels bit-for-bit against probe/dma_work.bin, which is the
 * strongest check available without hardware.
 *
 * Channel 2 rather than 0. The vendor's allocator hands audio whichever
 * channel is free and, with the LCD holding 0 and 1, that is 2 in the capture.
 * Rockbox does not drive an LCD DMA channel today, so 0 would work - but if it
 * ever does it will take 0 and 1 for the same reasons the vendor did, and a
 * silent collision on a shared engine is not a debugging session anyone wants.
 */
#define PCM_DMA_CH      2

/* bits 21:20 = 2 (32-bit source)   bit 19 source burst
 * bit 17 source is memory          bits 9:8 = 2 (32-bit destination)
 * bit 7 destination burst          bits 4:0 = 1 (audio request line) */
#define PCM_DMA_CTRL    0x002a0281u
/* bits 31:28 source burst length - 1, bits 27:24 destination burst length - 1 */
#define PCM_DMA_LENCFG  0xff000000u
/* bits 31:14 from descriptor +0x08, which is 32 in every vendor caller;
 * bit 1 enables the transfer-complete interrupt, bit 0 the other source. */
#define PCM_DMA_CFG     0x00080002u

static void __attribute__((section(".icode"), noinline))
yp3_dma_module_enable(void)
{
    /* FIRM 0xd7d834, the head of dma_open. A gated module accepts and reads
     * back destination, source, length and config perfectly while every
     * transfer times out - the second row of the table in CLAUDE.md, and the
     * reason the LCD lost two rounds. From SRAM: clk_enable spins. */
    if (!ROM_CLK_IS_ON(DMA_MODULE))
        ROM_CLK_ENABLE(DMA_MODULE);
}

static void yp3_dma_open(void)
{
    yp3_dma_module_enable();

    DMA_CTRL(PCM_DMA_CH) &= ~DMA_CTRL_GO;
    DMA_CTRL(PCM_DMA_CH)  = PCM_DMA_CTRL;
    DMA_LEN(PCM_DMA_CH)   = PCM_DMA_LENCFG;
    DMA_IRQST(PCM_DMA_CH) = DMA_IRQST(PCM_DMA_CH);      /* W1C anything stale */
    DMA_CFG(PCM_DMA_CH)   = (DMA_CFG(PCM_DMA_CH) & 0x3fffu) | PCM_DMA_CFG;

    logf("dma: ch%d ctrl=%08lx cfg=%08lx len=%08lx", PCM_DMA_CH,
         (unsigned long)DMA_CTRL(PCM_DMA_CH),
         (unsigned long)DMA_CFG(PCM_DMA_CH),
         (unsigned long)DMA_LEN(PCM_DMA_CH));
}

/* dma_start, SRAM 0x814c94. Destination, source, the 18-bit count, then bit 30
 * with bit 29 cleared. The audio stream wrapper the vendor calls this through
 * (SRAM 0x814c08) refuses a length that is not a multiple of 4; with 32-bit
 * widths on both sides that is the floor to keep. */
/* The audio peripheral request line is 1, and it is not a guess: the vendor's
 * audio dma_open at FIRM 0xd7d6f0 builds its descriptor with the source and
 * destination request ids set to 1 and 0 (or 0 and 1) depending on direction -
 * 0x10 and 0x14 in the descriptor, at 0xd7d6fe and 0xd7d74a. Playback is
 * memory (id 0) to audio (id 1), which is precisely what PCM_DMA_CTRL's
 * bits 4:0 and bit 17 already encode.
 *
 * An eight-way sweep of that field used to run here on the first buffer. It
 * reported "NO request line 0-7 moved a byte", and that was true but
 * meaningless: the transmitter had never been started, so no line could have
 * asserted. It is gone - a speculative probe in the start path is exactly the
 * kind of thing that produces a surprise three flashes later. */
static void yp3_dma_play(const void *addr, size_t size)
{
    DMA_DST(PCM_DMA_CH) = AUD_TX_FIFO;
    DMA_SRC(PCM_DMA_CH) = (uint32_t)addr;
    DMA_LEN(PCM_DMA_CH) = (DMA_LEN(PCM_DMA_CH) & ~DMA_LEN_MASK)
                        | (size & DMA_LEN_MASK);
    DMA_CTRL(PCM_DMA_CH) = (DMA_CTRL(PCM_DMA_CH) & ~DMA_CTRL_CONT)
                         | DMA_CTRL_GO;
}

/* --- self-test state ------------------------------------------------------
 *
 * Declared here because the DMA interrupt below has to know about it. The
 * test itself is at the foot of this file. */
/* OFF. It answered what it was built for - the digital path is complete, the
 * transmitter runs, the DMA moves samples at the sample rate - and the thing
 * it could not answer, why none of that reaches a pin, is recorded in
 * docs/AUDIO.md with nine hypotheses excluded.
 *
 * It is off now because it had started to COST evidence rather than produce
 * it. yp3_dma2_irq logs only while pcm_irqs <= 8, and a self-test run spends
 * all eight of those on itself - 1215 transfers before the UI is even up - so
 * real playback, the path that now matters, logged no interrupts at all. It
 * also spends 25 seconds of every boot and hands normal playback an audio
 * block it has already configured and torn down.
 *
 * A probe that outlives its question starts eating the instrumentation of
 * whatever comes next. */
#ifndef YP3_AUDIO_SELFTEST
#define YP3_AUDIO_SELFTEST 0
#endif
#if YP3_AUDIO_SELFTEST
/* 20 ms of 44.1 kHz stereo. NOT const, and never const: .rodata is SPI NOR on
 * this XIP build, and a DMA reading flash while the CPU fetches instructions
 * from it wedges the bus with no fault and no timeout. This lands in .bss,
 * which is SRAM, and is filled at run time. */
#define ST_FRAMES 880
static int16_t st_tone[ST_FRAMES * 2] __attribute__((aligned(4)));
static volatile bool st_active;
static volatile uint32_t st_irqs;
void yp3_audio_selftest(void);       /* defined at the foot of this file */
#else
/* The interrupt's log guard reads this whether or not the test is built. */
#define st_active false
#endif

/* --- interrupt ------------------------------------------------------------
 *
 * The DMA channel's own, IRQ 60 + channel. The audio block raises nothing the
 * driver services: no NVIC line is enabled for it anywhere in the vendor's
 * audio HAL.
 *
 * The shared vendor body is SRAM 0x814c34 - read the status, write it straight
 * back (write-1-to-clear), then dispatch bit 0 and bit 1 to two callbacks.
 * Audio leaves callback A null and takes completion on bit 1. */
#define NVIC_ISER   ((volatile uint32_t *)0xE000E100)
#define NVIC_ICER   ((volatile uint32_t *)0xE000E180)
#define NVIC_IPR    ((volatile uint8_t  *)0xE000E400)

static void yp3_irq_enable(unsigned irq, unsigned prio)
{
    NVIC_IPR[irq] = prio << 4;          /* 4 priority bits, high nibble */
    NVIC_ISER[irq >> 5] = 1u << (irq & 31);
}

static void yp3_irq_disable(unsigned irq)
{
    NVIC_ICER[irq >> 5] = 1u << (irq & 31);
}

static uint32_t pcm_irqs;        /* transfer-complete callbacks taken */
static uint32_t pcm_plays;       /* sink_play calls */

void yp3_dma2_irq(void)
{
    uint32_t status = DMA_IRQST(PCM_DMA_CH);

    DMA_IRQST(PCM_DMA_CH) = status;     /* W1C first, as the vendor does */
    pcm_irqs++;

    /* The first few only. No line at all means the channel never ran, which
     * separates a dead engine from one that runs and never signals - the two
     * produce identical register readbacks and this port has confused them
     * before. logf from interrupt context is not something to leave in the
     * tree; this is bounded and comes out once the path works. */
    /* Not while the self-test is talking. logf from interrupt context
     * interleaved with the foreground's own logf and corrupted a line in the
     * v3 run - `st: gpio1 dma irq #2 st=00000003` - and the line it ate was a
     * pre-walk GPIO dump, which was one of the two things that boot existed to
     * capture. The ring has no locking; the fix is not to write from both
     * sides at once. */
    if (pcm_irqs <= 8 && !st_active)
        logf("dma irq #%lu st=%08lx", (unsigned long)pcm_irqs,
             (unsigned long)status);

#if YP3_AUDIO_SELFTEST
    /* The self-test drives the hardware directly and must not reach into the
     * core's PCM state machine - nothing above pcm_sink has been started at
     * the point it runs. It re-arms its own buffer and returns. */
    if (st_active) {
        st_irqs++;
        if (status & DMA_IRQST_DONE)
            yp3_dma_play(st_tone, sizeof st_tone);
        return;
    }
#endif

    if (status & DMA_IRQST_DONE) {
        const void *addr;
        size_t size;

        if (pcm_play_dma_complete_callback(PCM_DMAST_OK, &addr, &size))
            yp3_dma_play(addr, size);
    }
}

/* --- the speaker power amplifier -------------------------------------------
 *
 * FIRM 0xd42ce0, the branch that prints "PA open in media_manager OK.":
 *
 *     d42ce2  r0 = [0xd42d0c]        = 0x0001f00b
 *     d42ce4  bl 0x8051f4            SRAM thunk -> b.w 0x7ac
 *     d42ce8  r0 = "PA open in media_manager OK."
 *     d42cf2  r0 = 0x0001f000        the close path, same helper
 *
 * 0x8051f4 sits in the same SRAM thunk table as the clock helpers this port
 * already uses (0x8051ec -> 0x27a0, 0x8051f0 -> 0x2a7c), and it branches to
 * ROM 0x7ac - the GPIO helper, ROM_GPIO_CFG1 here.
 *
 * Decoding the descriptor with the ROM's own unpacker at 0x77a, which takes
 * the port from bits 16..19 and the pin from bits 11..15:
 *
 *     0x0001f00b -> port 1 pin 30, low nibble 0xb    PA on
 *     0x0001f000 -> port 1 pin 30, low nibble 0x0    PA off
 *
 * Checked against the two descriptors this port already had names for:
 * 0x0001bf90 and 0x0001b790 decode to port 1 pins 23 and 22, which is exactly
 * what usb-yp3box.c documents them as. So the speaker amplifier is port 1
 * pin 30, and these are the vendor's own two words for it - not a guess at a
 * pin, and not a guess at what to write to it.
 *
 * This is a board amplifier behind the SoC's own output stage and is unrelated
 * to the P1.10..P1.15 pads the vendor's analogue enable muxes; those conflict
 * with the LCD assignments and are left alone until docs/BOARD-PINS.md's
 * unresolved conflict is settled. */
#define SPK_PA_ON       0x0001f00bu     /* FIRM 0xd42d0c */
#define SPK_PA_OFF      0x0001f000u     /* FIRM 0xd42cf2 */

static bool spk_pa_on;

static void yp3_speaker_pa(bool on)
{
    if (on == spk_pa_on)
        return;
    spk_pa_on = on;
    ROM_GPIO_CFG1(on ? SPK_PA_ON : SPK_PA_OFF);
    logf("spk: PA %s", on ? "open" : "close");
}

/* --- the sink ------------------------------------------------------------- */

static bool tx_running;

static void sink_init(void)
{
    /* The block is left off until there is something to play: it reprograms a
     * PLL, and a clock change during LCD or SD bring-up is one more variable
     * in the wrong place. sink_set_freq brings it up on demand. */
#if YP3_AUDIO_SELFTEST
    yp3_audio_selftest();
#endif
}

static void sink_postinit(void)
{
    /* Empty, and two traps worth recording for whoever fills it.
     *
     * Do NOT start playback here. pcm_postinit() sets pcm_is_ready[] only
     * after this returns, so a play request made from this hook is silently
     * dropped and the audio thread stalls before its queue loop ever runs -
     * that hung the device on the loading screen.
     *
     * Do NOT hook probes to sink_play() either: whether it is reached at all
     * is a race, because pcmbuf_play_start() starts the mixer only once a full
     * 8 KiB chunk is committed, and on some boots the codec has produced less
     * than that when it fires. Reading registers here is fine; blocking is
     * not. */
}

static void sink_set_freq(uint16_t freq)
{
    /* freq is a Rockbox sample-rate INDEX, not a rate, and the port declares
     * exactly one. Both halves of the rate now have a home: the PLL family
     * here, and the rate code in yp3_tx_enable. */
    unsigned long rate = SAMPR_44;
    (void)freq;

    if (!audio_block_up)
        yp3_audio_block_init();

    yp3_audio_pll(rate);
    ROM_CLK_APPLY(AUDIO_MCLK);

    yp3_tx_enable(rate, true);
    yp3_dma_open();
    tx_running = true;
}

static void sink_lock(void)
{
    yp3_irq_disable(DMA_IRQ(PCM_DMA_CH));
}

static void sink_unlock(void)
{
    yp3_irq_enable(DMA_IRQ(PCM_DMA_CH), 2);
}

static void sink_play(const void *addr, size_t size)
{
    if (!tx_running)
        sink_set_freq(0);

    yp3_dma_play(addr, size);
    yp3_irq_enable(DMA_IRQ(PCM_DMA_CH), 2);     /* vendor priority is 2 */

    /* Amplifier last, once samples are already flowing, so it is not powered
     * up into a silent line. */
    yp3_speaker_pa(true);

    /* The first two only: after that this is a per-buffer hot path.
     *
     * Readback discipline, from the table in CLAUDE.md: zeroes here mean a
     * module is off; correct values with no completion mean a clock was never
     * committed - or that the engine was told to do something that can never
     * finish. All three have happened on this path. */
    if (pcm_plays < 2) {
        logf("pcm play #%lu addr=%08lx size=%lu mclk=%lu",
             (unsigned long)pcm_plays, (unsigned long)addr,
             (unsigned long)size, yp3_audio_mclk);
        logf("pcm mod: audio25=%d dma21=%d",
             ROM_CLK_IS_ON(AUDIO_MODULE), ROM_CLK_IS_ON(DMA_MODULE));
        logf("pcm dma: ctrl=%08lx src=%08lx dst=%08lx len=%08lx st=%08lx",
             (unsigned long)DMA_CTRL(PCM_DMA_CH),
             (unsigned long)DMA_SRC(PCM_DMA_CH),
             (unsigned long)DMA_DST(PCM_DMA_CH),
             (unsigned long)DMA_LEN(PCM_DMA_CH),
             (unsigned long)DMA_IRQST(PCM_DMA_CH));
        logf("pcm aud: 00=%08lx tx=%08lx fmt=%08lx vol=%08lx",
             (unsigned long)AUD(0x00), (unsigned long)AUD_TX_CTRL,
             (unsigned long)AUD_TX_FMT, (unsigned long)AUD_TX_VOL(0));
        logf("pcm ch: ch0=%08lx ch1=%08lx iser1=%08lx",
             (unsigned long)AUD(AUD_TX_CH(0)),
             (unsigned long)AUD(AUD_TX_CH(1)),
             (unsigned long)REG32(0xE000E104));
    }
    pcm_plays++;
}

static void sink_stop(void)
{
    /* Amplifier first on the way down. The vendor's teardown order is
     * explicitly unresolved in docs/AUDIO-BOARD.md ("Ordering of mute versus
     * PA GPIO and all delays remain [U]"), so this is our choice, not a
     * transcription: muting the amplifier before the samples stop is the
     * usual way to keep the stop out of the speaker. */
    yp3_speaker_pa(false);

    /* The number that settles it: plays without completions means the channel
     * was armed and never ran. Residue is DMA_LEN & 0x3ffff (SRAM 0x814d1c) -
     * a full count left means it never started, a partial one means it
     * started and stalled, and those are different faults. */
    logf("pcm stop: plays=%lu irqs=%lu residue=%lu st=%08lx",
         (unsigned long)pcm_plays, (unsigned long)pcm_irqs,
         (unsigned long)(DMA_LEN(PCM_DMA_CH) & DMA_LEN_MASK),
         (unsigned long)DMA_IRQST(PCM_DMA_CH));

    /* Interrupt off before the channel, so a completion cannot request another
     * transfer on the way down. dma_abort is a bit-29 clear (SRAM 0x814ce8);
     * bit 30 is the start. */
    yp3_irq_disable(DMA_IRQ(PCM_DMA_CH));
    DMA_CTRL(PCM_DMA_CH) &= ~(DMA_CTRL_GO | DMA_CTRL_CONT);
    yp3_tx_disable();
    tx_running = false;
}

/* ==========================================================================
 * The audio self-test
 * ==========================================================================
 *
 * WHY THIS EXISTS. Nothing on this path has ever produced a sample. Reaching
 * it through Rockbox means going via pcmbuf, buffering and a codec, and
 * ROADMAP A1 already documents a livelock in that stack - so a silent boot
 * cannot be attributed to the hardware. This runs from sink_init(), which
 * pcm_init() calls after lcd_init() and storage_init() and before
 * audio_init(): the LCD is up, the card is mounted so the black box can flush
 * the log, and no codec, no PCM ring and no UI are running.
 *
 * IT IS STAGED, AND THAT IS THE POINT. CLAUDE.md's rule is never to land four
 * changes in one flash, because when it fails every one of them failed
 * together. That rule is about CONFIGURATION changes whose failures alias.
 * This is the opposite construction: every stage logs its verdict BEFORE the
 * next one runs, so the last line in the ring names the stage that stopped.
 * One flash, many hypotheses, no aliasing.
 *
 * WHAT EACH STAGE SETTLES:
 *
 *   S0  port-1 GPIO window, dumped raw     which pads the LCD already claimed,
 *                                          from the running system, no meter
 *   S1  block up, registers read back      zeroes => module 0x25 is wrong,
 *                                          and it costs nothing else
 *   S2  PLL committed                      the master clock
 *   S3  TX file configured, read back      the digital transmitter took it
 *   S4  bit 29 of 0x50/0x54/0x58           a HARDWARE acknowledgement that the
 *                                          analogue channel powered up - the
 *                                          vendor's own gain ioctls test it
 *   S5  one DMA transfer, bounded wait     "dma irq" present or absent, plus
 *                                          residue: full count => never
 *                                          started, partial => started and
 *                                          stalled. Different faults, and they
 *                                          read identically in the registers.
 *   S6  a tone you can hear                three passes over the analogue
 *                                          channel mask, four gain steps each
 *
 * S6 IS READ BY EAR, AND IT IS BUILT TO BE. Each pass uses a different tone
 * frequency, so which analogue channel produced sound is reported by PITCH and
 * needs no counting: 1 kHz = channels 0+1, 500 Hz = channel 2 alone, 2 kHz =
 * all three. Within a pass the gain steps 0, 8, 16, 31 as four one-second
 * bursts with a gap between them, so "it came in on the third burst" measures
 * YP3_ANALOG_GAIN, which has been [U] for the life of this port.
 *
 * WHAT IT DELIBERATELY DOES NOT DO: it never runs the analogue output pad mux.
 * Analogue channel 0's pads are P1.10/P1.11 and lcd-yp3box.c drives the very
 * same descriptor (0x15000) as the panel reset, so that walk would reset the
 * display mid-test and turn a clean result into a confusing one. The pads are
 * downstream of getting a sample out at all.
 *
 * Delete this whole block in the commit that acts on its result. */
#if YP3_AUDIO_SELFTEST

static void st_wait_ms(unsigned ms)
{
    while (ms--)
        udelay(1000);
}

/* A square wave, because it is the loudest thing per unit amplitude and the
 * easiest to be sure you heard. Amplitude is deliberately a quarter of full
 * scale: the analogue gain is unknown and this may be going into a 3 W
 * amplifier a few centimetres from someone's ear. */
static void st_fill(unsigned long rate, unsigned freq)
{
    unsigned half = (unsigned)(rate / (freq * 2u));
    unsigned i;

    if (half == 0)
        half = 1;
    for (i = 0; i < ST_FRAMES; i++) {
        int16_t v = ((i / half) & 1u) ? 8000 : -8000;
        st_tone[i * 2]     = v;
        st_tone[i * 2 + 1] = v;
    }
}

static void st_set_gain(unsigned mask, unsigned g)
{
    if (mask & 1u) field_set(&AUD(0x30), 0x001f0000u, g);
    if (mask & 2u) field_set(&AUD(0x30), 0x1f000000u, g);
    if (mask & 4u) field_set(&AUD(0x2c), 0x001f0000u, g);
}

/* The analogue output pads, transcribed from the vendor's own walk.
 *
 * Each analogue channel owns TWO pads, which is what a differential output
 * looks like - and the XS4150's inputs are a differential pair on pins 3/4
 * `[D]`, so that fits:
 *
 *     channel 0   0x15000 / 0x15800   P1.10 / P1.11
 *     channel 1   0x16000 / 0x16800   P1.12 / P1.13
 *     channel 2   0x17000 / 0x17800   P1.14 / P1.15
 *
 * The walk is a four-stage state machine, state in ctx[0xf0], 50 ms apart.
 * Channel 0's stages are FIRM 0xd7a668, 0xd7a6c4, 0xd7a704, 0xd7a74c and each
 * one is guarded on the previous state:
 *
 *     1  gpio_set_mode(pad, 0x780)   both pads     then gpio_set_alt(pad, 4)
 *     2  gpio_set_mode(pad, 0x780)   both pads     (mode only, again)
 *     3  gpio_set_alt(pad, 12)       both pads
 *     4  gpio_set_alt(pad, 0)        both pads     <- the resting state
 *
 * 0x780 is the mode field pre-shifted: [10:7] = 15. So the end state is
 * mode 15, alt 0, and stages 1..3 are a sequence to get there without a pop.
 *
 * ONE DEPARTURE. The vendor reaches mode and alt through two helpers at FIRM
 * 0xc48244 and 0xc48268, and FIRM is what an XIP build overwrites - those
 * addresses are Rockbox now. The boot ROM's configurator takes mode and alt in
 * one packed descriptor, which is what this driver already uses for the
 * amplifier, so each stage rewrites both fields with mode held at 15. The
 * sequence of alt values is unchanged and that is the part that matters.
 *
 * P1.10 IS ALSO lcd-yp3box.c's LCD_RESET_PIN, the identical 0x15000. Channel 0
 * is walked LAST here for that reason: if the panel dies, everything else has
 * already been tested and logged. */
#define PAD_MODE15  0x780u

static void st_pad_walk(unsigned ch)
{
    unsigned a = 0x15000u + ch * 0x1000u;       /* 0x15000/0x16000/0x17000 */
    unsigned b = a + 0x800u;

    ROM_GPIO_CFG1(a | PAD_MODE15 | 4u);
    ROM_GPIO_CFG1(b | PAD_MODE15 | 4u);
    st_wait_ms(50);
    ROM_GPIO_CFG1(a | PAD_MODE15 | 4u);
    ROM_GPIO_CFG1(b | PAD_MODE15 | 4u);
    st_wait_ms(50);
    ROM_GPIO_CFG1(a | PAD_MODE15 | 12u);
    ROM_GPIO_CFG1(b | PAD_MODE15 | 12u);
    st_wait_ms(50);
    ROM_GPIO_CFG1(a | PAD_MODE15 | 0u);
    ROM_GPIO_CFG1(b | PAD_MODE15 | 0u);
}

/* The port-1 register window.
 *
 * The v3 run decoded three of these against pins whose function is already
 * known, and all three agree `[L]`:
 *
 *   +0x00..+0x0c   FOUR BITS OF MODE PER PIN, eight pins per word. P1.10's
 *                  nibble in +0x04 read 1 before its walk and 15 after, and
 *                  mode 1 is exactly what BOARD-PINS decodes lcd-yp3box.c's
 *                  panel-reset descriptor 0x000150cb as.
 *   +0x10          pin data, one bit per pin. It read 0x40020400 with the
 *                  amplifier open, the backlight on and panel reset high -
 *                  bits 30, 17 and 10, which is P1.30, P1.17 and P1.10.
 *   +0x14, +0x18   two bits of PULL per pin, pins 0-15 and 16-31. Each walk
 *                  cleared 01 to 00 on exactly the two pads it touched, and
 *                  that is a DEPARTURE - the vendor's set_mode does not touch
 *                  pull, and this driver's packed descriptor carries [5:4] = 0.
 *
 * The alternate-function value is NOT in this window, which is why the v3
 * dumps showed the alt walk doing nothing visible. ROM 0x7ac maps alt through
 * a table and hands it to ROM 0x5ae, and the sibling helper at ROM 0x82c
 * writes its argument to *port_base + 0x218* - so the alt registers sit a long
 * way above the 0x40 window. Both halves are dumped now. */

/* THE ALTERNATE-FUNCTION REGISTERS, decoded from ROM 0x5ae `[C]` and confirmed
 * against this board `[L]`. They were in the dump all along.
 *
 * ROM 0x5ae(base, pin, value) does:
 *
 *     w     = base + (pin >> 4) * 4
 *     shift = (pin & 15) * 2
 *     w[0x24] = (w[0x24] & ~(3 << shift)) | ((value & 3) << shift)
 *     w[0x2c] = (w[0x2c] & ~(3 << shift)) | (((value >> 16) & 3) << shift)
 *
 * so the alt is TWO two-bit fields per pin: +0x24/+0x2c for pins 0-15 and
 * +0x28/+0x30 for pins 16-31. `value` is not the alt index - ROM 0x7ac maps
 * the index through a twelve-entry table at ROM 0x9c4 covering alts 4..15,
 * with anything else becoming 0x80000000. Port bases are the table at ROM
 * 0x9b4: 0x40081000 + 0x40 * port.
 *
 * The v4 run proves the decode `[L]`. P1.10's field held 2 in BOTH +0x24 and
 * +0x2c before its walk, which is table entry 0x80020002 - index 11 - and
 * BOARD-PINS decodes lcd-yp3box.c's panel-reset descriptor 0x000150cb as alt
 * 11. Exactly the same number, from two unrelated directions.
 *
 * AND THE SAME DUMP SHOWS P1.11..P1.15 ALREADY AT ALT 0. Their fields read
 * zero before any walk, so walking channels 1 and 2 changed no alt at all -
 * the only thing those walks altered was the PULL field. Whatever made v3
 * audible, it was not an alt change.
 *
 * The +0x200..+0x21c window this driver went looking in is not involved:
 * +0x218 held 0x800 before and after, and everything else there read zero. */

/* Set exactly one combination of the four DIGITAL transmit channels.
 *
 * The vendor's loop at 0xd7b360 writes these four words from a bitmap it
 * assembles out of six context bytes, and docs/AUDIO.md has said since it was
 * written that WHICH PHYSICAL OUTPUT EACH OF THE FOUR TX CHANNELS DRIVES IS
 * [U]. This driver picked 0 and 1 on the first day and every build since has
 * kept them, because "the stereo pair" sounded like an answer. It is a guess,
 * it is the only link in the chain that has never been varied, and by now it
 * is the last cheap one left.
 *
 * An enabled word is (x & ~4) | 3 and a disabled one is x & ~0xb, both from
 * 0xd7b390 and 0xd7b404. */
static void st_tx_channels(unsigned mask)
{
    unsigned n;

    for (n = 0; n < 4; n++) {
        if (mask & (1u << n))
            AUD(AUD_TX_CH(n)) = (AUD(AUD_TX_CH(n)) & ~4u) | 3u;
        else
            AUD(AUD_TX_CH(n)) &= ~0xbu;
    }
}

/* THE ANALOGUE OUTPUT PATH GAINS. A second set of gain fields, separate from
 * the AUD(0x30)/AUD(0x2c) ones, that this driver has never written at all -
 * and every log this port has produced reads them as zero `[L]`.
 *
 * The vendor loads each from its context, in the very functions tx_enable
 * calls per enabled channel `[C]`:
 *
 *   0xd7a418   AUD(0x08)[12:8]  = ctx[0xa0]      channel 0
 *   0xd7a4a0   AUD(0x08)[28:24] = ctx[0xa4]      channel 1
 *   0xd7a51e   AUD(0x0c)[14:8]  = ctx[0xa8]      channel 2, and 0xd7a514
 *              also sets bit 16 there, which ioctl 67's plain |= 0x43 does not
 *
 * Our logs read r08=20432043 and 0c=00000043: all three fields zero, bit 16
 * clear. docs/AUDIO.md already records this exact failure for the DIGITAL
 * volume - "clocked, configured, actively transferring and still silent" -
 * and it is the same trap one layer down. */
static void st_path_gain(unsigned g)
{
    field_set(&AUD(0x08), 0x00001f00u, g);          /* channel 0, 5 bits */
    field_set(&AUD(0x08), 0x1f000000u, g);          /* channel 1, 5 bits */
    field_set(&AUD(0x0c), 0x00007f00u,              /* channel 2, 7 bits */
              (g * 4u) > 127u ? 127u : g * 4u);
}

void yp3_audio_selftest(void)
{
    /* v7 closed the last cheap guess: every digital TX channel combination
     * was tried, alone and together, and none of them made a sound. What the
     * tester DID hear was the amplifier's own noise floor rising and falling
     * with the PA - so the XS4150 and the speaker are alive and P1.30 really
     * is the enable, and the amp is being fed nothing.
     *
     * This is not another ladder over a guess. Three named gain fields that
     * the vendor writes on every playback start have been zero in every log
     * this port has ever produced.
     *
     * One pitch per rung, so anything heard names its own rung. */
    static const unsigned char gain[5] = { 0u, 8u, 16u, 24u, 31u };
    static const unsigned short freq[5] = { 400u, 600u, 900u, 1400u, 2100u };
    unsigned i, k;

    logf("st: ==== v8: the analogue path gains were never written ====");

    if (!audio_block_up)
        yp3_audio_block_init();
    yp3_audio_pll(SAMPR_44);
    ROM_CLK_APPLY(AUDIO_MCLK);

    st_fill(SAMPR_44, 1000);
    yp3_analog_mask = 7u;
    yp3_tx_enable(SAMPR_44, true);
    st_set_gain(7u, 31u);
    st_tx_channels(0xfu);           /* all four: v7 showed none of them alone
                                     * matters, so it is not the variable */

    /* Everything the earlier runs proved lands, applied once. */
    field_set(&AUD(0x00), 0x01000000u, 1);
    field_set(&AUD(0x00), 0x02000000u, 1);
    AUD(0x0c) |= 0x43u;
    AUD(0x0c) |= 0x10000u;          /* 0xd7a514 - ioctl 67 does not set this */
    if (!ROM_CLK_IS_ON(2u))
        ROM_CLK_ENABLE(2u);
    ROM_CLK_DIV(0x2cu, 8u);
    ROM_CLK_APPLY(0x2cu);
    st_pad_walk(2);

    yp3_dma_open();
    st_irqs = 0;
    st_active = true;
    yp3_dma_play(st_tone, sizeof st_tone);
    yp3_irq_enable(DMA_IRQ(PCM_DMA_CH), 2);
    for (i = 0; i < 200 && st_irqs == 0; i++)
        st_wait_ms(1);
    logf("st: digital irqs=%lu 08=%08lx 0c=%08lx", (unsigned long)st_irqs,
         (unsigned long)AUD(0x08), (unsigned long)AUD(0x0c));
    yp3_speaker_pa(true);

    for (k = 0; k < 5; k++) {
        st_path_gain(gain[k]);
        st_fill(SAMPR_44, freq[k]);
        logf("st: %c path-gain=%u %uHz 08=%08lx 0c=%08lx", 'A' + k, gain[k],
             freq[k], (unsigned long)AUD(0x08), (unsigned long)AUD(0x0c));
        st_wait_ms(3000);
        st_path_gain(0u);
        st_wait_ms(700);
    }

    yp3_speaker_pa(false);
    st_active = false;
    yp3_irq_disable(DMA_IRQ(PCM_DMA_CH));
    DMA_CTRL(PCM_DMA_CH) &= ~(DMA_CTRL_GO | DMA_CTRL_CONT);
    yp3_tx_disable();
    st_set_gain(7u, YP3_ANALOG_GAIN);
    yp3_analog_mask = 3u;

    logf("st: ==== done irqs=%lu ====", (unsigned long)st_irqs);
}
#endif /* YP3_AUDIO_SELFTEST */

/* One rate for now. Both halves of rate selection are now recovered - the PLL
 * family and the TX_CTRL[11:8] code - so 11025, 12000, 16000, 22050, 24000,
 * 32000, 44100 and 48000 are all reachable. Adding them means changing this
 * array and the target's HW_SAMPR_CAPS together, and it is deliberately not
 * bundled with the rewrite: this path has never produced a sample, and one
 * variable at a time is the whole reason the previous three cycles were
 * readable. */
static const unsigned long yp3_sampr[] = { SAMPR_44 };

struct pcm_sink builtin_pcm_sink = {
    .caps = {
        .samprs       = yp3_sampr,
        .num_samprs   = 1,
        .default_freq = 0,
        .volume_type  = PCM_SINK_SWVOL,
    },
    .ops = {
        .init     = sink_init,
        .postinit = sink_postinit,
        .set_freq = sink_set_freq,
        .lock     = sink_lock,
        .unlock   = sink_unlock,
        .play     = sink_play,
        .stop     = sink_stop,
    },
};
