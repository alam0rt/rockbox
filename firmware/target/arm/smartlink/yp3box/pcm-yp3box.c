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
    logf("aud: init ctrl=%08lx rx=%08lx tx=%08lx",
         (unsigned long)AUD(0x00), (unsigned long)AUD_RX_CTRL,
         (unsigned long)AUD_TX_CTRL);
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

static void yp3_analog_enable(void)
{
    /* Already up? Bit 29 of each channel's power register reads back as the
     * state flag - the vendor's own gain ioctls test exactly this. */
    if ((AUD(0x50) & (1u << 29)) && (AUD(0x54) & (1u << 29)))
        return;

    AUD(0x30) |= 0x01u;                 /* channel 0 present */
    AUD(0x30) |= 0x10u;                 /* channel 1 present */
    AUD(0x40) |= 0x200u;
    AUD(0x4c) |= 0x20000u;
    AUD(0x4c) |= 0x10000u;
    AUD(0x40)  = (AUD(0x40) & ~0x30u) | 0x20u;
    AUD(0x4c) |= 0x100000u;
    AUD(0x40) |= 0x800u;
    AUD(0x50) |= 0x8000u;               /* pre-charge */
    AUD(0x54) |= 0x8000u;
    udelay(10000);
    AUD(0x50) |= 0x10000000u;           /* enable */
    AUD(0x54) |= 0x10000000u;
    AUD(0x50) |= 0x20000000u;           /* up */
    AUD(0x54) |= 0x20000000u;
    AUD(0x4c) |= 0x30u;
    AUD(0x4c) |= 0x03u;
    udelay(10000);
    AUD(0x50) |= 0xfffu;                /* the vendor's mvn/lsr idiom */
    AUD(0x54) |= 0xfffu;
    udelay(10000);
    AUD(0x4c) &= ~0x03u;
    field_set(&AUD(0x30), 0x001f0000u, YP3_ANALOG_GAIN);
    field_set(&AUD(0x30), 0x1f000000u, YP3_ANALOG_GAIN);

    /* audio_pa_bias, FIRM 0xd7ab90: 0x0d per analogue channel. */
    AUD(0x20) |= 0x00000d0du;

    logf("aud: analog 30=%08lx 4c=%08lx 50=%08lx 54=%08lx",
         (unsigned long)AUD(0x30), (unsigned long)AUD(0x4c),
         (unsigned long)AUD(0x50), (unsigned long)AUD(0x54));
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
static void yp3_tx_enable(unsigned long rate, bool stereo)
{
    unsigned n;

    AUD(0x00) |= 0x80000022u;
    AUD(0x00) |= 0x40000000u;
    AUD(0xfc) &= ~0x1fu;

    yp3_analog_enable();

    AUD_TX_CTRL &= ~2u;                 /* stop while reconfiguring */
    AUD_TX_CTRL |= 1u;

    /* Volume to zero first, then the real value - the vendor's two-step, which
     * is there to keep the transition out of the speaker.
     *
     * THESE DEFAULT TO ZERO IN THE VENDOR. audio_hw_init leaves the four
     * playback volumes at 0 and only the capture side at 0x1ff, so a block
     * that is clocked, configured and actively transferring is still silent
     * until this runs. That is exactly the failure that reads as "the DMA is
     * not working", and it is why the maximum goes in here unconditionally:
     * Rockbox attenuates in software (PCM_SINK_SWVOL), so the hardware wants
     * to sit at unity. */
    for (n = 0; n < 4; n++)
        field_set(&AUD_TX_VOL(n), AUD_VOL_MAX, 0);
    for (n = 0; n < 4; n++)
        field_set(&AUD_TX_VOL(n), AUD_VOL_MAX, AUD_VOL_MAX);

    field_set(&AUD_TX_CTRL, 0xf00u, yp3_rate_code(rate));
    AUD_TX_FMT = (AUD_TX_FMT & ~4u) | (stereo ? 4u : 0u) | 0x80u;

    AUD(AUD_TX_CH(0)) = (AUD(AUD_TX_CH(0)) & ~8u) | 1u;
    AUD(AUD_TX_CH(1)) = (AUD(AUD_TX_CH(1)) & ~8u) | 1u;
    AUD(AUD_TX_CH(2)) &= ~0xbu;
    AUD(AUD_TX_CH(3)) &= ~0xbu;

    AUD_TX_CTRL |= 2u;                  /* run */

    logf("aud: tx ctrl=%08lx fmt=%08lx code=%u vol=%08lx",
         (unsigned long)AUD_TX_CTRL, (unsigned long)AUD_TX_FMT,
         yp3_rate_code(rate), (unsigned long)AUD_TX_VOL(0));
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
static void yp3_dma_play(const void *addr, size_t size)
{
    DMA_DST(PCM_DMA_CH) = AUD_TX_FIFO;
    DMA_SRC(PCM_DMA_CH) = (uint32_t)addr;
    DMA_LEN(PCM_DMA_CH) = (DMA_LEN(PCM_DMA_CH) & ~DMA_LEN_MASK)
                        | (size & DMA_LEN_MASK);
    DMA_CTRL(PCM_DMA_CH) = (DMA_CTRL(PCM_DMA_CH) & ~DMA_CTRL_CONT)
                         | DMA_CTRL_GO;
}

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
    if (pcm_irqs <= 8)
        logf("dma irq #%lu st=%08lx", (unsigned long)pcm_irqs,
             (unsigned long)status);

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
        logf("pcm ch: 220=%08lx 250=%08lx iser1=%08lx",
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
