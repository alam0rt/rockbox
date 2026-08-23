/*
 * PCM playback for the Smartlink SL6801.
 *
 * Three separate blocks have to be brought up, and they are independent:
 *
 *   codec / I2S at 0x4009b000   module 0x57, clocks 0x1f and 0x38, fed by a
 *                               dedicated audio PLL (24.576 or 22.579 MHz)
 *   PCM DMA at 0x40040200       one channel per direction, buffer address and
 *                               length per transfer
 *   IRQ 43                      half- and full-transfer completion
 *
 * Everything here is transcribed from the vendor's /dev/audio0 driver and
 * observed hardware state.
 *
 * The Rockbox side is the software-volume sink path, which is what makes this
 * driver simple: with PCM_SINK_SWVOL the core hands us chunks out of its own
 * double buffer (pcm_sw_volume.c, PCM_PLAY_DBL_BUF_SAMPLES, max 4 KB), each at
 * a stable SRAM address. So "program a transfer, get an interrupt, ask for the
 * next" is the whole driver - no circular buffer, no descriptor chain.
 */
#include "config.h"
#include "system.h"
#include "audio.h"
#include "pcm.h"
#include "pcm-internal.h"
#include "pcm_sink.h"
#include "sl6801-regs.h"

/* --- audio master clock ---------------------------------------------------
 *
 * The codec runs off a dedicated PLL and the vendor picks between exactly two
 * settings by testing the requested rate against 8000. FIRM 0xd7d6a4, and again
 * at 0xd7d4d4:
 *
 *     r2 = 8000
 *     r3 = rate % r2
 *     pll = (r3 == 0) ? 24576000 : 22579000
 *     set_pll(pll)
 *
 * 24576000 is 512 * 48000 and 22579000 is the achievable approximation of
 * 512 * 44100, so this is the usual "48k family or 44.1k family" split: 8000,
 * 16000, 24000, 32000 and 48000 divide by 8000, while 11025, 22050 and 44100 do
 * not.
 *
 * set_pll itself is FIRM's HAL at 0x80da34. It is NOT callable from here - it
 * lives in the loaded SRAM HAL at 0x800000-0x830000, which our own .data/.bss
 * overwrite - so it is reproduced. Both constant pairs are read straight out of
 * its literal pool (0x80da88..0x80da9c):
 *
 *     want 24576000:  mult = 0x3126 (12582), pre = 3
 *     otherwise:      mult = 0x186c2 (100034), pre = 2, achieves 22579000
 *
 *     while (CLK(0x14) & (1 << 11)) ;             PLL busy
 *     CLK(0x10) = (CLK(0x10) & 0xfc1f0000) | 0x3000 | (pre << 5);
 *     CLK(0x14) = (CLK(0x14) & 0x3800) | 0x80000000 | 0x400 | (mult << 14);
 *
 * The vendor also caches the achieved rate at SRAM 0x0081b08c; that is its
 * variable, not a register, so it is kept here instead.
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

    while (ACLK(0x14) & (1u << 11)) ;
    ACLK(0x10) = (ACLK(0x10) & 0xfc1f0000u) | 0x3000u | (pre << 5);
    ACLK(0x14) = (ACLK(0x14) & 0x3800u) | 0x80000000u | 0x400u | (mult << 14);
    yp3_audio_mclk = f48 ? 24576000ul : 22579000ul;
}

/* --- codec block ----------------------------------------------------------
 *
 * FIRM 0xd7e844, the /dev/audio0 ioctl 3 handler ("enable"), and its mirror at
 * 0xd7e88e. Module 0x57 plus clocks 0x1f and 0x38: both halves, as always.
 *
 * The source select on clock 0x39 is what the vendor does the first time round
 * (FIRM 0xd7e81a, guarded by a flag of its own at SRAM 0x0081e808+0x22 that is
 * clear until it has run once). We have no such flag, and selecting a source is
 * idempotent, so it runs on every enable.
 *
 * .icode for the same reason as the PLL above. */
#define AUDIO_MODULE    0x57
#define AUDIO_CLK_A     0x1f
#define AUDIO_CLK_B     0x38
#define AUDIO_CLK_SRC   0x39
#define AUDIO_CLK_SRC_FROM 0x2a         /* cpu pll */

#define CODEC(o) (*(volatile uint32_t *)(0x4009b000u + (o)))

static void __attribute__((section(".icode"), noinline))
yp3_codec_enable(unsigned long rate)
{
    ROM_CLK_SET_SRC(AUDIO_CLK_SRC, AUDIO_CLK_SRC_FROM);
    ROM_CLK_APPLY(0);
    yp3_audio_pll(rate);
    if (!ROM_CLK_IS_ON(AUDIO_MODULE))
        ROM_CLK_ENABLE(AUDIO_MODULE);
    ROM_CLK_APPLY(AUDIO_CLK_A);
    ROM_CLK_APPLY(AUDIO_CLK_B);

    CODEC(0x4c) = (CODEC(0x4c) & ~0x0f000000u) | 0x0e000000u;
    CODEC(0x50) = 27;
    CODEC(0x40) |= 0x80000000u | 1u;    /* run */
}

/* --- DMA ------------------------------------------------------------------
 *
 * The vendor's audio_dma_setup writes three pieces of state before arming
 * channel 0: format/mode at 0x4004000e, the playback configuration halfword
 * at 0x40040016, and the channel registers. The observed playback caller at
 * SRAM 0x813738 passes dir=0, mode=1, flag=1. The old port only programmed the
 * channel registers with mode=0, flag=0, leaving the audio block unconfigured.
 */
#define AUDIO_DMA_PLAY_CH   0
#define AUDIO_DMA_MODE      1u
#define AUDIO_DMA_DIR       0u
#define AUDIO_DMA_FLAG      1u
#define AUDIO_DMA_FORMAT    REG8(0x4004000e)
#define AUDIO_DMA_PLAY_CFG  REG16(0x40040016)

#define AUDIO_DMA_MODULE   0x21u        /* FIRM 0xd7d834: dma_open */

/* dma_open starts the DMA engine on first use. ROM_CLK_ENABLE spins on an
 * acknowledge bit, so keep this transition in SRAM just like the audio PLL
 * and codec clock setup. */
static void __attribute__((section(".icode"), noinline))
yp3_dma_enable(void)
{
    if (!ROM_CLK_IS_ON(AUDIO_DMA_MODULE))
        ROM_CLK_ENABLE(AUDIO_DMA_MODULE);
}

static void yp3_dma_play(const void *addr, size_t size)
{
    uint16_t cfg, ctrl;

    yp3_dma_enable();

    /* Faithful dir=0/mode=1/flag=1 branch of audio_dma_setup. */
    AUDIO_DMA_FORMAT = AUDIO_DMA_MODE;
    cfg = AUDIO_DMA_PLAY_CFG & ~0x800u;
    AUDIO_DMA_PLAY_CFG = cfg | 0xa000u;

    AUDIO_DMA_ADDR(AUDIO_DMA_PLAY_CH) = (uint32_t)addr;
    AUDIO_DMA_LEN(AUDIO_DMA_PLAY_CH)  = size;

    ctrl = AUDIO_DMA_CTRL(AUDIO_DMA_PLAY_CH);
    ctrl = (ctrl & ~0x6u) | (AUDIO_DMA_DIR << 1);
    if (AUDIO_DMA_FLAG)
        ctrl |= 4u;
    ctrl = (ctrl & ~0xf0u) | 8u | (AUDIO_DMA_MODE << 4);
    AUDIO_DMA_CTRL(AUDIO_DMA_PLAY_CH) = ctrl | 1u;
}

/* --- IRQ 43 ---------------------------------------------------------------
 *
 * The vendor's handler is at 0x813774: read the status word, call the half- and
 * full-transfer callbacks, then write status|7 back to clear (W1C).
 *
 * We only act on full-transfer completion. The halfway point is still cleared;
 * otherwise it re-asserts immediately. */
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

void yp3_audio_irq(void)
{
    uint32_t status = AUDIO_IRQSTAT;

    if (status & 2u) {                  /* full transfer complete */
        const void *addr;
        size_t size;

        if (pcm_play_dma_complete_callback(PCM_DMAST_OK, &addr, &size))
            yp3_dma_play(addr, size);
    }

    AUDIO_IRQSTAT = status | 7u;        /* W1C, after callbacks as vendor does */
}

/* --- the sink ------------------------------------------------------------- */


static void sink_init(void)
{
    /* The codec block is left off until there is something to play: it
     * reprograms a PLL, and a clock change during LCD or SD bring-up is one
     * more variable in the wrong place. sink_play brings it up on demand. */
}

static void sink_postinit(void)
{
}

static bool codec_running;

static void sink_set_freq(uint16_t freq)
{
    /* freq is a Rockbox sample-rate INDEX, not a rate. Only the family matters
     * to the PLL, and the port declares one rate, so this is the whole job
     * until the within-family divider is found. */
    (void)freq;
    yp3_codec_enable(SAMPR_44);
    codec_running = true;
}

static void sink_lock(void)
{
    yp3_irq_disable(IRQ_AUDIO);
}

static void sink_unlock(void)
{
    yp3_irq_enable(IRQ_AUDIO, 8);
}

static void sink_play(const void *addr, size_t size)
{
    if (!codec_running)
        sink_set_freq(0);

    yp3_dma_play(addr, size);

    /* audio_start, FIRM 0xd66fe4: the control word, the enable bits, then the
     * interrupt. Order matters - the vendor enables the channel before the
     * block, and the block before the NVIC. */
    AUDIO_CTRL    = (AUDIO_CTRL & ~0xf0u) | 0x11eu | 0x1u;
    AUDIO_ENABLE |= 0x3u;
    yp3_irq_enable(IRQ_AUDIO, 8);
}

static void sink_stop(void)
{
    /* Disable the interrupt before stopping DMA, so completion cannot request
     * another transfer. */
    yp3_irq_disable(IRQ_AUDIO);
    AUDIO_ENABLE &= ~0x3u;
    AUDIO_CTRL   &= ~0x102u;
    AUDIO_CTRL   &= ~0x1u;
    AUDIO_DMA_CTRL(AUDIO_DMA_PLAY_CH) &= ~1u;
}

/* One rate for now. The PLL family select is known and implemented, but the
 * divider that picks a rate WITHIN a family is not, and declaring rates we
 * cannot actually produce would resample everything to the wrong pitch. */
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
