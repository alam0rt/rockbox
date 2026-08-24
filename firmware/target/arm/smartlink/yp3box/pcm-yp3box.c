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
#include "pcm_mixer.h"
#include "kernel.h"
#include "blackbox.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

/* Playback reaches pcmbuf_play_start and then nothing happens: no sound and a
 * frozen elapsed time. Elapsed time only advances as the PCM buffer drains,
 * and the buffer only drains when the transfer-complete IRQ fires, so those
 * two symptoms are one symptom - the DMA never completes a buffer.
 *
 * That is the second row of the table in CLAUDE.md: registers that hold their
 * values while transfers never finish means a module that is off or a clock
 * that was never committed. So log both halves for the DMA engine and the
 * codec block, and read the DMA registers back after arming them - a readback
 * of zero says the module is dead, correct values with no completion says the
 * clock is.
 *
 * The IRQ only counts; logf from interrupt context is not worth the risk. The
 * count is reported from sink_stop, which is the compact, decisive number:
 * zero completions means the engine never ran at all. */

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

    /* Bounded. The vendor spins here unbounded (SRAM 0x80da4e); on a device
     * with no console an unbounded wait is indistinguishable from a dead
     * board, and this function is reached from the codec's own init. */
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

/* --- the audio block's module: 0x23, shared with USB -----------------------
 *
 * 0x40040000 would not hold a value - AUDIO_CTRL and AUDIO_ENABLE read back
 * zero immediately after being written, and the DMA channel read back zero
 * after being programmed. Module off, per the CLAUDE.md table.
 *
 * Which module was not recoverable from the code: /dev/audio0's handler (FIRM
 * 0xd6746c) enables only 0x57, no clk_enable anywhere in FIRM, the SRAM HAL or
 * the boot ROM reaches this block, and the id is not derivable from the base
 * (0x40095000 is 0x5c while the next page 0x40096000 is 0x54). So it was found
 * by comparison: probe/clk80_work.bin holds the gate registers with the vendor
 * running, the black box logs the same words from us, and nine modules
 * differed. A one-boot probe tried all nine and logged AUDIO_CTRL after each:
 *
 *     blk: 25 -> ctrl=00000000      blk: 5a -> ctrl=00000000
 *     blk: 27 -> ctrl=00000000      blk: 58 -> ctrl=00000000
 *     blk: 28 -> ctrl=00000000      blk: 4a -> ctrl=00000000
 *     blk: 22 -> ctrl=00000000      blk: 23 -> ctrl=80000113   <- alive
 *
 * and the DMA registers held their values from that point on: addr and len
 * read back as programmed instead of zero.
 *
 * 0x23 is the USB device module (FIRM 0xd66d98, driver_usbd_params_init), so
 * the two share a power domain. That also explains something that looked wrong
 * earlier: FIRM 0xd66fe4 writes 0x40040300 and 0x40040304 and is reached from
 * the /dev/sd USB path. It is one block serving both.
 *
 * The sharing has a consequence, handled in usb-yp3box.c: USB teardown used to
 * drop this module, which would take the audio block down with it. The vendor
 * never turns it off - it is on in the gate capture - so neither do we.
 *
 * Ordering note for whoever reads the old probe in git history: 0x23 was tried
 * eighth of nine, because it was already identified as "the USB module" and so
 * deprioritised. Being able to name a module is not evidence about what else
 * it gates. */
#define AUDIO_BLOCK_MODULE 0x23u

static void __attribute__((section(".icode"), noinline))
yp3_audio_block_enable(void)
{
    if (!ROM_CLK_IS_ON(AUDIO_BLOCK_MODULE))
        ROM_CLK_ENABLE(AUDIO_BLOCK_MODULE);
    logf("blk: module 23 on, ctrl=%08lx", (unsigned long)AUDIO_CTRL);
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
/* Ruled out: clock 0x21. It is started at FIRM 0xd66196, which sits near
 * audio_start and looked like the audio block's missing clock. It is not - the
 * enclosing function's strings are 'malloc err.' and 'pin = 0x%x, port = %d.',
 * so it is a GPIO pin-mux routine and 0x21 is almost certainly the GPIO
 * controller's clock. Tried on hardware anyway: 0x40040300 still read back
 * zero. Recorded so nobody spends another boot on it. */

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

    /* READ ONLY. Do not write these.
     *
     * A sweep that set gate bits for all 96 module ids hard-hung the device
     * twice, before it could even log its first line. The bits at 0x40080000
     * gate every peripheral including the SPI NOR controller this build
     * executes from, so writing an unknown one stops instruction fetch: the
     * CPU dies on its next instruction with no fault, no timeout and nothing
     * in the ring. CLAUDE.md already records a blind module sweep hanging this
     * device; removing the ROM's acknowledge spin made the sweep survivable in
     * one way and left the real hazard untouched.
     *
     * What is safe, and still worth having, is reading them: this is the first
     * look at which modules are actually powered in our configuration.
     * base 0x40080000, three banks of 32 - 0x70/0x74/0x78 gate, 0x60/0x64/0x68
     * release. ROM 0x2518 reports a module on only when BOTH bits are set. */
    {
        volatile unsigned long *ck = (volatile unsigned long *)0x40080000;
        logf("gate: rel  60=%08lx 64=%08lx 68=%08lx",
             ck[0x60/4], ck[0x64/4], ck[0x68/4]);
        logf("gate: gate 70=%08lx 74=%08lx 78=%08lx",
             ck[0x70/4], ck[0x74/4], ck[0x78/4]);
    }

    logf("codec: ctrl=%08lx (nonzero = block alive)", (unsigned long)AUDIO_CTRL);

    CODEC(0x4c) = (CODEC(0x4c) & ~0x0f000000u) | 0x0e000000u;
    CODEC(0x50) = 27;
    CODEC(0x40) |= 0x80000000u | 1u;    /* run */

    yp3_audio_block_enable();
}

/* --- DMA ------------------------------------------------------------------
 *
 * THIS IS A MENTOR/INVENTRA MUSB CONTROLLER, AND THE AUDIO PATH SHARES IT.
 *
 * 0x40040000 is the USB block - usb-yp3box.c has been writing FADDR at 0x00,
 * POWER at 0x01, INTRUSBE at 0x0b, INDEX at 0x0e, TXCSR at 0x12 and DEVCTL at
 * 0x60 the whole time, which is the MUSB common register map exactly. That is
 * also why module 0x23 gates "the audio block" and the USB device together:
 * it is one peripheral, and the nine-way probe found the right gate for the
 * wrong reason.
 *
 * Once the map is named, every register here decodes:
 *
 *   0x0e   INDEX          selects which endpoint 0x10-0x1f address
 *   0x12   TXCSR          indexed, transmit  (memory -> peripheral)
 *   0x16   RXCSR          indexed, receive   (peripheral -> memory)
 *   0x204  DMA_CNTL       bit 0 ENABLE, 1 DIRECTION, 2 MODE, 3 IRQENABLE,
 *                         bits 4-7 ENDPOINT, bit 8 BUSERROR
 *   0x208  DMA_ADDR
 *   0x20c  DMA_COUNT
 *
 * and the vendor's dma_start at FIRM 0xd7eda4 is textbook MUSB: TXCSR gets
 * 0x9400 (AUTOSET | DMAReqMode | DMAReqEnab) on the transmit branch, RXCSR
 * gets 0xa000 (AUTOCLEAR | DMAReqEnab) on the receive branch.
 *
 * WE TOOK THE RECEIVE BRANCH. dir was 0, so DMA_CNTL bit 1 was clear and the
 * configuration halfword went to RXCSR - we were asking the controller to
 * move 1024 bytes OUT of a USB endpoint FIFO and INTO memory. There is no
 * host, no enumerated endpoint and nothing in that FIFO, so the transfer was
 * armed and could never finish: every register read back perfectly and no
 * transfer ever completed, which is row two of the table in CLAUDE.md and was
 * read for several cycles as a missing clock.
 *
 * Playback is memory -> peripheral, so it is the transmit branch: DIRECTION
 * set, and TXCSR rather than RXCSR.
 *
 * The channel register index stays 0. In the vendor that index is not the
 * endpoint - it is derived separately and comes out 0 for every endpoint
 * except 2 (0xd7edba/0xd7edda), so 0x204/0x208/0x20c is right either way.
 */
#define AUDIO_DMA_PLAY_CH   0
#define AUDIO_DMA_EP        1u          /* INDEX, and DMA_CNTL bits 4-7 */
#define AUDIO_DMA_DIR       1u          /* 1 = transmit, memory -> peripheral */
#define AUDIO_DMA_FLAG      1u
#define AUDIO_DMA_INDEX     REG8(0x4004000e)
#define AUDIO_DMA_TXCSR     REG16(0x40040012)

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
    uint16_t csr, ctrl;

    yp3_dma_enable();

    /* The transmit branch of dma_start, FIRM 0xd7edae..0xd7edb8: select the
     * endpoint, then set AUTOSET | DMAReqMode | DMAReqEnab in TXCSR. The
     * vendor does no read-clear here - it ORs into whatever TXCSR holds. */
    AUDIO_DMA_INDEX = AUDIO_DMA_EP;
    csr = AUDIO_DMA_TXCSR;
    AUDIO_DMA_TXCSR = csr | 0x9400u;

    AUDIO_DMA_ADDR(AUDIO_DMA_PLAY_CH) = (uint32_t)addr;
    AUDIO_DMA_LEN(AUDIO_DMA_PLAY_CH)  = size;

    ctrl = AUDIO_DMA_CTRL(AUDIO_DMA_PLAY_CH);
    ctrl = (ctrl & ~0x6u) | (AUDIO_DMA_DIR << 1);
    if (AUDIO_DMA_FLAG)
        ctrl |= 4u;
    ctrl = (ctrl & ~0xf0u) | 8u | (AUDIO_DMA_EP << 4);
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

static uint32_t pcm_irqs;        /* transfer-complete callbacks taken */
static uint32_t pcm_plays;       /* sink_play calls */

void yp3_audio_irq(void)
{
    uint32_t status = AUDIO_IRQSTAT;

    pcm_irqs++;

    /* The first three only. "pcm stop:" already reports plays against irqs,
     * but it needs a clean stop and no run has yet reached one - they end on a
     * pin reset or a USB unplug. This says the same thing without waiting for
     * that: no line at all means vector 43 never fires, so the DMA is armed
     * and not running. A line with bit 1 clear means it fires but never
     * signals completion, which is a different fault entirely.
     *
     * logf from interrupt context is not something to leave in the tree, but
     * three bounded calls are worth one boot. */
    if (pcm_irqs <= 8)
        logf("audio irq #%lu status=%08lx", (unsigned long)pcm_irqs,
             (unsigned long)status);

    /* BIT 0 IS OUR COMPLETION, NOT BIT 1.
     *
     * Two interrupts have ever reached this port and both carried bit 0 with
     * bit 1 clear - status=0x05 on a13ac321d9 and status=0x01 on 4d9f090271.
     * Testing bit 1 meant both were counted, logged and then dropped: the
     * callback never ran, the next buffer was never armed, and playback
     * stopped after a single 1024-byte transfer. That is exactly the "fires
     * but never signals completion" case the log line above was written to
     * tell apart, and the log told us.
     *
     * The vendor dispatches bit 0 and bit 1 to two separate callbacks (SRAM
     * 0x813774) and vectors-sl6801.S calls the vector "half/full transfer
     * complete", so the two-callback shape is real - but we arm one buffer at
     * a time rather than running a circular transfer, so whichever half the
     * hardware thinks it finished, the buffer we handed it is done. Take
     * either bit as completion. */
    if (status & 3u) {
        const void *addr;
        size_t size;

        if (pcm_play_dma_complete_callback(PCM_DMAST_OK, &addr, &size))
            yp3_dma_play(addr, size);
    }

    AUDIO_IRQSTAT = status | 7u;        /* W1C, after callbacks as vendor does */
}

/* --- the speaker power amplifier -------------------------------------------
 *
 * docs/AUDIO-BOARD.md lists the PA enable as unresolved: "no board PA-enable,
 * speaker mute, jack-detect, or route-select packed descriptor is proved".
 * It is in the dump, reached from the media manager's own logging.
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
 * pin, and not a guess at what to write to it. */
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


static void sink_init(void)
{
    /* The codec block is left off until there is something to play: it
     * reprograms a PLL, and a clock change during LCD or SD bring-up is one
     * more variable in the wrong place. sink_play brings it up on demand. */
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

    /* Clear anything the block has latched before arming.
     *
     * Bits 0, 1 and 2 of IRQSTAT are all write-1-to-clear - the vendor ISR
     * acknowledges the three together with "status | 7" (SRAM 0x81379a) and
     * has no handler for bit 2 at all, which makes bit 2 a flag the block
     * raises and the driver is expected to drop.
     *
     * Every run that produced no interrupt read back irqst=00000004 at this
     * point, and every run that DID interrupt read back 0. A stale bit 2 is
     * the one state those runs have in common, so clear it on the way in
     * rather than arming a block that is still holding one. */
    AUDIO_IRQSTAT = 7u;

    yp3_dma_play(addr, size);

    /* audio_start, FIRM 0xd66fe4: the control word, the enable bits, then the
     * interrupt. Order matters - the vendor enables the channel before the
     * block, and the block before the NVIC. */
    AUDIO_CTRL    = (AUDIO_CTRL & ~0xf0u) | 0x11eu | 0x1u;
    AUDIO_ENABLE &= ~0x3u;
    AUDIO_ENABLE |= 0x3u;
    yp3_irq_enable(IRQ_AUDIO, 8);

    /* BIT 31 OF AUDIO_CTRL IS STICKY, AND WHILE IT IS SET VECTOR 43 NEVER
     * FIRES. Clear it with a whole-word write; a read-modify-write preserves
     * it forever, which is what every previous build did.
     *
     * Five boots of the black box say this and nothing else. The three runs
     * that armed the DMA and stayed silent all read back ctrl=80041113 with
     * irqst=00000004 - bit 31 set, an error bit posted, no interrupt. The one
     * run that ever produced "audio irq" read back ctrl=00010113, irqst=0, and
     * its probe log names the exact write that got it there:
     *
     *     lk try: retry=80041113 off=80041113 on=80041113 whole=00040113
     *
     * Toggling ENABLE off and back on left ctrl untouched at 80041113; only
     * the whole-word store cleared bit 31. So the ENABLE edge below was not
     * what started the transfer - this is. The edge is kept because it was
     * present in the run that worked and costs nothing to reproduce.
     *
     * Bits 2 and 3 still refuse to latch after this, so they are not a
     * transmitter enable; that theory is dead. Written as two stores because
     * that is the sequence the working run executed, and the second one is
     * what leaves bit 16 set. */
    AUDIO_CTRL    = 0x11fu;
    AUDIO_CTRL    = (AUDIO_CTRL & ~0xf0u) | 0x11eu | 0x1u;
    AUDIO_ENABLE |= 0x3u;

    /* Amplifier last, once samples are already flowing, so it is not powered
     * up into a silent line. */
    yp3_speaker_pa(true);

    /* The first two only: after that this is a per-buffer hot path. */
    if (pcm_plays < 2) {
        logf("pcm play #%lu addr=%08lx size=%lu mclk=%lu",
             (unsigned long)pcm_plays, (unsigned long)addr,
             (unsigned long)size, yp3_audio_mclk);
        /* 0x21 is the GENERAL DMA engine at 0x40001000 (dma_open, FIRM
         * 0xd7d7f0), not this block - it is on because SD and the LCD use it,
         * and it has never been evidence about 0x40040000. Kept only so the
         * old logs stay comparable; the probe above is what answers it. */
        logf("pcm mod: gen-dma21=%d codec57=%d (neither gates 0x40040000)",
             ROM_CLK_IS_ON(AUDIO_DMA_MODULE), ROM_CLK_IS_ON(AUDIO_MODULE));
        /* Readback. Zeroes here mean the module is off; correct values with
         * no completion mean the clock was never committed. */
        logf("pcm dma: addr=%08lx len=%lu ctrl=%04x idx=%02x txcsr=%04x",
             (unsigned long)AUDIO_DMA_ADDR(AUDIO_DMA_PLAY_CH),
             (unsigned long)AUDIO_DMA_LEN(AUDIO_DMA_PLAY_CH),
             AUDIO_DMA_CTRL(AUDIO_DMA_PLAY_CH),
             AUDIO_DMA_INDEX, AUDIO_DMA_TXCSR);
        logf("pcm blk: ctrl=%08lx en=%08lx irqst=%08lx iser1=%08lx",
             (unsigned long)AUDIO_CTRL, (unsigned long)AUDIO_ENABLE,
             (unsigned long)AUDIO_IRQSTAT,
             (unsigned long)REG32(0xE000E104));
        logf("pcm cod: 40=%08lx 4c=%08lx 50=%08lx",
             (unsigned long)CODEC(0x40), (unsigned long)CODEC(0x4c),
             (unsigned long)CODEC(0x50));
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

    /* The number that settles it: plays without completions means the engine
     * was armed and never ran. */
    logf("pcm stop: plays=%lu irqs=%lu dmalen=%lu irqst=%08lx",
         (unsigned long)pcm_plays, (unsigned long)pcm_irqs,
         (unsigned long)AUDIO_DMA_LEN(AUDIO_DMA_PLAY_CH),
         (unsigned long)AUDIO_IRQSTAT);

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
