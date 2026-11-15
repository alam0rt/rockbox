/* Button input for the SL6801: an ADC resistor ladder on channel 1. */
/* The vendor reads buttons through /dev/kadc_ch1, not the boot ROM's download
 * key scan. The latter uses different GPIOs and also conflicts with the LCD. */
/* The controller setup and key thresholds below follow the vendor firmware. */
#include "config.h"
#include "button.h"
#include "sl6801-regs.h"
#include "button-target.h"
#include "lcd.h"
#include "font.h"
#include "system.h"
#include "kernel.h"
#include "power.h"
#include "string.h"
/* YP3_PMU_IRQ comes from here; without the include the #ifndef default wins
 * and the build flag reaches nothing. */
#include "yp3-buildstamp.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

#define ADC(o)       ADC_REG(o)     /* base and per-channel regs */
#define ADC_MODULE   0x54u          /* FIRM 0xd65fac: clk_enable(0x54) */
#define KEY_CLOCK    0x21u          /* FIRM 0xd6618c */
#define ADC_CH       1u             /* /dev/kadc_ch1 */
/* /dev/key_io's pin, from the vendor's table at SRAM 0x81a088. The descriptor
 * encoding is ROM 0x77a and 0x7ac:
 *
 *   [19:16] port   [15:11] pin   [10:7] mode nibble
 *   [5:4]   the 2-bit field at port +0x14 (pins 0-15) / +0x18 (pins 16-31)
 *   [3:0]   index+4 into the BIAS table at ROM 0x9c4, whose word supplies a
 *           2-bit field at +0x24 and another at +0x2c
 *
 * so 0x00036000 is port 3, pin 12, mode 0. The low nibble is the part that
 * matters here and the part we had wrong: 0 is not "no bias selected", it is
 * BELOW the table's base of 4, so ROM 0x7ac falls through to 0x80000000 and
 * clears both fields. The pin was left floating. It read high, which is why
 * right worked and left could never be seen at all.
 *
 * The vendor's read at FIRM 0xd662ac polls the pin TWICE, with a different bias
 * each time and a sleep in between:
 *
 *   desc|0x0b -> table[7] = 0x80020002 -> +0x24=2 +0x2c=2, and a pin that then
 *                reads 0 is key id 0x43, the record with level 0: RIGHT
 *   desc|0x05 -> table[1] = 0x80030001 -> +0x24=1 +0x2c=3, and a pin that then
 *                reads 1 is key id 0x42, the record with level 1: LEFT
 *
 * Two keys on one pin, one to each rail, distinguished by which way the pin is
 * biased when you look. The stock capture agrees from the other direction:
 * the stock capture has port 3 pin 12 at +0x24=1 +0x2c=3, the pull-down of the
 * pair - which is the one the vendor's loop applies last. */
#define KEY_IO_PIN      0x00036000u
#define KEY_IO_PIN_UP   (KEY_IO_PIN | 0x0bu)  /* biased high: LOW means right */
#define KEY_IO_PIN_DOWN (KEY_IO_PIN | 0x05u)  /* biased low: HIGH means left */


/* Measured band edges, channel 1: rest 3741-3745, M/up 1975-1978, VOL/down
 * 55-64, noise spread 4-9 counts. */
#define ADC_UP_MIN     1000u
#define ADC_UP_MAX     2800u
#define ADC_DOWN_MAX   1000u


static void adc_init_device(void)
{

    if (!ROM_CLK_IS_ON(ADC_MODULE))
        ROM_CLK_ENABLE(ADC_MODULE);
    udelay(20000);

    /* The key controller clock, exactly as FIRM 0xd6618c does it - through the
     * source wrapper, never the raw setter. */
    ROM_CLK_SET_SRC(KEY_CLOCK, KEY_CLOCK);
    ROM_CLK_APPLY(KEY_CLOCK);

    ADC(0x00) = 0x80180000u;        /* 0xd7df4c, from the struct at 0x81e4b4 */
    ADC(0x04) = 0;
    ADC(0x0c) = 0;
    ADC(0x10) = 0;
    ADC(0x18) = 0;

    /* Only channel 1 is connected to the key ladder. */
    ADC(0x00) |= 1u << ADC_CH;
    ADC_CHCFG(ADC_CH) = 0x0fff0030u;

    /* Port 3 pin 12 is the vendor's /dev/key_io input. LEFT and RIGHT move
     * nothing on the ADC ladder, so they are distinguished through this pin. */
    ROM_GPIO_CFG1(KEY_IO_PIN_UP);

    udelay(5000);                   /* adc_open's delay(5) */
    ADC(0x08) = 0xa800u;            /* 0xd7df3c */
    ADC(0x04) = 0;
    udelay(20000);
}


/* The centre Play/Pause key is not on the ladder and not on 3.12: it is the
 * PMU's on/off key. See the comment at the bottom of this file for the chain
 * of vendor code that establishes that. Arm the three key bits in the PMU
 * interrupt block and drop whatever is already latched, so the first
 * button_read_device() does not see a press left over from power-on.
 *
 * The enable register holds the INVERTED mask (FIRM 0xcf7b0c). The vendor's
 * live mask for this register is 0x7c (SRAM 0x81b034+18); we only consume the
 * three key bits, so the two extra bits it enables are left off. */
static void key_init_pmu(void)
{
    uint8_t status;

    (void)yp3_pmu_write(PMU_REG_KEY_ENABLE, (uint8_t)~PMU_KEY_ANY);
    if (yp3_pmu_read(PMU_REG_KEY_STATUS, &status) && (status & PMU_KEY_ANY))
        (void)yp3_pmu_write(PMU_REG_KEY_STATUS, status & PMU_KEY_ANY);

    /* power_init has already logged the previous run's trace. */
    memset(&yp3_keytrace, 0, sizeof(yp3_keytrace));
    yp3_keytrace.odd_tick = -1;
    yp3_keytrace.magic = YP3_KEYTRACE_MAGIC;
}


/* --- the PMU's interrupt line, OBSERVED, not yet used (yp3-1so.8) -----------
 *
 * The vendor does not poll the PMU: it raises P2.2, IRQ 3, and a bottom half
 * reads 0x30/0x31/0x4a once per event. Here the poll below costs ~25 mailbox
 * transactions a second at ~1.7 ms each, inside the tick. Before the poll is
 * taken away, YP3_PMU_IRQ=1 arms the line exactly as the vendor does and
 * COUNTS, with the poll still running and still the only thing that produces
 * key events - so a boot answers, as numbers: does IRQ 3 fire at all, once
 * per key edge or continuously, before the poll sees the edge or after, and
 * which way the pin idles. NOT YET RUN ON HARDWARE.
 *
 * All `[C]`, each re-read from the dumps for this:
 *   arm   FIRM 0xcf7e70: gpio_config(0x00021705) (literal 0xcf7f94: P2.2,
 *         mode 14), then 0xc48238 -> ROM 0x842(0x00021000, 0x80000003)
 *         (literal 0xcf7f98). ROM 0x842 with bit 31 set: 0x604 writes the
 *         type nibble (3) at port +0x200 + 4*(pin/8), 0x648 clears pending
 *         (+0x214, write 1) and sets the enable bit (+0x210). With bit 31
 *         clear it is 0x660: enable bit off, pending cleared.
 *   hook  FIRM 0xd67890: IRQ 3, priority 7.
 *   isr   SRAM 0x80d414: if pending (ROM 0x872), DISABLE the pin (0x842 with
 *         3), clear pending (ROM 0x896), signal the bottom half.
 *   bh    FIRM 0xcf7c6c: read and write back 0x30/0x31/0x4a, then re-arm with
 *         0x80000003 (literal 0xcf7cb8).
 * Masking the pin from the ISR until the PMU status is cleared is what a
 * level interrupt needs, so type 3 is probably a level `[I]`; polarity `[U]`.
 *
 * Here the "bottom half" is the existing poll: it re-arms after its own read
 * (and clear, when a key bit was set). That bounds the interrupt rate at the
 * poll rate even if the line turns out to be stuck asserted - which it may
 * be, because this port only ever clears the three key bits of 0x31 and the
 * PMU's other sources (0x30, 0x4a, and 0x31 bit 2, which has been seen set)
 * stay latched. The arm line logs the enables and statuses that would say so.
 * Priority 7 is below the PMU lock's BASEPRI (0x30), so the ISR can never cut
 * a mailbox transaction in half. */
#ifndef YP3_PMU_IRQ
#define YP3_PMU_IRQ 0
#endif
#define PMU_INT_IRQ         3u
#define PMU_NVIC_ICER       ((volatile uint32_t *)0xE000E180)

#if YP3_PMU_IRQ
#define PMU_INT_PIN         0x00021000u
#define PMU_INT_PIN_CFG     0x00021705u
#define PMU_INT_ARM         0x80000003u
#define PMU_INT_DISARM      0x00000003u
#define ROM_GPIO_INT_CFG    ((void (*)(unsigned, unsigned))0x843u)
#define ROM_GPIO_INT_PEND   ((unsigned (*)(unsigned))0x873u)
#define ROM_GPIO_INT_CLR    ((void (*)(unsigned))0x897u)
#define PMU_NVIC_ISER       ((volatile uint32_t *)0xE000E100)
#define PMU_NVIC_IPR        ((volatile uint8_t  *)0xE000E400)

static struct {
    volatile uint32_t irqs;         /* ISR entries with the pin pending       */
    volatile uint32_t stray;        /* ISR entries without                    */
    volatile bool     flag;         /* an interrupt since the last poll       */
    uint32_t key_polls;             /* polls that found a key bit             */
    uint32_t key_matched;           /* ...with the interrupt already taken    */
    uint32_t flag_nokey;            /* interrupt taken, no key bit in 0x31    */
    uint32_t idle_hi, idle_lo;      /* pin level on quiet polls               */
    uint32_t ev_hi, ev_lo;          /* pin level on polls with a key bit      */
    uint8_t  nokey_st;              /* 0x31 at the last flag_nokey            */
    uint32_t logged_irqs;
    unsigned polls;
    bool     armed;
} pmu_irq;

static void pmu_irq_arm(void)
{
    uint8_t en19 = 0, en1a = 0, en4b = 0, st30 = 0, st31 = 0, st4a = 0;

    (void)yp3_pmu_read(0x19, &en19);
    (void)yp3_pmu_read(0x1a, &en1a);
    (void)yp3_pmu_read(0x4b, &en4b);
    (void)yp3_pmu_read(0x30, &st30);
    (void)yp3_pmu_read(0x31, &st31);
    (void)yp3_pmu_read(0x4a, &st4a);
    logf("pmuirq: arm en 19=%02x 1a=%02x 4b=%02x st 30=%02x 31=%02x 4a=%02x pin=%d",
         en19, en1a, en4b, st30, st31, st4a, (int)((GPIO_IN(2) >> 2) & 1u));

    ROM_GPIO_CFG1(PMU_INT_PIN_CFG);
    ROM_GPIO_INT_CFG(PMU_INT_PIN, PMU_INT_ARM);
    PMU_NVIC_IPR[PMU_INT_IRQ] = 7u << 4;
    PMU_NVIC_ISER[PMU_INT_IRQ >> 5] = 1u << (PMU_INT_IRQ & 31);
    pmu_irq.armed = true;
    logf("pmuirq: armed, pin=%d type=%lx en=%lx pend=%lx",
         (int)((GPIO_IN(2) >> 2) & 1u),
         (unsigned long)REG32(GPIO_PORT(2) + 0x200),
         (unsigned long)REG32(GPIO_PORT(2) + 0x210),
         (unsigned long)REG32(GPIO_PORT(2) + 0x214));
}

/* From the poll, after its read and clear. Tick context. */
static void pmu_irq_observe(uint8_t status)
{
    bool key = (status & PMU_KEY_ANY) != 0;
    bool hi = ((GPIO_IN(2) >> 2) & 1u) != 0;
    bool flag = pmu_irq.flag;

    if (!pmu_irq.armed)
        return;
    if (key) {
        pmu_irq.key_polls++;
        if (flag)
            pmu_irq.key_matched++;
        if (hi) pmu_irq.ev_hi++; else pmu_irq.ev_lo++;
    } else {
        if (flag) {
            pmu_irq.flag_nokey++;
            pmu_irq.nokey_st = status;
        }
        if (hi) pmu_irq.idle_hi++; else pmu_irq.idle_lo++;
    }
    if (flag) {
        pmu_irq.flag = false;
        ROM_GPIO_INT_CFG(PMU_INT_PIN, PMU_INT_ARM);     /* the bh's re-arm */
    }
    /* Every ~5 s, and only when the count moved, so a quiet line is quiet
     * in the ring too. A line stuck asserted shows as irqs climbing by one
     * per poll with nokey beside it. */
    if (++pmu_irq.polls >= 125) {
        pmu_irq.polls = 0;
        if (pmu_irq.irqs != pmu_irq.logged_irqs || pmu_irq.stray) {
            pmu_irq.logged_irqs = pmu_irq.irqs;
            logf("pmuirq: irqs=%lu stray=%lu key=%lu matched=%lu nokey=%lu(st %02x) "
                 "pin idle %lu/%lu ev %lu/%lu hi/lo",
                 (unsigned long)pmu_irq.irqs, (unsigned long)pmu_irq.stray,
                 (unsigned long)pmu_irq.key_polls,
                 (unsigned long)pmu_irq.key_matched,
                 (unsigned long)pmu_irq.flag_nokey, pmu_irq.nokey_st,
                 (unsigned long)pmu_irq.idle_hi, (unsigned long)pmu_irq.idle_lo,
                 (unsigned long)pmu_irq.ev_hi, (unsigned long)pmu_irq.ev_lo);
        }
    }
}
#endif /* YP3_PMU_IRQ */

/* Vector 3 (vectors-sl6801.S). Without YP3_PMU_IRQ nothing enables it, and
 * if it ever fired it is masked, as yp3_spurious_irq would. */
void yp3_pmu_gpio_irq(void)
{
#if YP3_PMU_IRQ
    if (ROM_GPIO_INT_PEND(PMU_INT_PIN)) {
        ROM_GPIO_INT_CFG(PMU_INT_PIN, PMU_INT_DISARM);
        ROM_GPIO_INT_CLR(PMU_INT_PIN);
        pmu_irq.irqs++;
        pmu_irq.flag = true;
        return;
    }
    /* Another port-2 pin, or nothing: not ours to clear, so after a few of
     * them stop listening rather than loop. */
    if (++pmu_irq.stray < 8)
        return;
#endif
    PMU_NVIC_ICER[PMU_INT_IRQ >> 5] = 1u << (PMU_INT_IRQ & 31);
}

void button_init_device(void)
{
    adc_init_device();
    key_init_pmu();
#if YP3_PMU_IRQ
    pmu_irq_arm();
#endif
}

/* MEASURED on hardware with the probe above, channel 1:
 *
 *     nothing   3741-3745        M / up      1975-1978
 *     VOL/down    55-  64        noise spread 4-9 counts
 *
 * The bands are hundreds of counts apart and noise is single digits, so plain
 * midpoint thresholds are safe and no averaging is needed - Rockbox's own
 * button_read repetition handles the rest.
 *
 *     M / up              -> BUTTON_MENU   menu
 *     VOL / down          -> BUTTON_VOL    volume / back
 *     forward/right       -> BUTTON_NEXT   next
 *     back / left         -> BUTTON_PREV   previous
 *     centre Play/Pause   -> BUTTON_PLAY   PMU on/off key, see below
 *
 * Right and left share pin 3.12 and are told apart by which way the pin is
 * biased when it is sampled - see KEY_IO_PIN above for the vendor's version.
 * The vendor sleeps a tick between changing the bias and reading; we cannot
 * sleep here, because button_read_device runs from the tick task itself.
 *
 * So the bias ALTERNATES between calls. Each call reads the level the previous
 * call's bias settled, then sets up the other bias for the next one, which
 * gives the pin a full tick to settle instead of a busy-wait. Each key is
 * therefore sampled at half the tick rate, and the two levels are LATCHED so
 * both are reported every call - without the latch a held key would appear to
 * flap press/release/press at 50 Hz and Rockbox's repeat handling would never
 * see it as held.
 */
static bool bias_is_up = true;      /* which bias the pin is carrying now */
static bool right_held, left_held;

/* The centre key reports EDGES, not a level, so its state has to be latched
 * across calls the way the biased pin's two keys are. A tap that begins and
 * ends inside one tick sets both bits in the same read; report the press on
 * this call and the release on the next, or the tap disappears. */
static bool play_held, play_release_pending;

/* yp3-0gn.10, see struct yp3_keytrace. Tick context: plain stores only. */
struct yp3_keytrace yp3_keytrace __attribute__((section(".yp3snap")));

static void keytrace_note(bool was_held, uint8_t status)
{
    struct yp3_keytrace *k = &yp3_keytrace;

    k->last_tick = current_tick;
    k->seen |= status;
    if ((status & (uint8_t)~PMU_KEY_ANY) && k->odd_tick < 0) {
        k->odd_st = status;
        k->odd_tick = current_tick;
    }
    if (play_held && !was_held) {
        k->downs++;
        k->down_tick = current_tick;
        k->down_st = status;
        k->chg_down = charger_inserted();
    } else if (!play_held && was_held) {
        k->ups++;
        k->up_tick = current_tick;
    }
    k->held = play_held;
}

static void play_read_pmu(void)
{
    uint8_t status;
    bool was_held = play_held;

    if (!yp3_pmu_read(PMU_REG_KEY_STATUS, &status))
        status = 0;                 /* a failed read is not recorded */
    else
        keytrace_note(was_held, status);    /* note the raw byte first */

#if YP3_PMU_IRQ
    /* Before the clear below: the pin is sampled while the event is still
     * latched in the PMU, which is what gives the polarity. */
    pmu_irq_observe(status);
#endif
    if (!(status & PMU_KEY_ANY)) {
        if (play_release_pending) {
            play_release_pending = false;
            play_held = false;
            keytrace_note(was_held, 0);
        }
        return;
    }

    (void)yp3_pmu_write(PMU_REG_KEY_STATUS, status & PMU_KEY_ANY);

    if (status & PMU_KEY_PRESS) {
        play_held = true;
        /* PMU_KEY_LONG arrives while the key is still down: not a release. */
        play_release_pending = (status & PMU_KEY_RELEASE) != 0;
    } else if (status & PMU_KEY_RELEASE) {
        play_held = false;
        play_release_pending = false;
    }
    keytrace_note(was_held, status);
}

int button_read_device(void)
{
    int btn = 0;
    unsigned v = ADC_DATA(ADC_CH) & 0xfffu;
    bool level = ((GPIO_IN(3) >> 12) & 1u) != 0;

    if (v < ADC_DOWN_MAX)
        btn |= BUTTON_VOL;                       /* VOL / down, ~60 */
    else if (v >= ADC_UP_MIN && v < ADC_UP_MAX)
        btn |= BUTTON_MENU;                      /* M / up, ~1976 */

    /* Read what the bias applied last time has settled, then flip it. */
    if (bias_is_up) {
        right_held = !level;  /* pulled down against the up bias */
        ROM_GPIO_CFG1(KEY_IO_PIN_DOWN);
    } else {
        left_held = level;    /* pulled up against the down bias */
        ROM_GPIO_CFG1(KEY_IO_PIN_UP);
    }
    bias_is_up = !bias_is_up;

    if (right_held)
        btn |= BUTTON_NEXT;                      /* forward / right, id 0x43 */
    if (left_held)
        btn |= BUTTON_PREV;                      /* back / left, id 0x42 */

    /* Every fourth tick only. Each read is a PMU mailbox transaction, and
     * measured on hardware one takes up to 3.4 ms - 100 a second was a third
     * of the CPU spent spinning inside the tick (pmu=116/3386us, tk=6.7 ms of
     * every 10). The PMU latches press and release as edges, and a press and
     * release seen in the same read are already split across two calls
     * below, so a slower poll loses no taps; it only adds up to 40 ms of
     * latency to the centre key. */
    static unsigned pmu_div;
    if (++pmu_div >= 4) {
        pmu_div = 0;
        play_read_pmu();
    }
    if (play_held)
        btn |= BUTTON_PLAY;                      /* centre, id 0x21 "enter" */

    /* The rig's scripted presses, so an unattended cycle can walk the UI to a
     * track and start it. Injected here rather than into the button queue so a
     * synthetic press is indistinguishable from a real one all the way up:
     * repeat, long-press and action.c all see it. Compiles to nothing without
     * YP3_RIG_KEYS. See rig-yp3box.c. */
    btn |= yp3_rig_buttons();

    return btn;
}

/* THE CENTRE KEY: the PMU's on/off key, reported as vendor key id 0x21
 * "enter".
 *
 * It moves no ADC channel and does not touch 3.12 under either bias, and the
 * vendor opens exactly three key devices - /dev/kadc_ch1, /dev/key_io and
 * /dev/key_onoff. The chain that closes it, all read out of the dump:
 *
 *   FIRM 0xcfe214  the key manager opens all three devices, passing the SAME
 *                  callback, 0xcfe645, to each. /dev/key_onoff gets key ids
 *                  0x37 and 0x47 - both power, not the centre key.
 *   SRAM 0x81d490  holds 0xcfe645 at run time, and FIRM 0xd47c40 is what
 *                  stores it: the key manager registers that callback with
 *                  the /dev/pmu driver (opened at FIRM 0xd47bdc), not with
 *                  key_onoff.
 *   FIRM 0xd47784  the PMU event dispatcher. Bit 0 of the event word logs
 *                  "pmu event power key press", bit 1 "release", bit 2
 *                  "long press", and each calls that callback.
 *   FIRM 0xcfe644  the callback: event 1 -> key id 0x21 event 0x10,
 *                  event 2 -> id 0x21 event 0x30, event 4 -> id 0x47.
 *                  0x21 is "enter" (FIRM 0xcfe780, string 0xc4b94a), which is
 *                  the centre Play/Pause key; the LONG press is what becomes
 *                  the power key, which is why holding centre powers the
 *                  stock firmware down.
 *   FIRM 0xcf7c6c  where the event word comes from: PMU registers 0x30, 0x31
 *                  and 0x4a are read, written straight back to clear, and
 *                  turned into event bits by 0xcf7b6c - which takes the key
 *                  bits from 0x31 alone: bit 6 -> 1, bit 4 -> 2, bit 5 -> 4.
 *
 * The vendor reaches all this through an interrupt: the PMU raises GPIO 2.2
 * (descriptor 0x21000, ROM interrupt-status word at port base +0x214), FIRM
 * registers ROM 0x8137ac on IRQ 3 (FIRM 0xd678be), and a "pmu_isr_bh" thread
 * then does the register read above. Nothing about the status bits needs the
 * interrupt, so this driver just polls them from the tick task like the rest
 * of the keys; GPIO 2.2 and IRQ 3 stay untouched.
 *
 * The GPIO side is now exhausted as an explanation, which agrees: in the stock
 * capture, pin 3.12 is the ONLY pin that is mode 0 with a bias configured.
 * Every other mode-0 pin has both bias fields clear, i.e. nothing is wired to
 * it that stock cares to read.
 */
