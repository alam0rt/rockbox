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
}


void button_init_device(void)
{
    adc_init_device();
    key_init_pmu();
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

static void play_read_pmu(void)
{
    uint8_t status;

    if (!yp3_pmu_read(PMU_REG_KEY_STATUS, &status) || !(status & PMU_KEY_ANY)) {
        if (play_release_pending) {
            play_release_pending = false;
            play_held = false;
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

    play_read_pmu();
    if (play_held)
        btn |= BUTTON_PLAY;                      /* centre, id 0x21 "enter" */

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
