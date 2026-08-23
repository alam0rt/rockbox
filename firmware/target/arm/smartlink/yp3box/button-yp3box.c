/* Button input for the SL6801: an ADC resistor ladder on channel 1.
 *
 * The vendor reads buttons through /dev/kadc_ch1, not through GPIOs. The
 * previous driver here read the boot ROM's download-key scan (P1.1, P1.23,
 * P1.22, P3.0), which is a different mechanism entirely - and P3.0 turned out to
 * be the pad whose mode-2 setting blocks the LCD data bus, so reading it as a
 * button was doubly wrong.
 *
 * The whole path was reversed from the vendor rather than guessed:
 *
 *   FIRM 0xd65fac   clk_enable(0x54) once, then a 2000-tick settle
 *                   cfg[ch] = { 1, 0, 0x0fff0030 } at SRAM 0x81d920, 12 bytes
 *                   per channel, then adc_open(cfg)
 *   FIRM 0xd65f88   adc_open: adc_init(cfg), delay 5, then 0xd7df3c
 *   FIRM 0xd7dee4   adc_init: struct at 0x81e4b4 gets base 0x40096000 and the
 *                   magic 0x80180000, then the cfg block is copied in and
 *                   applied by 0xd7df4c
 *   FIRM 0xd7df4c   base[0x00] = 0x80180000, then per channel n:
 *                     base[0x00] |= cfg[n][0] << n          enable
 *                     base[0x10] |= cfg[n][1] << (4*n)      mode
 *                     base[0x20 + 0x10*n] = cfg[n][2]       channel config
 *   FIRM 0xd7df3c   base[0x08] = 0xa800, then base[0x04] = 0
 *   SRAM 0x8156d0   read: value = base[0x24 + 0x10*ch]
 *
 * The key controller's own clock is set up at FIRM 0xd6618c with
 * clk_set_source(0x21, 0x21) through the wrapper at 0x8050f2, then started via
 * 0x8051ec -> ROM 0x27a0.
 */
#include "config.h"
#include "button.h"
#include "sl6801.h"
#include "button-target.h"
#include "breadcrumb.h"
#include "lcd.h"
#include "font.h"
#include "system.h"

#define ADC(o)       ADC_REG(o)     /* base and per-channel regs: sl6801.h */
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
 * biased when you look. probe/gpio_work.bin agrees from the other direction:
 * the stock capture has port 3 pin 12 at +0x24=1 +0x2c=3, the pull-down of the
 * pair - which is the one the vendor's loop applies last. */
#define KEY_IO_PIN      0x00036000u
#define KEY_IO_PIN_UP   (KEY_IO_PIN | 0x0bu)   /* biased high: LOW means right */
#define KEY_IO_PIN_DOWN (KEY_IO_PIN | 0x05u)   /* biased low:  HIGH means left */

/* DIAGNOSTIC: draw raw ADC values on the panel and never return, so the ladder
 * can be MEASURED instead of derived. The vendor's own threshold table (SRAM
 * 0x819fc8) gives eight ascending bounds - 80, 256, 592, 896, 1040, 1280, 1472,
 * 1824 - but its per-record key byte reads 0 for every entry and the middle word
 * is 0x45 for the first seven and 0x25 for the rest, so it does not say which
 * band is which button. docs/BUTTONS.md has flagged that as unresolved since it
 * was written. With the display working, measuring takes one boot. */
#define YP3_ADC_PROBE 0

/* Measured band edges, channel 1: rest 3741-3745, M/up 1975-1978, VOL/down
 * 55-64, noise spread 4-9 counts. */
#define ADC_UP_MIN     1000u
#define ADC_UP_MAX     2800u
#define ADC_DOWN_MAX   1000u


static void adc_init_device(void)
{
    unsigned ch;

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

    /* The vendor enables only the channel it opens (FIRM 0xd65fac sets
     * cfg[ch][0] = 1 for that one and leaves the rest zero). While the probe is
     * hunting for the buttons that are NOT on channel 1, enable all TEN: an
     * unconnected input just reads noise, and finding the missing keys in one
     * boot is worth the deviation. Narrow this back to ADC_CH once the ladder is
     * settled.
     *
     * Ten, because adc_init's copy loop at FIRM 0xd7df4c ends on cmp r2,#10 and
     * the vendor registers /dev/kadc_ch0 through ch9. The previous two rounds
     * stopped at eight, so channels 8 and 9 have never been read - which is the
     * first place to look for the two keys that move nothing else. */
    for (ch = 0; ch < ADC_NCHAN; ch++) {
        ADC(0x00) |= 1u << ch;
        ADC_CHCFG(ch) = 0x0fff0030u;   /* cfg[n][2], FIRM literal at 0xd66008 */
    }

    /* Port 3 pin 12 as a GPIO input - the vendor's /dev/key_io, whose table at
     * SRAM 0x81a088 has two records on this pin emitting 0x42 at level 1 and
     * 0x43 at level 0. LEFT and RIGHT move nothing on the ADC ladder, so this is
     * where they must come from. The full-state diff already showed this pin
     * differing: stock runs it mode 0 pull 0, we leave it at mode 15 pull 1. */
    ROM_GPIO_CFG1(KEY_IO_PIN_UP);

    udelay(5000);                   /* adc_open's delay(5) */
    ADC(0x08) = 0xa800u;            /* 0xd7df3c */
    ADC(0x04) = 0;
    udelay(20000);
}

#if YP3_ADC_PROBE
/* Round 3. The first two rounds put live numbers on the panel and asked the eye
 * to catch a change; that found up, down and right, and said nothing at all
 * about left and centre. Two things were wrong with it:
 *
 *   - it read channels 0-7, but the ADC has TEN (FIRM 0xd7df4c), so 8 and 9
 *     were never sampled;
 *   - a value that only moves while a key is down is easy to miss at a 150 ms
 *     refresh, and four raw GPIO words are hopeless to diff by eye.
 *
 * So this round RECORDS instead of displaying. Every channel keeps the span
 * between the smallest and largest value it has ever shown, and every GPIO port
 * keeps a sticky mask of the bits that have ever differed from their value at
 * boot. A key that touches anything at all therefore leaves a permanent mark,
 * and the mark says exactly which channel or which pin.
 *
 * Noise is 4-9 counts, so any span in the hundreds is a real key. Hold M (up)
 * to zero the accumulators - that is how you attribute a mark to one key:
 * hold M to clear, release, press only the key under test, read the screen.
 */
static void adc_probe_screen(void)
{
    uint32_t mn[ADC_NCHAN], mx[ADC_NCHAN];
    uint32_t base[4], chg[4];
    unsigned i, n = 0;

    for (i = 0; i < 4; i++) {
        base[i] = GPIO_IN(i);
        chg[i] = 0;
    }
    for (i = 0; i < ADC_NCHAN; i++) {
        mn[i] = 0xffffffffu;
        mx[i] = 0;
    }

    lcd_setfont(FONT_SYSFIXED);
    while (1) {
        uint32_t v[ADC_NCHAN];
        int line = 0;
        unsigned pass;
        int clear;

        /* Sample hard between refreshes - the accumulators are the point, so
         * spend the time here rather than in the redraw. */
        for (pass = 0; pass < 200; pass++) {
            for (i = 0; i < ADC_NCHAN; i++) {
                uint32_t x = ADC_DATA(i) & 0xfffu;
                v[i] = x;
                if (x < mn[i]) mn[i] = x;
                if (x > mx[i]) mx[i] = x;
            }
            for (i = 0; i < 4; i++)
                chg[i] |= GPIO_IN(i) ^ base[i];
        }

        /* M / up held: forget everything, so the next key tested stands alone. */
        clear = (v[ADC_CH] >= ADC_UP_MIN && v[ADC_CH] < ADC_UP_MAX);
        if (clear) {
            for (i = 0; i < ADC_NCHAN; i++) { mn[i] = 0xffffffffu; mx[i] = 0; }
            for (i = 0; i < 4; i++) { base[i] = GPIO_IN(i); chg[i] = 0; }
        }

        lcd_clear_display();
        lcd_putsf(0, line++, "hold M = clear %c", clear ? '*' : ' ');
        for (i = 0; i < ADC_NCHAN; i += 2)
            lcd_putsf(0, line++, "%u%4u%4u %u%4u%4u",
                      i, (unsigned)v[i], (unsigned)(mx[i] - mn[i]),
                      i + 1, (unsigned)v[i + 1], (unsigned)(mx[i + 1] - mn[i + 1]));
        line++;
        for (i = 0; i < 4; i++)
            lcd_putsf(0, line++, "p%u %08x", i, (unsigned)GPIO_IN(i));
        for (i = 0; i < 4; i++)
            lcd_putsf(0, line++, "d%u %08x", i, (unsigned)chg[i]);
        lcd_putsf(0, line++, "n %u", ++n);
        lcd_update();
    }
}
#endif

void button_init_device(void)
{
    adc_init_device();
    BC_SLOT(173) = ADC_DATA(ADC_CH);     /* resting value, for the dump */
#if YP3_ADC_PROBE
    adc_probe_screen();
#endif
}

/* MEASURED on hardware with the probe above, channel 1:
 *
 *     nothing   3741-3745        M / up      1975-1978
 *     VOL/down    55-  64        noise spread 4-9 counts
 *
 * The bands are hundreds of counts apart and the noise is single digits, so
 * plain midpoint thresholds are safe and no averaging is needed - Rockbox's own
 * button_read repetition handles the rest.
 *
 *     M / up        -> BUTTON_PREV   scroll up
 *     VOL / down    -> BUTTON_NEXT   scroll down
 *     forward/right -> BUTTON_PLAY   select
 *     back / left   -> BUTTON_MENU   cancel
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

int button_read_device(void)
{
    int btn = 0;
    unsigned v = ADC_DATA(ADC_CH) & 0xfffu;
    bool level = ((GPIO_IN(3) >> 12) & 1u) != 0;

    if (v < ADC_DOWN_MAX)
        btn |= BUTTON_NEXT;                      /* VOL / down, ~60 */
    else if (v >= ADC_UP_MIN && v < ADC_UP_MAX)
        btn |= BUTTON_PREV;                      /* M / up, ~1976 */

    /* Read what the bias applied last time has settled, then flip it. */
    if (bias_is_up) {
        right_held = !level;                     /* pulled down against the up bias */
        ROM_GPIO_CFG1(KEY_IO_PIN_DOWN);
    } else {
        left_held = level;                       /* pulled up against the down bias */
        ROM_GPIO_CFG1(KEY_IO_PIN_UP);
    }
    bias_is_up = !bias_is_up;

    if (right_held)
        btn |= BUTTON_PLAY;                      /* forward / right, id 0x43 */
    if (left_held)
        btn |= BUTTON_MENU;                      /* back / left, id 0x42 */

    return btn;
}

/* STILL UNMAPPED: centre (play/pause).
 *
 * It moves no ADC channel and does not touch 3.12 under either bias, and the
 * vendor opens exactly three key devices - /dev/kadc_ch1, /dev/key_io and
 * /dev/key_onoff (the only three name strings referenced from its key manager,
 * at FIRM 0xc4b7ea, 0xc4b84c and 0xc4b81a) - so key_onoff is what is left.
 *
 * That one is not a register poll. Its driver at FIRM 0xd66378 creates a
 * four-deep message queue named "konoff" and its read at 0xd663dc blocks on it
 * through 0xd7f45e -> ROM 0xb372, mapping queue events 0 and 2 onto the two key
 * ids its caller supplies at open. Nothing in FIRM ever sends to that queue, so
 * the sender is in ROM and the key arrives as an interrupt, not a level. That
 * is its own bring-up.
 *
 * The GPIO side is now exhausted as an explanation: in the stock capture
 * probe/gpio_work.bin, pin 3.12 is the ONLY pin that is mode 0 with a bias
 * configured. Every other mode-0 pin has both bias fields clear, i.e. nothing
 * is wired to it that stock cares to read.
 */
