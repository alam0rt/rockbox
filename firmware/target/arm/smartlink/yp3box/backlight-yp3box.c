/* Backlight = PWM channel 3, register base 0x40084080.
 *
 * Reversed from the vendor driver rather than probed:
 *   /dev/pwm_ch3 struct 0x0082bdb0 -> 0x40084080 in the handle table
 *   0xc483fc(base,on)   [base+0x00] bit4 = enable
 *   0xc4843c(base,on)   [base+0x10] bit0 = run
 *   0xc48430(base,h,l)  [base+0x14] = l | (h<<16)   period
 *   0xc48438(base,v)    [base+0x18] = v             duty
 * pwm_off (0xd67b88) clears both bits, in that order.
 * The explicit channel-3 backlight path uses module 0x44 and clock 0x27;
 * the separate generic helper IDs 0x49/0x2b must not be substituted here.
 */
#include "config.h"
#include "backlight-target.h"
#include <stdint.h>

#define PWM3    0x40084080u
#define PWMR(o) (*(volatile uint32_t *)(PWM3 + (o)))

/* From the vendor timer/PWM init at FIRM 0xd7e314:
 *     if (!clk_is_on(0x44)) clk_enable(0x44);     module
 *     clk_set_source(0x27, 16); clk_apply(0x27);  clock
 *     base = 0x40084020 + ch*32                   ch3 -> 0x40084080
 * Without the MODULE enable the PWM registers do not hold a value - exactly the
 * symptom the LCDC had before clk_enable(0x5c). */
/* Bindings come from sl6801-regs.h rather than a local copy: it carries
 * ROM_CLK_SET_SRC, and a safety wrapper that exists in three hand-copies is
 * a wrapper waiting to be forgotten in one of them. */
#include "sl6801-regs.h"

/* THE PWM OUTPUT PIN.
 *
 * The driver registration for /dev/pwm_ch3 (FIRM 0xd67b38) stores a pin code in
 * dev[0x48]:  dev[0x48] = 0x00018a80
 * and the open path (FIRM 0xd67bd0) passes it to the SRAM thunk 0x8051f4, which
 * tail-calls ROM 0x7ac = gpio_config(code).
 *
 * The code packs pin AND alternate function together (ROM 0x7ac decodes
 * port=[19:16], pin=[15:11], func=[10:7], plus mode bits [6] and [5:4]):
 *     0x00018a80 -> port 1, pin 17, function 5
 *
 * Without this the PWM peripheral runs but its output never reaches the pad,
 * which is exactly what we saw: mode/period/duty all programmed correctly and
 * the panel completely unlit. Note pin 17 is NOT among the twelve pins the LCD
 * path configures, so nothing else was covering it. */
#define ROM_GPIO_CFG1 ((void (*)(unsigned))0x7adu)   /* thumb */
#define PWM3_OUT_PIN  0x00018a80u

/* The backlight pin is SHARED, and the simple path does not involve PWM at all.
 *
 * The /dev/lcd ioctl (FIRM 0xd66500) configures the very same pin - port 1
 * pin 17 - as a plain GPIO, switching only bit 6:
 *     cmd 0 -> gpio_config(0x000188c0)   func 1, bit6 = 1
 *     cmd 1 -> gpio_config(0x00018880)   func 1, bit6 = 0
 * while backlight_set (FIRM 0xd42574) re-muxes the same pin to function 5
 * (0x00018a80) to drive it from PWM channel 3 for brightness control.
 *
 * So bit 6 is the output level and function 1 is GPIO. For first light we only
 * need the GPIO form; PWM is a brightness refinement. The two are mutually
 * exclusive - whichever configures the pin last owns it. */
#define BL_PIN_ON   0x000188c0u
#define BL_PIN_OFF  0x00018880u
#define PWM_MODULE    0x44u
#define PWM_CLOCK     0x27u

/* From the vendor's backlight_set at FIRM 0xd42574:
 *     cfg.mode = 3; cfg.period = 48000 (0xbb80); cfg.duty = pct * 480
 * so full brightness is period == duty == 48000. */
#define PWM_PERIOD 48000u

/* Mode word for +0x00, from the vendor open path:
 *   0xd7e394: [base+0x00] = 0x40; [base+0x00] |= cfgval
 *   0xd7e314 builds cfgval from the backlight cfg bytes 03 00 01 0f:
 *     (1<<4)&0x10 | (0<<7)&0xff | (0x0f&0x0f) | (1<<5)&0x20 == 0x3f
 * so the mode register ends up 0x7f. Setting only bit4 (0x10), as we did,
 * leaves the PWM unconfigured and it never drives the pin. */
#define PWM_MODE 0x7fu

void backlight_hw_on(void)
{
    /* PWM first, then claim the pad as a GPIO driven high. If the PWM route
     * works the pad mux below would override it, so drive the simple path last:
     * it is the one the LCD driver itself uses. */
    PWMR(0x00) = 0x40u;
    PWMR(0x00) |= (PWM_MODE & ~0x40u);
    PWMR(0x10) |= 0x01u;        /* run */
    ROM_GPIO_CFG1(BL_PIN_ON);
}

void backlight_hw_off(void)
{
    /* Mirrors the vendor's pwm_off (FIRM 0xd67b88): run bit first, then enable.
     *   0xc4843c: [base+0x10] bit0 = run
     *   0xc483fc: [base+0x00] bit4 = enable
     * Then release the pad low via the same GPIO form /dev/lcd uses. */
    PWMR(0x10) &= ~0x01u;
    PWMR(0x00) &= ~0x10u;
    ROM_GPIO_CFG1(BL_PIN_OFF);
}

bool backlight_hw_init(void)
{
    volatile uint32_t d;


    ROM_GPIO_CFG1(PWM3_OUT_PIN);        /* mux the PWM output pad */

    if (!ROM_CLK_IS_ON(PWM_MODULE))
        ROM_CLK_ENABLE(PWM_MODULE);
    /* Through the wrapper. Source 16 needs no pre-start, so this is exactly
     * what the raw call did - but the raw form is the one that has frozen this
     * device twice when the source happened to be 8 or 9, so it does not stay
     * in the tree. tools/check_clock_rules.py enforces that. */
    ROM_CLK_SET_SRC(PWM_CLOCK, 16);
    ROM_CLK_APPLY(PWM_CLOCK);
    for (d = 0; d < 5000; d++) ;

    PWMR(0x14) = PWM_PERIOD;            /* period, per vendor */
    PWMR(0x18) = PWM_PERIOD;            /* duty = period -> 100% */


    backlight_hw_on();

    return true;
}
