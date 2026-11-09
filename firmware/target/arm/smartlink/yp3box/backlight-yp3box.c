/* Backlight: pad P1.17, driven either as a plain GPIO or by PWM channel 3.
 *
 * YP3_BL_PWM (default 0) picks the path. 0 is the GPIO on/off path that has
 * lit this panel on every boot [L]. 1 is the vendor's PWM path, transcribed
 * below and NOT YET RUN ON HARDWARE - see the readback breadcrumbs, which are
 * there so the first boot answers whether it took.
 *
 * ---- What the vendor does, all [C], every address checked with objdump -----
 *
 * Open, once: backlight_open FIRM 0xd4252c -> open("/dev/pwm_ch3", 0, cfg)
 * with cfg bytes 03 00 01 0f 00 (literal 0xd4255c, then strb 0 at 0xd42540).
 * The PWM driver's open op is FIRM 0xd67bd0, which for channel cfg[0]=3:
 *   0xd67c40  gpio_config(dev[0x48] = 0x00018a80)  via SRAM 0x8051f4 -> ROM 0x7ac
 *             = port 1, pin 17, function 5 (ROM 0x77a/0x568 decode: port
 *             [19:16], pin [15:11], func [10:7]; P1 mode words at 0x40081040)
 *   0xd67c4c  0xd7e314(ch=3, {1, cfg[1], cfg[2], cfg[3]}, cfg[4]=0):
 *     0xd7e31e  if (!clk_is_on(0x44)) clk_enable(0x44)     ROM 0x2518 / 0x2564
 *     0xd7e32a  src = cfg[4] ? 16 : 10   -> cfg[4] is 0, so SOURCE 10
 *     0xd7e330  clk_set_source(0x27, src) via the SRAM wrapper 0x8050f2
 *     0xd7e336  clk start(0x27)           via SRAM 0x8051ec -> ROM 0x27a0
 *     0xd7e340  base = 0x40084020 + ch*32  -> 0x40084080, into the handle
 *               table at 0x81e59c[ch+4] (sram_live.bin holds exactly that:
 *               [0]=0x40084000, [2]=0x40084014, [7]=0x40084080)
 *     0xd7e35a  mode = (b0<<4 & 0x10) | (b1<<7 & 0x80) | (b3 & 0xf) | (b2<<5 & 0x20)
 *               = 0x10 | 0 | 0x0f | 0x20 = 0x3f
 *     0xd7e394  [base+0x00] = 0x40; [base+0x00] |= 0x3f      -> 0x7f
 *   0xd67c54  0xc483bc(ch, &stack): writes +0x14, +0x18, +0x10, +0x1c from a
 *             struct the caller never initialises EXCEPT byte 13 (strb 0 at
 *             0xd67c18), which 0xc4844e puts in +0x10 bit 0. So the timer half
 *             of the channel is configured with garbage and explicitly NOT run.
 *             The PWM output does not need +0x10 bit 0.
 *
 * Set level: backlight_set(pct) FIRM 0xd42574:
 *     pct = min(pct, 100); ioctl(h, 0, {3, -, 48000, pct * 480})
 *   ioctl op FIRM 0xd67c80, cmd 0 -> 0xc482b8(ch, period, duty):
 *     if (period < duty) return;
 *     0xc48428  while ([base+0x00] & (1 << 28)) ;       (lsls #3 -> bit 28)
 *     0xc4840e  [base+0x04] = duty | (period << 16)
 *   THE DUTY REGISTER IS +0x04. +0x14/+0x18 belong to the timer half above;
 *   the previous version of this file wrote 48000 there and nothing to +0x04.
 *
 * On/off: FIRM 0xcfe518: arg 0 -> backlight_set(get_brightness()),
 *   arg 1 -> backlight_set(0). The vendor turns the light off with duty 0 and
 *   leaves the pad on the PWM.
 *
 * Levels: set_brightness FIRM 0xd106a0 accepts 10..100 only (sub #10, cmp #90)
 *   and stores the byte at SRAM 0x824258; the UI at 0xd0ba02 builds it as
 *   (level + 1) * 10 for level 0..9. sram_live.bin has 0x3c = 60% there.
 *   So Rockbox setting n in 1..10 is n*10 percent, duty n * 4800.
 *
 * Off (0xd67b88): +0x10 bit 0 clear (0xc4843c), then +0x00 bit 4 clear
 *   (0xc483fc). Not used here - off is duty 0, as the vendor does it.
 *
 * ---- The GPIO path ----
 * /dev/lcd ioctl FIRM 0xd66500: cmd 0 -> gpio_config(0x000188c0) (func 1,
 * bit 6 = 1, on), cmd 1 -> gpio_config(0x00018880) (off). Whichever of the
 * two configures the pad last owns it.
 */
#include "config.h"
#include "system.h"
#include "backlight.h"
#include "backlight-target.h"
#include "sl6801-regs.h"
#include "yp3-buildstamp.h"
#include <stdint.h>
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

#ifndef YP3_BL_PWM
#define YP3_BL_PWM 0            /* untested on hardware; 1 = PWM brightness */
#endif

#define BL_PIN_ON     0x000188c0u   /* P1.17 func 1 (GPIO), bit 6 high */
#define BL_PIN_OFF    0x00018880u   /* P1.17 func 1 (GPIO), bit 6 low  */

static int bl_level = DEFAULT_BRIGHTNESS_SETTING;

#if YP3_BL_PWM

#define PWM3          0x40084080u
#define PWMR(o)       (*(volatile uint32_t *)(PWM3 + (o)))
#define PWM3_OUT_PIN  0x00018a80u   /* P1.17 func 5, dev[0x48] at 0xd67b4e */
#define PWM_MODULE    0x44u
#define PWM_CLOCK     0x27u
#define PWM_CLK_SRC   10u           /* 0xd7e32a: cfg[4] == 0 -> 10, 24 MHz ref [I] */
#define PWM_MODE      0x3fu         /* 0xd7e35a with cfg 01 00 01 0f */
#define PWM_PERIOD    48000u        /* 0xd42584 */
#define PWM_BUSY      (1u << 28)    /* 0xc48428 */
#define PWM_SPIN      100000u

/* P1 mode word for pins 16..23, 4 bits per pin (ROM 0x568); pin 17 -> [7:4].
 * 5 = PWM, 1 = GPIO. Read-only, for the breadcrumb. */
#define P1_MODE_16_23 (*(volatile uint32_t *)0x40081048u)
#define P1_IN         (*(volatile uint32_t *)0x40081050u)

static bool bl_pwm_ok;
static bool bl_on;
static unsigned bl_busy_timeouts;

/* Module enable and clock from SRAM: clk_enable spins on an acknowledge bit
 * and a clock change can perturb the controller we fetch from. Same order as
 * 0xd7e314. */
static void __attribute__((section(".icode"), noinline)) bl_pwm_bringup(void)
{
    volatile uint32_t d;

    if (!ROM_CLK_IS_ON(PWM_MODULE))
        ROM_CLK_ENABLE(PWM_MODULE);
    ROM_CLK_SET_SRC(PWM_CLOCK, PWM_CLK_SRC);
    ROM_CLK_APPLY(PWM_CLOCK);           /* ROM 0x27a0 = start, as 0x8051ec */
    for (d = 0; d < 5000; d++) ;

    PWMR(0x00) = 0x40u;                 /* 0xd7e394 */
    PWMR(0x00) |= PWM_MODE;             /* 0xd7e398..0xd7e39c -> 0x7f */
    /* The vendor's +0x10 write is uninitialised stack with bit 0 forced to 0
     * (0xd67c18). Only that bit is defined, so only that bit is written. */
    PWMR(0x10) &= ~1u;
}

/* 0xc482b8: wait for the update bit, then period:duty in one word. */
static bool bl_pwm_set_duty(unsigned duty)
{
    unsigned spins = 0;

    if (duty > PWM_PERIOD)
        duty = PWM_PERIOD;
    while ((PWMR(0x00) & PWM_BUSY) && ++spins < PWM_SPIN)
        ;
    if (spins >= PWM_SPIN) {
        bl_busy_timeouts++;
        return false;
    }
    PWMR(0x04) = duty | (PWM_PERIOD << 16);
    return true;
}

static unsigned bl_duty(int level)
{
    if (level < MIN_BRIGHTNESS_SETTING) level = MIN_BRIGHTNESS_SETTING;
    if (level > MAX_BRIGHTNESS_SETTING) level = MAX_BRIGHTNESS_SETTING;
    return (unsigned)level * 10u * 480u;    /* pct * 480, 0xd4258c */
}

static void bl_log(const char *what)
{
    logf("bl: %s ok=%d on=%d lvl=%d mod=%d r00=%08lx r04=%08lx r10=%08lx",
         what, bl_pwm_ok, bl_on, bl_level, ROM_CLK_IS_ON(PWM_MODULE),
         (unsigned long)PWMR(0x00), (unsigned long)PWMR(0x04),
         (unsigned long)PWMR(0x10));
    logf("bl: c00=%08lx c14=%08lx p1m=%lx pad=%d busy_to=%u",
         (unsigned long)*(volatile uint32_t *)0x40084000u,
         (unsigned long)*(volatile uint32_t *)0x40084014u,
         (unsigned long)((P1_MODE_16_23 >> 4) & 0xf),
         (int)((P1_IN >> 17) & 1), bl_busy_timeouts);
}

void backlight_hw_on(void)
{
    bl_on = true;
    if (bl_pwm_ok && bl_pwm_set_duty(bl_duty(bl_level))) {
        ROM_GPIO_CFG1(PWM3_OUT_PIN);    /* duty first, then hand it the pad */
        return;
    }
    ROM_GPIO_CFG1(BL_PIN_ON);
}

void backlight_hw_off(void)
{
    bl_on = false;
    /* Pad to GPIO low first: that off is known to work [L] whether or not the
     * PWM does. Then duty 0, which is the vendor's own off (0xcfe532). */
    ROM_GPIO_CFG1(BL_PIN_OFF);
    if (bl_pwm_ok)
        bl_pwm_set_duty(0);
}

void backlight_hw_brightness(int level)
{
    bl_level = level;
    if (bl_pwm_ok && bl_on) {
        bl_pwm_set_duty(bl_duty(level));
        logf("bl: level %d duty %u r04=%08lx", level, bl_duty(level),
             (unsigned long)PWMR(0x04));
    }
}

bool backlight_hw_init(void)
{
    unsigned want = bl_duty(bl_level) | (PWM_PERIOD << 16);

    /* The pad stays a GPIO driven high while the PWM is brought up, so the
     * panel is never dark for the duration. The vendor muxes first
     * (0xd67c40 before 0xd67c4c); doing it last is a departure [I]. */
    ROM_GPIO_CFG1(BL_PIN_ON);
    bl_pwm_bringup();
    bl_pwm_ok = true;
    if (!bl_pwm_set_duty(bl_duty(bl_level)) || PWMR(0x04) != want
        || !(PWMR(0x00) & 0x10u))
        bl_pwm_ok = false;  /* register file did not take: stay on GPIO */
    bl_log("init");

    backlight_hw_on();
    bl_log("on");
    return true;
}

#else /* !YP3_BL_PWM: GPIO only, full brightness */

void backlight_hw_on(void)
{
    ROM_GPIO_CFG1(BL_PIN_ON);
}

void backlight_hw_off(void)
{
    ROM_GPIO_CFG1(BL_PIN_OFF);
}

void backlight_hw_brightness(int level)
{
    /* No PWM in this build: the setting is remembered and has no effect. */
    bl_level = level;
}

bool backlight_hw_init(void)
{
    logf("bl: gpio only (YP3_BL_PWM=0)");
    backlight_hw_on();
    return true;
}

#endif /* YP3_BL_PWM */
