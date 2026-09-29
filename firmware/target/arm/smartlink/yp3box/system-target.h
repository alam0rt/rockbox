#ifndef __SYSTEM_TARGET_H__
#define __SYSTEM_TARGET_H__
#include "system-arm.h"
#include "cpucache-armv7m.h"
#include <stdint.h>

#define CPUFREQ_DEFAULT     192000000
#define CPUFREQ_NORMAL      192000000
#define CPUFREQ_MAX         192000000

void udelay(uint32_t us);
/* Mask every device interrupt in the NVIC. SysTick is a core exception and is
 * unaffected, so the kernel tick survives. Defined in timer-yp3box.c; declared
 * here because the power-down path needs it as much as system_init does. */
void yp3_mask_all_irqs(void);
static inline void mdelay(unsigned int ms) { udelay(ms * 1000); }

/* The rig - rig-yp3box.c. Getting the device through a flash/boot/observe
 * cycle with no buttons pressed. yp3_rig_tick() runs from systick_handler,
 * yp3_rig_init() from the end of system_init, and the reboots never return. */
struct yp3_bootctl {
    uint32_t magic;
    uint32_t request;
    uint32_t warm_flag;
    uint32_t warm_vec;
    uint32_t resets;
    uint32_t boots;
    uint32_t rom_wdog_load;
    uint32_t spare;
};
extern struct yp3_bootctl yp3_bootctl;   /* placed by app.lds, head of .persist */

void yp3_rig_init(void);
int  yp3_rig_buttons(void);   /* ORed into button_read_device() */
void yp3_rig_tick(void);
void yp3_watchdog_arm(uint32_t load);
void yp3_watchdog_feed(void);
void yp3_watchdog_disable(void);
void yp3_reboot(void) __attribute__((noreturn));
void yp3_reboot_to_bootloader(void) __attribute__((noreturn));

#endif
