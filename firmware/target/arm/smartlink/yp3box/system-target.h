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

#endif
