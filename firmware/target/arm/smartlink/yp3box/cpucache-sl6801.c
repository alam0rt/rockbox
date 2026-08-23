/* Cortex-M4 has no instruction or data cache (unlike the M7), so cache
 * maintenance is a no-op. Provided in place of cpucache-armv7m.c, which
 * #includes regs/cortex-m/cm_cache.h - a header not present in the tree. */
#include "cpucache-armv7m.h"

void __discard_idcache(void)
{
    arm_dsb();
    arm_isb();
}
