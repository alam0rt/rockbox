/* Compatibility entry points for optional hardware facilities not implemented
 * by this target. */
#include "config.h"
#include "system.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* --- cache: correct as-is, Cortex-M4 has no cache --- */
void commit_dcache(void) { }
void commit_discard_dcache(void) { }
void commit_discard_idcache(void) { }

/* timer/tick now in timer-yp3box.c */

/* --- adc / i2c --- */
void adc_init(void) { }
void i2c_init(void) { }

/* --- debug menu --- */
bool dbg_hw_info(void) { return false; }
bool dbg_ports(void) { return false; }


/* SD now lives in sd-yp3box.c */
