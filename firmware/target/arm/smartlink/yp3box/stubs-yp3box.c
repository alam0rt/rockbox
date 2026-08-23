/* Stubs so the target links. NONE of this works - the point is to get a complete
 * binary and measure its real size. Every function here is a TODO. */
#include "config.h"
#include "system.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "breadcrumb.h"

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

/* --- battery curves --- */
unsigned short battery_level_disksafe = 3400;
unsigned short battery_level_shutoff  = 3300;
unsigned short percent_to_volt_charge[11] =
    { 3300,3500,3600,3660,3700,3740,3800,3880,3950,4050,4200 };
unsigned short percent_to_volt_discharge[11] =
    { 3300,3500,3600,3660,3700,3740,3800,3880,3950,4050,4200 };

/* SD now lives in sd-yp3box.c */
