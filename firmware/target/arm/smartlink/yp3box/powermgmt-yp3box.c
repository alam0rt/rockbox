/*
 * Battery voltage from the PMU's calibrated sense path.
 *
 * FIRM 0xcf7a28 selects the battery sense input through PMU register 0x47,
 * reads register 0x2e, then clears that selection. Its integer conversion is:
 *
 *     (14173 * (sample & 0x7f) + 0x2a8000 + 0x3980) / 1000
 *
 * The state cache used by the stock firmware is in SRAM that Rockbox reuses, so
 * reproducing this read is required instead of dereferencing 0x00823dde.
 */
#include "config.h"
#include "powermgmt.h"
#include "sl6801-regs.h"

int _battery_voltage(void)
{
    uint8_t config;
    uint8_t sample;
    uint32_t voltage;

    if (!yp3_pmu_read(PMU_REG_VOLTAGE_CONFIG, &config))
        return -1;
    if (!yp3_pmu_write(PMU_REG_VOLTAGE_CONFIG, config | 0x10u)
            || !yp3_pmu_read(PMU_REG_BATTERY_VOLTAGE, &sample)
            || !yp3_pmu_read(PMU_REG_VOLTAGE_CONFIG, &config)) {
        (void)yp3_pmu_write(PMU_REG_VOLTAGE_CONFIG, config & ~0x10u);
        return -1;
    }
    if (!yp3_pmu_write(PMU_REG_VOLTAGE_CONFIG, config & ~0x10u))
        return -1;

    voltage = 14173u * (sample & 0x7fu) + 0x2a8000u + 0x3980u;
    return (int)(voltage / 1000u);
}

/* Li-ion profile used by the existing Rockbox 3.7 V targets. Until a YP3
 * discharge/charge sweep is correlated with a DMM, these thresholds are a
 * conservative UI estimate, not a battery gauge calibration. */
unsigned short battery_level_disksafe = 3400;
unsigned short battery_level_shutoff = 3300;

unsigned short percent_to_volt_discharge[11] =
{
    3300, 3653, 3701, 3735, 3768, 3790, 3833, 3900, 3966, 4056, 4140
};

#if CONFIG_CHARGING
unsigned short percent_to_volt_charge[11] =
{
    3333, 3757, 3815, 3845, 3867, 3900, 3950, 4008, 4078, 4166, 4245
};
#endif
