#ifndef __ADC_TARGET_H__
#define __ADC_TARGET_H__
/* The battery monitor is PMU-backed; ADC channel 1 is the button ladder.
 * Keep the generic compatibility names, but battery code must use
 * _battery_voltage() rather than adc_read(). */
#define NUM_ADC_CHANNELS 1
#define ADC_BATTERY      0
#define ADC_UNREG_POWER  ADC_BATTERY
#endif
