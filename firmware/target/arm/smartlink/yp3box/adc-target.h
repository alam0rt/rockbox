#ifndef __ADC_TARGET_H__
#define __ADC_TARGET_H__
/* SL6801 has /dev/kadc_ch0..9; ch1 is the battery channel. TODO: implement. */
#define NUM_ADC_CHANNELS 1
#define ADC_BATTERY      0
#define ADC_UNREG_POWER  ADC_BATTERY
#endif
