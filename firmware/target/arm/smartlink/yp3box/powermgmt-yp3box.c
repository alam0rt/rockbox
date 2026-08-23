/* Battery voltage. The stock firmware keeps a state struct at 0x00823dde with
 * percent at +3 and (probably) millivolts at +8, but those are ITS addresses -
 * a Rockbox build must read the ADC (kadc_ch1) itself.  TODO. */
#include "config.h"
#include "powermgmt.h"

int _battery_voltage(void)
{
    return 4000;   /* placeholder until the ADC driver exists */
}
