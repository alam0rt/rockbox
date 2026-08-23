#ifndef __BACKLIGHT_TARGET_H__
#define __BACKLIGHT_TARGET_H__
#include <stdbool.h>
bool backlight_hw_init(void);
void backlight_hw_on(void);
void backlight_hw_off(void);
#endif
