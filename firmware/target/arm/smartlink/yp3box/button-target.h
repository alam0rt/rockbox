#ifndef __BUTTON_TARGET_H__
#define __BUTTON_TARGET_H__
#define BUTTON_NONE     0x00000000
#define BUTTON_PLAY     0x00000001
#define BUTTON_PREV     0x00000002
#define BUTTON_NEXT     0x00000004
#define BUTTON_MENU     0x00000008
#define BUTTON_VOL      0x00000010
#define BUTTON_MAIN \
    (BUTTON_PLAY | BUTTON_PREV | BUTTON_NEXT | BUTTON_MENU | BUTTON_VOL)
/* Must be a button no key produces: firmware/drivers/button.c shuts the player
 * down on the first POWEROFF_BUTTON repeat, so pointing this at a key that is
 * held for any normal purpose powers the device off mid-use. Four of the five
 * keys are now mapped (see button-yp3box.c) and BUTTON_VOL is the one left
 * over, so it takes the role until the vendor's on/off key is brought up - and
 * that key is the natural power button anyway. */
#define POWEROFF_BUTTON BUTTON_VOL
#define POWEROFF_COUNT  10
#endif
