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

/* Previous and Next ARE the left and right keys - one switch each side of the
 * centre, and the driver reads them as one pin biased two ways (vendor key ids
 * 0x42 "left" and 0x43 "right"). Saying so gives the core the directional pair
 * it looks for: apps/action.c mirrors left/right for right-to-left languages
 * and warns when a pad declares neither the pair nor NO_BUTTON_LR. Aliases,
 * not new bits, so BUTTON_MAIN and the keymap are unchanged. */
#define BUTTON_LEFT     BUTTON_PREV
#define BUTTON_RIGHT    BUTTON_NEXT
/* The product's Play/Pause key is also its hardware power key. Keep the
 * software-poweroff fallback on that logical button, not on VOL: VOL is a real
 * control and must remain safe to hold while adjusting volume. It also matches
 * the hardware, which turns a long press of that same key into the vendor's
 * power key. */
#define POWEROFF_BUTTON BUTTON_PLAY
#define POWEROFF_COUNT  10
#endif
