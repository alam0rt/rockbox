/* Keymap for the anko yp3box (5 buttons: M, Prev, Next, Play, VOL) */
#include "config.h"
#include "action.h"
#include "button.h"
#include "settings.h"

/* FOUR physical keys of the five, all measured:
 *
 *     M / up        BUTTON_PREV   ADC ladder, channel 1, ~1976
 *     VOL / down    BUTTON_NEXT   ADC ladder, channel 1, ~60
 *     forward/right BUTTON_PLAY   pin 3.12 biased high, reads low
 *     back / left   BUTTON_MENU   pin 3.12 biased low, reads high
 *
 * That is enough for a conventional mapping and the provisional one this
 * replaces - where right did double duty and a long press on DOWN was the only
 * way out of a menu - can go.
 *
 * Centre (play/pause) is still missing; it is behind the vendor's
 * /dev/key_onoff, which delivers through a message queue rather than a level.
 * Until it arrives, right's short press is select and its long press is the
 * menu, which is the usual arrangement for a four-key pad.
 */
static const struct button_mapping button_context_standard[] = {
    { ACTION_STD_PREV,       BUTTON_PREV,                BUTTON_NONE },
    { ACTION_STD_PREVREPEAT, BUTTON_PREV|BUTTON_REPEAT,  BUTTON_NONE },
    { ACTION_STD_NEXT,       BUTTON_NEXT,                BUTTON_NONE },
    { ACTION_STD_NEXTREPEAT, BUTTON_NEXT|BUTTON_REPEAT,  BUTTON_NONE },
    { ACTION_STD_OK,         BUTTON_PLAY|BUTTON_REL,     BUTTON_PLAY },
    { ACTION_STD_MENU,       BUTTON_PLAY|BUTTON_REPEAT,  BUTTON_PLAY },
    { ACTION_STD_CANCEL,     BUTTON_MENU|BUTTON_REL,     BUTTON_MENU },
    { ACTION_STD_CONTEXT,    BUTTON_MENU|BUTTON_REPEAT,  BUTTON_MENU },
    LAST_ITEM_IN_LIST
};

static const struct button_mapping button_context_wps[] = {
    { ACTION_WPS_PLAY,     BUTTON_PLAY|BUTTON_REL,     BUTTON_PLAY },
    { ACTION_WPS_SKIPNEXT, BUTTON_NEXT|BUTTON_REL,     BUTTON_NEXT },
    { ACTION_WPS_SKIPPREV, BUTTON_PREV|BUTTON_REL,     BUTTON_PREV },
    { ACTION_WPS_BROWSE,   BUTTON_MENU|BUTTON_REL,     BUTTON_MENU },
    { ACTION_WPS_CONTEXT,  BUTTON_PLAY|BUTTON_REPEAT,  BUTTON_PLAY },
    LAST_ITEM_IN_LIST
};

const struct button_mapping *get_context_mapping(int context)
{
    switch (context & ~CONTEXT_REMOTE) {
    case CONTEXT_WPS:
        return button_context_wps;
    default:
        return button_context_standard;
    }
}
