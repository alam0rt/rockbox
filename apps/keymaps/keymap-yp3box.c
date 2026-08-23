/* Keymap for the anko yp3box (M, Previous, Play/Pause, Next, VOL) */
#include "config.h"
#include "action.h"
#include "button.h"
#include "settings.h"

/* Product controls, using the four inputs currently identified by the target
 * driver:
 *
 *     M / up               BUTTON_MENU
 *     Previous / left     BUTTON_PREV
 *     Play/Pause / centre BUTTON_PLAY (the PMU on/off key, vendor id 0x21)
 *     Next / right        BUTTON_NEXT
 *     VOL / down          BUTTON_VOL
 *
 * The old map used BUTTON_PLAY for right because the centre input had not been
 * found. Keep BUTTON_PLAY's play/select semantics and let the actual right key
 * be NEXT; the centre key now emits BUTTON_PLAY itself.
 */
static const struct button_mapping button_context_standard[] = {
    { ACTION_STD_PREV,       BUTTON_PREV,                BUTTON_NONE },
    { ACTION_STD_PREVREPEAT, BUTTON_PREV|BUTTON_REPEAT,  BUTTON_NONE },
    { ACTION_STD_NEXT,       BUTTON_NEXT,                BUTTON_NONE },
    { ACTION_STD_NEXTREPEAT, BUTTON_NEXT|BUTTON_REPEAT,  BUTTON_NONE },
    { ACTION_STD_OK,         BUTTON_PLAY|BUTTON_REL,     BUTTON_PLAY },
    { ACTION_STD_MENU,       BUTTON_MENU|BUTTON_REL,     BUTTON_MENU },
    { ACTION_STD_CONTEXT,    BUTTON_MENU|BUTTON_REPEAT,  BUTTON_MENU },
    { ACTION_STD_CANCEL,     BUTTON_VOL|BUTTON_REL,      BUTTON_VOL },
    LAST_ITEM_IN_LIST
};

static const struct button_mapping button_context_wps[] = {
    { ACTION_WPS_PLAY,       BUTTON_PLAY|BUTTON_REL,     BUTTON_PLAY },
    { ACTION_WPS_SKIPNEXT,   BUTTON_NEXT|BUTTON_REL,     BUTTON_NEXT },
    { ACTION_WPS_SKIPPREV,   BUTTON_PREV|BUTTON_REL,     BUTTON_PREV },
    { ACTION_WPS_MENU,       BUTTON_MENU|BUTTON_REL,     BUTTON_MENU },
    { ACTION_WPS_CONTEXT,    BUTTON_MENU|BUTTON_REPEAT,  BUTTON_MENU },
    { ACTION_WPS_BROWSE,     BUTTON_VOL|BUTTON_REL,      BUTTON_VOL },
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
