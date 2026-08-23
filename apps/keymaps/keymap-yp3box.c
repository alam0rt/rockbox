/* Keymap for the anko yp3box (M, Previous, Play/Pause, Next, VOL)
 *
 * Copyright (C) 2026 the Rockbox project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include "config.h"
#include "action.h"
#include "button.h"
#include "settings.h"

/* Five keys, and the product's own labels for them:
 *
 *     M / up               BUTTON_MENU     ADC ladder, ~1976 counts
 *     VOL / down           BUTTON_VOL      ADC ladder, ~60 counts
 *     Previous / left      BUTTON_PREV     GPIO 3.12, vendor key id 0x42
 *     Next / right         BUTTON_NEXT     GPIO 3.12, vendor key id 0x43
 *     Play/Pause / centre  BUTTON_PLAY     PMU on/off key, vendor id 0x21
 *
 * M and VOL are the vertical pair, so they are up and down: in a list they
 * move the selection, in the WPS they are volume. Previous and Next are the
 * horizontal pair - back and forward in a list, skip in the WPS - and the
 * centre key selects and plays.
 *
 * Nothing is bound to a long press of the centre key. That key is also the
 * hardware power key (POWEROFF_BUTTON in button-target.h, and the PMU turns a
 * long press into the vendor's power event), so a long press there has one
 * meaning and should keep it. The two long presses that are needed instead sit
 * on the horizontal pair: long back opens the menu, long forward opens the
 * context menu.
 */
static const struct button_mapping button_context_standard[] = {
    { ACTION_STD_PREV,       BUTTON_MENU,                BUTTON_NONE },
    { ACTION_STD_PREVREPEAT, BUTTON_MENU|BUTTON_REPEAT,  BUTTON_NONE },
    { ACTION_STD_NEXT,       BUTTON_VOL,                 BUTTON_NONE },
    { ACTION_STD_NEXTREPEAT, BUTTON_VOL|BUTTON_REPEAT,   BUTTON_NONE },
    { ACTION_STD_OK,         BUTTON_PLAY|BUTTON_REL,     BUTTON_PLAY },
    { ACTION_STD_OK,         BUTTON_NEXT|BUTTON_REL,     BUTTON_NEXT },
    { ACTION_STD_CANCEL,     BUTTON_PREV|BUTTON_REL,     BUTTON_PREV },
    { ACTION_STD_MENU,       BUTTON_PREV|BUTTON_REPEAT,  BUTTON_PREV },
    { ACTION_STD_CONTEXT,    BUTTON_NEXT|BUTTON_REPEAT,  BUTTON_NEXT },
    LAST_ITEM_IN_LIST
};

/* Settings screens ask for INC/DEC rather than PREV/NEXT, so the vertical pair
 * has to be named again here or sliders do not move. Everything else falls
 * through to the standard list. */
static const struct button_mapping button_context_settings[] = {
    { ACTION_SETTINGS_INC,       BUTTON_MENU,                BUTTON_NONE },
    { ACTION_SETTINGS_INCREPEAT, BUTTON_MENU|BUTTON_REPEAT,  BUTTON_NONE },
    { ACTION_SETTINGS_DEC,       BUTTON_VOL,                 BUTTON_NONE },
    { ACTION_SETTINGS_DECREPEAT, BUTTON_VOL|BUTTON_REPEAT,   BUTTON_NONE },
    { ACTION_STD_CANCEL,         BUTTON_PREV|BUTTON_REL,     BUTTON_PREV },
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD)
};

static const struct button_mapping button_context_wps[] = {
    { ACTION_WPS_PLAY,       BUTTON_PLAY|BUTTON_REL,     BUTTON_PLAY },
    { ACTION_WPS_SKIPNEXT,   BUTTON_NEXT|BUTTON_REL,     BUTTON_NEXT },
    { ACTION_WPS_SKIPPREV,   BUTTON_PREV|BUTTON_REL,     BUTTON_PREV },
    { ACTION_WPS_VOLUP,      BUTTON_MENU,                BUTTON_NONE },
    { ACTION_WPS_VOLUP,      BUTTON_MENU|BUTTON_REPEAT,  BUTTON_NONE },
    { ACTION_WPS_VOLDOWN,    BUTTON_VOL,                 BUTTON_NONE },
    { ACTION_WPS_VOLDOWN,    BUTTON_VOL|BUTTON_REPEAT,   BUTTON_NONE },
    { ACTION_WPS_BROWSE,     BUTTON_PREV|BUTTON_REPEAT,  BUTTON_PREV },
    { ACTION_WPS_CONTEXT,    BUTTON_NEXT|BUTTON_REPEAT,  BUTTON_NEXT },
    LAST_ITEM_IN_LIST
};

const struct button_mapping *get_context_mapping(int context)
{
    switch (context & ~CONTEXT_REMOTE) {
    case CONTEXT_WPS:
        return button_context_wps;
    case CONTEXT_SETTINGS:
    case CONTEXT_SETTINGS_EQ:
    case CONTEXT_SETTINGS_TIME:
    case CONTEXT_SETTINGS_COLOURCHOOSER:
        return button_context_settings;
    default:
        return button_context_standard;
    }
}
