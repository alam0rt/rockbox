#             __________               __   ___.
#   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
#   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
#   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
#   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
#                     \/            \/     \/    \/            \/
#
# Doom, linked into the core instead of loaded as a plugin.
#
# On an XIP target with 64 KB of plugin buffer and 1.6 MB of spare flash, the
# 228 KB of Doom code cannot be loaded but can be executed in place. Building it
# into the image puts .text and .rodata in flash and leaves only .data and .bss
# in SRAM. See docs/DOOM.md in the yp3box port repo.
#
# The sources are the plugin's, unmodified; only the glue and the flags differ.

DOOMCORELIB := $(BUILDDIR)/lib/libdoomcore.a
DOOMCORE_DIR := $(APPSDIR)/plugins/doom
# helper.c is pluginlib, not the engine: rockdoom.c calls its
# backlight_ignore_timeout()/backlight_use_settings(). Pulling the one file in
# beats reimplementing it, and libplugin.a as a whole is plugin-flagged code we
# do not want in the core image.
DOOMCORE_SRC := $(call preprocess, $(DOOMCORE_DIR)/SOURCES) \
                $(APPSDIR)/plugins/lib/helper.c \
                $(DOOMCORE_DIR)/doom_builtin.c
DOOMCORE_OBJ := $(call c2obj, $(DOOMCORE_SRC))

INCLUDES += -I$(DOOMCORE_DIR)
OTHER_SRC += $(DOOMCORE_SRC)

CORE_LIBS += $(DOOMCORELIB)

# Doom is plugin code: it reaches the core through rb->, and plugin.h only
# declares that pointer. doom_builtin.c defines it. PLUGIN itself is NOT
# defined - that would pull in the plugin header/loader machinery, which is
# exactly what this arrangement exists to avoid.
DOOMCORE_FLAGS = $(CFLAGS) -I$(APPSDIR) -I$(APPSDIR)/gui -I$(APPSDIR)/recorder \
                  -I$(APPSDIR)/plugins \
                  -Wno-strict-prototypes -O2 -fno-strict-aliasing -fgnu89-inline \
                  -Wno-stringop-truncation

$(DOOMCORE_OBJ): $(BUILDDIR)/%.o: $(ROOTDIR)/%.c
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) $(DOOMCORE_FLAGS) -c $< -o $@

$(DOOMCORELIB): $(DOOMCORE_OBJ)
	$(SILENT)$(shell rm -f $@)
	$(call PRINTS,AR $(@F))$(AR) rcs $@ $^ >/dev/null
