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
DOOMCORE_HELPER_HDR := $(BUILDDIR)/lib/helper.h


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

DOOMSTUB := $(BUILDDIR)/apps/plugins/doom.rock
DOOMSTUB_OBJ := $(BUILDDIR)/apps/plugins/doom.o
DOOMSTUB_CRT0 := $(BUILDDIR)/apps/plugins/plugin_crt0.o
DOOMSTUB_LDS := $(BUILDDIR)/apps/plugins/plugin.link
DOOMSTUB_FLAGS = $(CFLAGS) -DPLUGIN -I$(APPSDIR) -I$(APPSDIR)/gui \
                 -I$(APPSDIR)/recorder -I$(APPSDIR)/plugins
$(DOOMSTUB_OBJ) $(DOOMSTUB_CRT0): $(BUILDDIR)/sysfont.h $(BUILDDIR)/lang/lang.h


# The target disables the general plugin suite, but the browser still needs the
# small standard .rock launcher for built-in Doom. Keep this path independent of
# plugins.make so building the launcher cannot pull in every plugin library.
build: $(DOOMSTUB)
$(DOOMSTUB_LDS): $(APPSDIR)/plugins/plugin.lds $(FIRMDIR)/export/config/$(MODELNAME).h
	$(call PRINTS,PP $(@F))
	$(shell mkdir -p $(dir $@))
	$(call preprocess2file,$<,$@,-DPLUGIN)
$(DOOMSTUB_OBJ): $(APPSDIR)/plugins/doom.c $(APPSDIR)/plugin.h
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) $(DOOMSTUB_FLAGS) -c $< -o $@

$(DOOMSTUB_CRT0): $(APPSDIR)/plugins/plugin_crt0.c $(APPSDIR)/plugin.h
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) $(DOOMSTUB_FLAGS) -c $< -o $@

$(DOOMSTUB): $(DOOMSTUB_OBJ) $(DOOMSTUB_CRT0) $(DOOMSTUB_LDS) $(SETJMPLIB)
	$(call PRINTS,LD $(@F))$(CC) $(DOOMSTUB_FLAGS) -nostdlib -o $@.elf \
		$(DOOMSTUB_OBJ) $(DOOMSTUB_CRT0) $(SETJMPLIB) -lgcc -T$(DOOMSTUB_LDS)
	$(call PRINTS,OC $(@F))$(OC) -S -x $@.elf $@

$(DOOMCORE_OBJ): $(BUILDDIR)/%.o: $(ROOTDIR)/%.c
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) $(DOOMCORE_FLAGS) -c $< -o $@
$(DOOMCORE_HELPER_HDR): $(APPSDIR)/plugins/lib/helper.h
	$(SILENT)mkdir -p $(dir $@)
	$(SILENT)cp $< $@


$(DOOMCORELIB): $(DOOMCORE_OBJ)
	$(SILENT)$(shell rm -f $@)
	$(call PRINTS,AR $(@F))$(AR) rcs $@ $^ >/dev/null
