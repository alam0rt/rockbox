/*
 * anko 43667840 / yp3_2.0.46 - Smartlink SL6801
 *
 * Values here are measured on hardware:
 *   Cortex-M4F, cpu pll 384 MHz / core pll 192 MHz, ahb 32 MHz
 *   512 KB SRAM at 0x800000, SPI NOR XIP-mapped at 0xc00000
 *   GC9106 128x160 RGB565 panel behind an LCDC at 0x40095000
 */
#define MODEL_NAME      "anko yp3box"
#define MODEL_NUMBER    126

#define CONFIG_CPU          SL6801
#define CPU_FREQ            192000000
/* Rate the general-purpose timer API counts at. Same as the core clock: the
 * timer we drive it from is the Cortex-M SysTick's sibling. */
#define TIMER_FREQ          CPU_FREQ


#define CONFIG_LCD          LCD_YP3BOX
#define LCD_WIDTH           128
#define LCD_HEIGHT          160
#define LCD_DEPTH           16
#define LCD_PIXELFORMAT     RGB565
#define LCD_DPI             120
#define HAVE_LCD_COLOR
#define HAVE_LCD_BITMAP

#define HAVE_BACKLIGHT
/* the panel can be blanked before power-off; see lcd_shutdown() */
#define HAVE_LCD_SHUTDOWN

#define CONFIG_KEYPAD       YP3BOX_PAD

#define CONFIG_STORAGE      STORAGE_SD
#define HAVE_HOTSWAP
#define HAVE_HOTSWAP_STORAGE_AS_MAIN
#define HAVE_MULTIVOLUME
#define STORAGE_WANTS_ALIGN
#define SECTOR_SIZE         512

#define CONFIG_I2C          I2C_NONE
#define CONFIG_RTC          0

/* boot test: we only need UTF-8, not legacy codepages */
#define MAX_CP_TABLE_SIZE 256
/* The disk cache is not a cache in the "make it smaller and lose speed" sense:
 * every open file and directory takes one entry OUT of it for as long as it is
 * open (file_cache_alloc -> dc_get_buffer), and running out is a panic, not a
 * slow path. Four entries meant the fifth concurrent stream killed the device -
 * "file_cache_alloc - OOM" in the tree browser, which opens a dir and reads
 * files under it while the settings and skin streams are still up.
 *
 * fs_defines.h sizes it for the worst case, MAX_OPEN_FILES + MAX_OPEN_DIRS +
 * AUX_FILEOBJS = 11 + 12 + 3, and picks 32 for MEMORYSIZE < 8. That is 16 KB
 * taken off the audio buffer (225 KB -> 211 KB), which is affordable; guessing
 * a smaller number is not, because the next guess costs another boot. */
#define DC_NUM_ENTRIES_OVERRIDE 32
#define HAVE_DUMMY_CODEC
#define HAVE_SW_VOLUME_CONTROL
#define HAVE_SW_TONE_CONTROLS

/* Codecs are loaded into codecbuf, so CODEC_SIZE has to be at least as large as
 * the biggest codec we intend to play. Measured on an ARM build (text+data+bss,
 * which is what gets loaded):
 *
 *     wavpack  41 KB     mpa (MP3) 111 KB     opus  >=131 KB
 *     wav      54 KB     vorbis     98 KB     aac   190 KB
 *                                             flac  194 KB
 *
 * 128 KB covers MP3, Vorbis, WAV and WavPack. FLAC, AAC and Opus do not fit;
 * the measured codec sizes above show what fits in this buffer.
 * This is a 512 KB device, so every byte here comes off the audio buffer. */

#define PLUGIN_BUFFER_SIZE  0x10000
#define CODEC_SIZE          0x20000

/* The tree cache is allocated before playback. The generic 1,000-entry default
 * consumes 52 KiB (40-byte names plus 12-byte entries) from the small core
 * arena, leaving only 444 bytes for the first audio allocation after the rest
 * of startup. Keep the browser useful while guaranteeing room for PCM and the
 * compressed-input reserve. Existing settings are clamped at tree init. */
#define MAX_FILES_IN_DIR_LIMIT 200

/* The core audio arena must also coexist with the tree cache, fonts and skin
 * data. The default 5-chunk ring can leave no contiguous allocation after the
 * UI has initialized. Use two 8 KiB chunks and no default 40 KiB backdrop;
 * explicit backdrops remain optional and fail cleanly if memory is tight. */
#define PCM_MIN_BUFFER_DIVISOR 8
#define AUDIO_BUFFER_RESERVE   0x4000
/* A 128x160 RGB565 default backdrop costs 40 KiB in core buflib. Keep that
 * memory available for the PCM ring and compressed-file buffer. */
#define DEFAULT_BACKDROP      "-"

#define CONFIG_TUNER        0
#define USB_NONE

#define ROCKBOX_DIR         "/.rockbox"
#define BOOTDIR             "/.rockbox"
#define BOOTFILE_EXT        "yp3"
#define BOOTFILE            "rockbox." BOOTFILE_EXT

#define HAVE_SEMAPHORE_OBJECTS
#define INCLUDE_TIMEOUT_API
