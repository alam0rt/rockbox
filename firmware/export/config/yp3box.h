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
/* Brightness is PWM channel 3 on P1.17 (backlight-yp3box.c). The vendor's
 * set_brightness (FIRM 0xd106a0) takes 10..100 percent in steps of 10, built
 * as (level + 1) * 10 by its UI (0xd0ba02), so setting n is n*10 percent. The
 * default is the 60% read out of a live vendor system (SRAM 0x824258).
 * Defined unconditionally: the PWM itself sits behind YP3_BL_PWM in the
 * driver, and with that off the setting is accepted and has no effect. */
#define HAVE_BACKLIGHT_BRIGHTNESS
#define MIN_BRIGHTNESS_SETTING      1
#define MAX_BRIGHTNESS_SETTING      10
#define DEFAULT_BRIGHTNESS_SETTING  6
/* the panel can be blanked before power-off; see lcd_shutdown() */
#define HAVE_LCD_SHUTDOWN
/* lcd_enable()/lcd_active(), so the UI stops rendering into a dark panel
 * (yp3-1so.6). Defined unconditionally, like the brightness setting above:
 * the panel only really sleeps in a YP3_LCD_SLEEP build (lcd-yp3box.c), and
 * without that flag lcd_enable() is empty and lcd_active() is always true -
 * the behaviour before this was defined. Not HAVE_LCD_SLEEP: one sleep state
 * is all the GC9106 sequence here has. */
#define HAVE_LCD_ENABLE

#define CONFIG_KEYPAD       YP3BOX_PAD

#define CONFIG_STORAGE      STORAGE_SD

/* The microSD slot is the only storage and there is no card-detect line, so
 * the player can legitimately be running with no medium. */
#define HAVE_STORAGE_MAY_BE_ABSENT
#define HAVE_HOTSWAP
#define HAVE_HOTSWAP_STORAGE_AS_MAIN
#define HAVE_MULTIVOLUME
#define STORAGE_WANTS_ALIGN
#define SECTOR_SIZE         512

/* Battery and charging. The measurement is PMU-backed, not an ADC channel:
 * powermgmt-yp3box.c implements _battery_voltage() from the vendor's own
 * conversion (FIRM 0xcf7a28), and power-yp3box.c reads the charger bits out of
 * PMU status register 0x2a. CHARGING_MONITOR is the right level - the port can
 * see that the charger is attached and charging, but does not control it.
 *
 * The capacity figures are the cell in this device (210 mAh, single value, so
 * MIN == MAX and the setting is not adjustable). They feed runtime estimation
 * only; the voltage curve in powermgmt-yp3box.c is still a generic Li-ion
 * profile pending a discharge sweep. */
#define CONFIG_BATTERY_MEASURE   VOLTAGE_MEASURE
#define CONFIG_CHARGING          CHARGING_MONITOR
#define HAVE_SW_POWEROFF
#define BATTERY_CAPACITY_DEFAULT 210
#define BATTERY_CAPACITY_MIN     210
#define BATTERY_CAPACITY_MAX     210
#define BATTERY_CAPACITY_INC     0

#define CONFIG_I2C          I2C_NONE
/* The SoC's own calendar RTC at 0x40085000 (rtc-yp3box.c, docs/RTC.md). The
 * id is defined here, not in config.h's list (1..27): nothing shared switches
 * on it, and the driver is included by system-yp3box.c, so this is the whole
 * registration. No HAVE_RTC_ALARM - nothing shows the alarm can wake the
 * device. `[U]` until a boot has read it and a set has read back. */
#define RTC_SL6801          6801
#define CONFIG_RTC          RTC_SL6801

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

/* ICODE_ATTR into SRAM, for the core only.
 *
 * config.h picks USE_IRAM per CPU and this one is not on its list, so every
 * function Rockbox marks as hot - the DSP, the resampler, pcmbuf, the mixer,
 * the LCD blitters - stayed in SPI NOR. Measured on this part (2026-09-29,
 * system-yp3box.c yp3_measure_speed): each instruction fetch from flash is its
 * own SPI transaction, 262x the SRAM cost at 1-bit and 132x in quad, and a
 * sampling profile put 0.1% of the CPU in the codec. config.h only ever ADDS
 * USE_IRAM, and it includes this file first, so defining it here enables the
 * attributes without touching shared code.
 *
 * Core only, as for IMX233 and JZ47xx: codecs already execute from codecbuf,
 * which is SRAM, and no plugins are built. */
#if !defined(PLUGIN) && !defined(CODEC) && !defined(BOOTLOADER)
#define USE_IRAM
#endif

/* Codecs are loaded into codecbuf, so CODEC_SIZE has to be at least as large as
 * the biggest codec we intend to play. Measured on an ARM build (text+data+bss,
 * which is what gets loaded):
 *
 *     wavpack  41 KB     mpa (MP3) 111 KB     opus  >=131 KB
 *     wav      54 KB     vorbis     98 KB     aac   190 KB
 *                                             flac  194 KB
 *
 * This is a 512 KB device, so every byte here comes off the audio buffer, and
 * that buffer is what stops playback working: with 0x20000/0x10000 reserved
 * here, audio_reset_buffer_noalloc() panicked with "EOM (17440 > 14448)" - the
 * PCM ring wanted 17 KB and 14 KB of core arena was left.
 *
 * Both reservations are fixed cuts off the top of SRAM, so both are worth more
 * to playback than to what they hold:
 *
 * PLUGIN_BUFFER_SIZE. No plugins are built for this target at all - the
 * generated Makefile has an empty ENABLEDPLUGINS and rockbox.zip contains zero
 * .rock files, because the plugin keymaps do not exist yet (ROADMAP B4). 64 KB
 * was reserved for nothing. 8 KB keeps the loader's arithmetic honest and
 * leaves room for a launcher stub; raise it again in the same commit that
 * makes plugins build.
 *
 * CODEC_SIZE. A codec's .bss is sized by this and a codec that overflows fails
 * to LINK, so the number is a real gate, not a hint. Measured need, from the
 * built .elf files (bss end minus codecbuf):
 *
 *     mpa (MP3) 103 KB    atrac3_rm  98 KB    everything else smaller
 *     cook     128 KB     ay        120 KB    gbs       110 KB
 *
 * 0x1b000 is 108 KB: it keeps MP3, Vorbis, WAV and WavPack with about 4.6 KB
 * of headroom over mpa, and drops cook (RealAudio), ay (ZX Spectrum) and gbs
 * (Game Boy) - see the gates in lib/rbcodec/codecs/SOURCES. The codecs that
 * gate leaves out, measured by hand-linking each one (2026-11-09 `[C]`,
 * docs/CODEC-MEMORY.md): Opus 129,972 plus ~107 KB of heap, MPC 160,084,
 * APE 180,956, AAC 183,404, FLAC 196,352, WMA 284,540. Almost all of it is
 * .bss workspace, not code. */

#define PLUGIN_BUFFER_SIZE  0x2000

/* Codec slots in flash, COPIED TO SRAM AT LOAD. Each codec's whole image is
 * linked into a per-codec slot in SPI NOR; lc_open() memcpys it into codecbuf
 * and the codec runs from SRAM.
 *
 * It used to EXECUTE IN PLACE from the slot, so codecbuf held only .data and
 * .bss and was sized by the worst writable footprint (spc, 76,780) rather than
 * the worst total. That was measured and reverted: this part is a Cortex-M4
 * with no instruction cache, so every codec instruction came over the SPI-NOR
 * link at 32 MHz while the core ran at 192, and an MP3 decoded 26 ms of audio
 * every 2.5 seconds - about 1% of real time. docs/AUDIO.md has the log.
 *
 * So CODEC_SIZE is back to being sized by the worst TOTAL, and the slots are
 * now purely a delivery mechanism - no codec files on the card, no codec
 * buffering into the audio arena, and no XIP penalty on the decoder.
 *
 * See apps/plugins/plugin.lds (SL6801 branch), tools/codec_slots.txt for the
 * slot addresses, and tools/gen_codec_slots.py which feeds the link flags and
 * this target's slot table. The runtime half is target/arm/smartlink/yp3box/
 * lc-yp3box.c: it owns lc_open(), so nothing in apps/ or lib/rbcodec knows
 * this target keeps codecs in flash.
 *
 * The name is historical: HAVE_CODEC_XIP now means "codecs are delivered in
 * flash slots and copied to SRAM", which is the opposite of execute in place.
 * It is spelled this way in plugin.lds and config.h too, both shared, so the
 * rename waits for a change that touches them anyway (yp3-1so.16). The hang
 * this comment used to warn about was the XIP layout; MP3 has played through
 * the copied path since 2026-09-29 `[L]`. The two defines below turn on
 * together. */
#define HAVE_CODEC_XIP

/* lc-yp3box.c provides lc_open(); the stock card loader in lc-rock.c is still
 * built, as lc_open_from_file(), and every PLUGIN falls through to it. Codecs
 * do not: see the note on the fallback in lc-yp3box.c. Only meaningful
 * together with HAVE_CODEC_XIP. */
#define HAVE_LC_OPEN_TARGET

/* 0x1b000 is 110,592, sized by mpa's 105,840-byte total (text 67,552, data 4,
 * bss 38,284) with 4.7 KB of headroom - it is the largest codec built here.
 * vorbis is 90,388, atrac3_rm 100,204, spc 88,284.
 *
 * This is 0x8000 more than the execute-in-place layout needed, and those 32 KB
 * come off the audio arena: 184,640 bytes to 151,872. That is affordable and
 * the reasoning is already in docs/AUDIO.md - the vendor's own firmware
 * decodes with an 18 KB working buffer and NO file readahead at all, because
 * this board streams from SD by design. A 148 KB arena is still far more than
 * the hardware was built around. */
#define CODEC_SIZE          0x1b000

/* The tree cache is allocated before playback. The generic 1,000-entry default
 * consumes 52 KiB (40-byte names plus 12-byte entries) from the small core
 * arena, leaving only 444 bytes for the first audio allocation after the rest
 * of startup. Keep the browser useful while guaranteeing room for PCM and the
 * compressed-input reserve. Existing settings are clamped at tree init. */
#define MAX_FILES_IN_DIR_LIMIT 200

/* A QUARTER of a second of decoded-PCM lookahead. A whole one is 172 KB at
 * 44.1 kHz stereo and the entire audio arena here is about 151 KB.
 *
 * It was a half, and moving codecs out of SPI NOR and into codecbuf took 32 KB
 * off the arena and left the FILE buffer at 10,916 bytes - not enough to hold
 * even the first handle. The device buffered zero tracks, so there was no id3
 * (the UI showed "? bitrate"), the codec never loaded and nothing played
 * `[L]`. The arithmetic, all of it off the same arena:
 *
 *     ring 91,100 + guard 32,768 + reserve 16,384 = 140,252 of 151,168
 *
 * A quarter-second ring is about 47,000, which puts the file buffer at ~55 KB
 * - MORE than the 44,388 it had before codecs moved, so this is not a
 * compromise to pay for codecbuf, it is a ring that was oversized for this
 * device all along. docs/AUDIO.md records why: the vendor's own firmware
 * decodes with an 18 KB working buffer and no file readahead whatsoever,
 * because this board streams from SD at 2 MB/s by design.
 *
 * This replaced PCM_MIN_BUFFER_DIVISOR, a knob this port invented that existed
 * on no other target, so nothing upstream ever exercised the path and it broke
 * twice in a row: two 8 KiB chunks could never close a chunk, and five could
 * never reach a watermark still sized against the full-rate ring. The fraction
 * below is stated once and pcmbuf.c derives the watermark from it, so the two
 * cannot drift apart again. */
#define PCMBUF_LOOKAHEAD_DEN   4

/* The software-volume double buffer (pcm_sw_volume.c) at the size the mixer
 * actually fills. It defaults to 1024 samples a half, 8 KB in all, but it only
 * ever receives the mixer's frame - mix_frame_size = MIX_FRAME_SAMPLES * 4 =
 * 1024 bytes, with MIX_FRAME_SAMPLES 256 on this CPU (pcm_mixer.h) - so
 * update_frame_params() always computes one frame and 6 KB of the 8 were
 * never written. The 1024-byte DMA transfers measured on hardware are that
 * frame. 256 samples is exactly one mixer frame, so the transfers do not
 * change; the 6 KB goes back to the audio arena (yp3-1bw). */
#define PCM_PLAY_DBL_BUF_SAMPLES 256

#define AUDIO_BUFFER_RESERVE   0x4000

/* The file buffer here is ~54 KB and a 320 kbps watermark is 40 KB, so every
 * half-second low event rebuffered, could not load the next track while the
 * current one was still filling, and cleared and retried for ever - the PCM
 * ring ran dry ~11 times in 200 log records and the track stuttered (yp3-ov6).
 * With this, apps/playback.c lets the buffering thread finish the last track
 * before it tries to load another. Shared-core ledger entry in CLAUDE.md. */
#define AUDIO_BUFFER_LOW_WAITS_FOR_LAST

/* A 128x160 RGB565 default backdrop costs 40 KiB in core buflib. Keep that
 * memory available for the PCM ring and compressed-file buffer. */
#define DEFAULT_BACKDROP      "-"

#define CONFIG_TUNER        0

/* USB mass storage. Not a Rockbox usbstack port and not a device-controller
 * driver: the boot ROM is a complete Bulk-Only Transport / SCSI target and
 * takes its medium from a table of function pointers, so the target only
 * supplies that table, the module/clock bring-up and VBUS detection. See
 * firmware/target/arm/smartlink/yp3box/usb-yp3box.c and docs/USB.md.
 *
 * That means the !HAVE_USBSTACK path in firmware/usb.c, which is the right
 * one here: it unmounts, quiesces storage, calls usb_enable(true) and waits.
 * The device is bus powered and cannot switch charging, so no
 * HAVE_USB_POWER / HAVE_USB_CHARGING_ENABLE. */
#define CONFIG_USBOTG       USBOTG_SL6801_ROM

#define ROCKBOX_DIR         "/.rockbox"
#define BOOTDIR             "/.rockbox"
#define BOOTFILE_EXT        "yp3"
#define BOOTFILE            "rockbox." BOOTFILE_EXT

/* Debug channel. This port has no console: the breadcrumb region went away
 * with the instrumentation cleanup, and every audio and battery hypothesis
 * otherwise costs a full flash-and-boot cycle to test. Build with
 * `YP3_LOGF=1 ./tools/40_build.sh` (configure --type=al) and the log is
 * readable on the device under Debug -> "View log" and dumpable to /logf.txt
 * with Debug -> "Dump Log File".
 *
 * 16 KB of ring is not affordable here - the audio arena is about 141 KB and
 * the ring comes straight out of it (tools/arena_report.py prints both per
 * build) - so the ring is 6 KB, which holds a few hundred lines: about 25 s
 * of a playing boot `[L]`.
 *
 * Which files log is decided in tools/40_build.sh, which passes -DLOGF_ENABLE
 * per object. Rockbox's own convention is a commented-out #define at the top
 * of each file, and following it would have meant editing seven shared files -
 * playback, buffering, pcmbuf, codec_thread, metadata, mp3 and usb - to name
 * this target in each. A per-object flag says the same thing from outside. */
/* The black box. The log ring, the panic message and the fault registers live
 * in a linker region nothing clears, so they survive a panic, a hard fault, a
 * hang plus pinhole reset, and a watchdog reset; the next boot writes them to
 * /.rockbox/blackbox.txt before the UI comes up. Debug -> Dump Log File needs
 * a working UI and keypad, which is exactly what those failures take away.
 * See firmware/target/arm/smartlink/yp3box/blackbox-yp3box.c. */
#define HAVE_BLACKBOX
/* Runs kept on the card: the newest is blackbox.txt, the rest rotate to
 * blackbox.1.txt .. blackbox.4.txt. Five, because the failure worth reading
 * is often the one before the one you are looking at - and because a reboot
 * loop writes one per boot, so keeping only the latest would erase the cause
 * with the symptom. */
#define BLACKBOX_KEEP       5

#ifdef ROCKBOX_HAS_LOGF
#define MAX_LOGF_SIZE       6144
#endif

#define HAVE_SEMAPHORE_OBJECTS
#define INCLUDE_TIMEOUT_API
