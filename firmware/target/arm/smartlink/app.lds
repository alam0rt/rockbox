/* XIP image: code and rodata execute directly from SPI-NOR, .data is copied to
 * SRAM by crt0, and ALL of SRAM is left for bss, stacks and the audio buffer.
 *
 * Previously everything was loaded into SRAM, leaving only 64 bytes for the
 * audio buffer. The .bss alone is about 180KB and the image is about 336KB, so
 * 512KB of SRAM cannot hold both. The vendor firmware executes from flash.
 *
 * Flash offset X is mapped at 0xc00000+X, but the window has a LIMITED EXTENT:
 * measured at run time, 0xc0e000 reads correctly while 0xda6000 reads as zeros.
 * The first XIP attempt linked at 0xda6000 - outside the window - so the ROM
 * jumped into unmapped space, fetched zeros and faulted before crt0 ran.
 *
 * The payload goes at flash 0x00e000 (0x00c0e000), the FIRM partition slot.
 * The 1.6MB slot is larger than our 336KB image and is overwritten.
 */
#include "config.h"

ENTRY(start)
OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)

/* 0x00800000-0x00806000 is reserved by the boot ROM. Keep Rockbox data above
 * it because this port continues calling ROM clock, GPIO and flash functions. */
#define SRAM_ORIG   0x00806000
#define SRAM_END    0x00880000
/* Reserve the top of SRAM for the codec and plugin buffers. */
#define FLASH_XIP   0x00c0e000
#define FLASH_END   0x00dfc000

/* Hot code copied into SRAM by crt0 with .data, chosen by the YP3_PROFILE
 * sampler rather than by guessing (2026-09-29). With playback working, 49% of
 * ALL CPU was resample_hermite and 9% dsp_sample_input, both executing from
 * SPI NOR at ~132x the SRAM cost, and 15% the scheduler. These nine objects
 * are ~10 KB of code. Archive members are named exactly: "*thread.o" would
 * also catch codec_thread.o and audio_thread.o. */
#ifdef YP3_BT_BLOB
#define SRAM_HOT_BT *libfirmware.a:semaphore.o
#else
#define SRAM_HOT_BT
#endif
#define SRAM_HOT \
    *libfirmware.a:thread.o *libfirmware.a:pcm_mixer.o \
    *libfirmware.a:pcm_sw_volume.o *pcmbuf.o \
    *librbcodec.a:resample.o *librbcodec.a:dsp_core.o \
    *librbcodec.a:dsp_sample_input.o *librbcodec.a:dsp_sample_output.o \
    *librbcodec.a:dsp_misc.o SRAM_HOT_BT

/* YP3_BT_BLOB (config/yp3box.h, the yp3-bt branch only): the vendor
 * Bluetooth stack runs at its own SRAM addresses, so one hole is left for it
 * - BT_HOLE_LO..BT_HOLE_HI, tools/bt/btpack.py HOLE_LO/HOLE_HI in the yp3box
 * repository, and bt-yp3box.c checks the two agree. Below the hole go three
 * things whose size no build switch moves: the stacks, the framebuffer and
 * the buffering thread's tables. Everything else links above it, so the
 * hole costs its own 54.5 KB and about 2 KB of slack. NOT YET RUN. */
#ifdef YP3_BT_BLOB
#define BT_HOLE_LO  0x008172e0
#define BT_HOLE_HI  0x008247e8
#define SRAM_BASE   BT_HOLE_HI
#define LO_OBJECTS  *libfirmware.a:lcd-16bit.o *buffering.o
#else
#define SRAM_BASE   SRAM_ORIG
#endif

MEMORY
{
    FLASH (rx)  : ORIGIN = FLASH_XIP, LENGTH = FLASH_END - FLASH_XIP
#ifdef YP3_BT_BLOB
    SRAM_LO (rwx) : ORIGIN = SRAM_ORIG, LENGTH = BT_HOLE_LO - SRAM_ORIG
#endif
    SRAM  (rwx) : ORIGIN = SRAM_BASE, LENGTH = SRAM_END - SRAM_BASE
}
#ifndef YP3_BT_BLOB
REGION_ALIAS("SRAM_LO", SRAM)
#endif

SECTIONS
{
    /* --- executed in place from flash --- */
    .text :
    {
        loadaddress = .;
        KEEP(*(.vectors.arm));
        KEEP(*(.vectors.platform));
        . = ALIGN(4);
        *(.init.text*);
        *(.init*);
        *(EXCLUDE_FILE(SRAM_HOT) .text*);
        *(EXCLUDE_FILE(SRAM_HOT) .rodata*);
        . = ALIGN(4);
    } > FLASH

    /* --- lives in SRAM, initialiser image stored in flash --- */
    /* .icode joins .data so crt0's copy loop brings it into SRAM. Clock changes
     * must run from RAM because an XIP fetch can stall during
     * reconfiguration. */
    .data :
    {
        . = ALIGN(4);
        _datastart = .;
        *(.icode);
        /* SRAM_HOT, one statement per file: ld takes one section list per
           file pattern. Keep in step with the macro above. */
        *libfirmware.a:thread.o(.text* .rodata*);
        *libfirmware.a:pcm_mixer.o(.text* .rodata*);
        *libfirmware.a:pcm_sw_volume.o(.text* .rodata*);
        *pcmbuf.o(.text* .rodata*);
        *librbcodec.a:resample.o(.text* .rodata*);
        *librbcodec.a:dsp_core.o(.text* .rodata*);
        *librbcodec.a:dsp_sample_input.o(.text* .rodata*);
        *librbcodec.a:dsp_sample_output.o(.text* .rodata*);
        *librbcodec.a:dsp_misc.o(.text* .rodata*);
#ifdef YP3_BT_BLOB
        /* The BT interrupt posts to a semaphore on every event. */
        *libfirmware.a:semaphore.o(.text* .rodata*);
#endif
        *(.irodata);
        *(.idata);
        . = ALIGN(4);
        *(.data*);
        . = ALIGN(4);
        _dataend = .;
    } > SRAM AT> FLASH
    _datacopy = LOADADDR(.data);

    .stack (NOLOAD) : ALIGN(8)
    {
        irqstackbegin = .;
        . += 0x400;
        irqstackend = .;
        stackbegin = .;
        /* Keep enough stack for filesystem and skin initialization. */
        . += 0x4000;

        stackend = .;

    } > SRAM_LO

    /* Cleared by crt0 like .bss. Empty unless YP3_BT_BLOB. */
    .bss_lo (NOLOAD) :
    {
        _bsslobegin = .;
#ifdef YP3_BT_BLOB
        *libfirmware.a:lcd-16bit.o(.bss* COMMON);
        *buffering.o(.bss* COMMON);
#endif
        . = ALIGN(4);
        _bssloend = .;
    } > SRAM_LO

    .bss (NOLOAD) :
    {
        _bssbegin = .;
        /* IBSS_ATTR (USE_IRAM, see config/yp3box.h). All of SRAM is IRAM
           here, so it simply joins .bss and is cleared with it. */
        *(.ibss);
        /* logf.o is excluded: its ring, index and wrap flag go into .persist
           below so they survive a reset. crt0 clears _bssbegin.._bssend. */
        *(EXCLUDE_FILE (*logf.o) .bss*);
        *(EXCLUDE_FILE (*logf.o) COMMON);
        . = ALIGN(4);
        _bssend = .;
    } > SRAM

    /* --- the black box -----------------------------------------------------
     *
     * SRAM that NOTHING clears: it is outside _bssbegin.._bssend so crt0's
     * clear skips it, outside _datastart.._dataend so the .data copy skips
     * it, and the boot ROM only initialises 0x800000-0x806000, so it also
     * survives a pinhole reset and a SYSRESETREQ.
     *
     * That is the whole point. A log you can only reach through Debug ->
     * Dump Log File is worthless for the failures worth logging: a hang, a
     * hard fault, or a dead keypad leave no way to drive the menu. Here the
     * log outlives the crash, and the next boot writes it to the card before
     * anything else can go wrong.
     */
    .persist (NOLOAD) : ALIGN(8)
    {
        _persistbegin = .;
        /* FIRST, and fixed at 32 bytes: crt0 reads and writes this before
           .data is copied or .bss is cleared, so it addresses the fields by
           constant offset. Anything inserted ahead of it silently moves every
           offset the assembly uses. See rig-yp3box.c for the layout. */
        yp3_bootctl = .;
        . += 32;                        /* struct yp3_bootctl */
        yp3_blackbox = .;
        . += 192;                       /* struct yp3_blackbox */
        yp3_flashlog = .;
        . += 16;                        /* quad switch stages, see system-yp3box.c */
        yp3_usblog = .;
        . += 64;                        /* USB MSC stage words, see usb-yp3box.c */
        *logf.o(.bss* COMMON);
        /* pcm-yp3box.c's register snapshot - self-test builds only, so this
           is empty otherwise. After the ring: nothing addresses it by a
           fixed offset. */
        . = ALIGN(4);
        KEEP(*(.yp3snap));
        /* bt-yp3box.c's stage crumb; empty without YP3_BT_BLOB. */
        KEEP(*(.yp3bt));
        _persistend = .;
    } > SRAM


    /* The plugin and codec buffers must be real SRAM. Keep the audio buffer
       below both regions so the allocators cannot overlap them. */
    pluginbuf   = SRAM_END - PLUGIN_BUFFER_SIZE;
    codecbuf    = pluginbuf - CODEC_SIZE;
    audiobuffer = ALIGN(32);
    audiobufend = codecbuf;

}

/* EXTERN forces these vector tables out of libfirmware.a. KEEP() alone does not
   pull archive members, so the vectors would otherwise be missing. */
EXTERN(__vectors_arm)
EXTERN(__vectors_platform)
ASSERT(__vectors_arm != 0, "vector table missing from image")
/* Keep the audio buffer non-empty after reserving codec and plugin memory. */
ASSERT(audiobuffer < audiobufend,
       "SRAM exhausted: .bss + codecbuf + pluginbuf leave no audio buffer")
