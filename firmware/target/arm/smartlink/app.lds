/* XIP image: code and rodata execute directly from SPI-NOR, .data is copied to
 * SRAM by crt0, and ALL of SRAM is left for bss, stacks and the audio buffer.
 *
 * Previously everything was loaded into SRAM, which left 64 bytes for the audio
 * buffer (audiobuffer 0x0082dc40, audiobufend 0x0082dc80) and hung audio_init().
 * .bss alone is ~180KB and the image ~336KB, so 512KB of SRAM could never hold
 * both. The vendor does not try: FIRM is 1.65MB running XIP from flash.
 *
 * Flash offset X is mapped at 0xc00000+X, but the window has a LIMITED EXTENT:
 * measured at run time, 0xc0e000 reads correctly while 0xda6000 reads as zeros.
 * The first XIP attempt linked at 0xda6000 - outside the window - so the ROM
 * jumped into unmapped space, fetched zeros and faulted before crt0 ran.
 *
 * The payload therefore goes at flash 0x00e000 (0x00c0e000), the FIRM partition's
 * slot: 1.6MB against our 336KB, proven mapped and executable because that is
 * exactly where the vendor's firmware runs from. FIRM is overwritten.
 */
#include "config.h"

ENTRY(start)
OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)

/* 0x00800000-0x00806000 is the boot ROM's own data and stack. The original
 * layout put .bss and the stack there on the grounds that it is "free the moment
 * the ROM hands over" - but this port keeps calling ROM functions for clocks,
 * GPIO and flash, so that region is NOT free. In the XIP build .data is copied
 * straight over it, clobbering the flash driver struct at 0x00800938, and crt0
 * zeroes .bss across the header structs at 0x008046c0 and 0x008049a0. The
 * symptom was every ROM clock query returning 0, so the LCD never got a pixel
 * clock. Start above it. */
#define SRAM_ORIG   0x00806000
#define SRAM_END    0x00880000
/* The Doom overlay window: the codec buffer plus the plugin buffer.
 *
 * NOT the audio buffer. That one is buflib's arena - the core allocator keeps
 * its bookkeeping there and the playback engine's allocations live in it - so
 * writing an overlay over it would corrupt the allocator rather than borrow
 * memory. The codec buffer holds a decoder only while something is playing, and
 * the plugin buffer belongs to whatever plugin is running, which here is Doom.
 * Both are ours the moment playback stops. */
#define POOL_START  (SRAM_END - CODEC_SIZE - PLUGIN_BUFFER_SIZE)
#define FLASH_XIP   0x00c0e000
#define FLASH_END   0x00dfc000

MEMORY
{
    FLASH (rx)  : ORIGIN = FLASH_XIP, LENGTH = FLASH_END - FLASH_XIP
    SRAM  (rwx) : ORIGIN = SRAM_ORIG, LENGTH = SRAM_END - SRAM_ORIG
    /* Deliberately overlapping SRAM: the codec and plugin buffers, reused by
       the Doom overlay while a game runs. */
    POOL  (rwx) : ORIGIN = POOL_START, LENGTH = SRAM_END - POOL_START
}

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
        *(.text*);
        *(.rodata*);
        . = ALIGN(4);
    } > FLASH

    /* --- lives in SRAM, initialiser image stored in flash --- */
    /* .icode joins .data so crt0's existing copy loop brings it into SRAM.
     *
     * Anything that reconfigures a clock MUST run from RAM: ROM clk_enable spins
     * on an acknowledge bit (0x23bc: [0x40080068] |= bit; while(!(reg & bit));)
     * and if the transition disturbs the flash controller, a CPU executing that
     * loop out of flash can never fetch the next instruction. */
    .data :
    {
        . = ALIGN(4);
        _datastart = .;
        *(.icode);
        *(.irodata);
        *(.idata);
        . = ALIGN(4);
        /* Doom's .data is not resident - it is copied into the overlay below
           when the game starts. See .doomdata. */
        *(EXCLUDE_FILE(*libdoomcore.a) .data*);
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
        /* 16 KB. It was 0x1800, and Rockbox panicked "stkov main" the first
         * time the boot got far enough to matter: the main thread's stack is
         * exactly stackbegin..stackend (thread-internal.h __get_main_stack) and
         * the kernel found the DEADBEEF sentinel at its bottom word overwritten.
         *
         * 6 KB was never sized against anything - the FAT mount and the skin
         * parser both run deep, and on Cortex-M every interrupt frame nests on
         * this same stack because handlers run on MSP. The watermark probe in
         * systick_handler now reports what is actually used, so the next value
         * here can be a measurement instead of a guess. */
        . += 0x4000;
        stackend = .;
    } > SRAM

    .bss (NOLOAD) :
    {
        _bssbegin = .;
        *(EXCLUDE_FILE(*libdoomcore.a) .bss*);
        *(EXCLUDE_FILE(*libdoomcore.a) COMMON);
        _bssend = .;
    } > SRAM

    /* Breadcrumbs in their own NOLOAD region: neither zeroed by crt0 (unlike
       .bss) nor overwritten by the image growing. */
    .breadcrumb (NOLOAD) : ALIGN(16)
    {
        __bc_start = .;
        . += 1280;  /* 320 slots - must match breadcrumb.h and crt0-sl6801.S */
    } > SRAM

    /* The plugin buffer must be REAL memory. It was previously SRAM_END, i.e.
       one byte past the end of SRAM, and it is not only plugins that use it:
       skin_data_load() takes its parse buffer from plugin_get_buffer(), so
       settings_apply_skins() parsed the default skin into unmapped space. Carve
       it out of the top of SRAM instead and end the audio buffer below it.

       codecbuf is a region of its own now, below the plugin buffer. It used to
       alias it, which was survivable only because nothing loaded a codec: the
       first track would have parsed a .codec straight over the plugin buffer.
       Upstream defines codecbuf in the .lds for every native target
       (apps/codecs.c:67), so this is the normal arrangement, not a local trick.

       The three of them plus .bss are the whole SRAM budget; the regions below
       make the allocation explicit. */
    pluginbuf   = SRAM_END - PLUGIN_BUFFER_SIZE;
    codecbuf    = pluginbuf - CODEC_SIZE;
    audiobuffer = ALIGN(32);
    audiobufend = codecbuf;

    /* --- Doom overlay -----------------------------------------------------
     *
     * Doom's 228 KB of code executes from flash like the rest of the image, but
     * its 172 KB of .data and .bss cannot be resident: that is more SRAM than
     * this device has to spare, and the linker assert above says so.
     *
     * They go here instead, on top of the codec and plugin buffers - 192 KB
     * that is dead while a game is running, because playback is stopped and no
     * codec is loaded. doom_builtin_main() copies .data in from flash and
     * zeroes .bss before the engine starts, which is exactly what the plugin
     * loader would have done had the plugin fitted.
     *
     * Nothing else may live in this window while Doom runs, and nothing does. */
    .doomdata :
    {
        __doom_data_start = .;
        *libdoomcore.a:(.data*);
        . = ALIGN(4);
        __doom_data_end = .;
    } > POOL AT> FLASH
    __doom_data_load = LOADADDR(.doomdata);

    .doombss (NOLOAD) :
    {
        __doom_bss_start = .;
        *libdoomcore.a:(.bss* COMMON);
        . = ALIGN(4);
        __doom_bss_end = .;
    } > POOL
    /* what is left of the window is Doom's zone heap */
    __doom_heap_start = __doom_bss_end;
    __doom_heap_end   = SRAM_END;
}

/* EXTERN forces these out of libfirmware.a - KEEP() alone does not pull archive
   members, which is why the vector table was silently missing from the image. */
EXTERN(__vectors_arm)
EXTERN(__vectors_platform)
ASSERT(__vectors_arm != 0, "vector table missing from image")
/* ASSERTs live out here rather than inside SECTIONS: ld's grammar takes them as
   statements there only in some positions, and the failure is a bare
   "syntax error" on the following line. */
ASSERT(audiobuffer < audiobufend,
       "SRAM exhausted: .bss + codecbuf + pluginbuf leave no audio buffer")
ASSERT(__doom_bss_end < SRAM_END, "Doom overlay does not fit in SRAM")
