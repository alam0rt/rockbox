#define __ASSEMBLER__
#include "config.h"
#include "cpu.h"

/* These output formats should be in the config-files */

#ifdef CPU_COLDFIRE
OUTPUT_FORMAT(elf32-m68k)
#elif defined(CPU_ARM)
OUTPUT_FORMAT(elf32-littlearm)
#elif defined(CPU_MIPS)
OUTPUT_FORMAT(elf32-littlemips)
#else
/* We can have an #error here we don't use this file when build sims! */
#error Unknown CPU architecture
#endif

/* MCF5249 have 96KB of IRAM */
#if CONFIG_CPU == MCF5249
#define DRAMORIG 0x31000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE
#define IRAMORIG 0x1000c000
#define IRAMSIZE 0xc000

/* MCF5250 have 128KB of IRAM */
#elif CONFIG_CPU == MCF5250
#define DRAMORIG 0x31000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE
#define IRAMORIG 0x1000c800
#define IRAMSIZE 0x13800

#elif CONFIG_CPU == PP5022 || CONFIG_CPU == PP5024
/* PP5022/24 have 128KB of IRAM */
#define DRAMORIG 0x00000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE
#define IRAMORIG 0x4000c000
#define IRAMSIZE 0x14000

#elif defined(CPU_PP)
/* all other PP's have 96KB of IRAM */
#define DRAMORIG 0x00000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE
#define IRAMORIG 0x4000c000
#define IRAMSIZE 0x0c000

#elif CONFIG_CPU == IMX31L
#define DRAMORIG 0x0
#define DRAMSIZE ((MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE \
                  - CODEC_SIZE - QHARRAY_SIZE - FRAME_SIZE - TTB_SIZE)

#elif CONFIG_CPU == S3C2440
#define DRAMORIG 0x0
#define DRAMSIZE ((MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE \
                  - CODEC_SIZE - LCD_BUFFER_SIZE - TTB_SIZE)

#elif CONFIG_CPU==DM320
#define DRAMORIG CONFIG_SDRAM_START
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE - LCD_TTB_AREA
/* The bit of IRAM that is available is used in the core */

#elif CONFIG_CPU==TCC7801
#define DRAMORIG 0x20000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE - TTB_SIZE
#define IRAMORIG 0x1000c000
#define IRAMSIZE 0xc000

#elif CONFIG_CPU==AS3525 && !defined(AMS_LOWMEM)
#define DRAMORIG DRAM_ORIG
#define DRAMSIZE (DRAM_SIZE - PLUGIN_BUFFER_SIZE - CODEC_SIZE - TTB_SIZE)
#define IRAMORIG (IRAM_ORIG + 0x20000)
#define IRAMSIZE (IRAM_ORIG + IRAM_SIZE - IRAMORIG)

#elif CONFIG_CPU==AS3525v2 || (CONFIG_CPU == AS3525 && defined(AMS_LOWMEM))
#define DRAMORIG DRAM_ORIG
#define DRAMSIZE (DRAM_SIZE - PLUGIN_BUFFER_SIZE - TTB_SIZE)
/* No IRAM defined, instead IRAM is exclusively used for the entire codec */
#define CODEC_ORIGIN (IRAM_ORIG + IRAM_SIZE - CODEC_SIZE)
#define PLUGIN_ORIGIN (DRAM_ORIG + DRAMSIZE)

#elif CONFIG_CPU==S5L8700
/* S5L8700 have 256KB of IRAM */
#define DRAMORIG 0x08000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE
#define IRAMORIG (0x00000000 + (48*1024))
#define IRAMSIZE (208*1024)

#elif CONFIG_CPU==S5L8701
/* S5L8701 have 176KB of IRAM */
#define DRAMORIG 0x08000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE
#define IRAMORIG (0x00000000 + (48*1024))
#define IRAMSIZE (128*1024)

#elif CONFIG_CPU==S5L8702
/* S5L8702 have 256KB of IRAM */
/* IRAM1 (2nd 128KB block) is not used because it is slower than DRAM */
#define DRAMORIG 0x08000000
#define DRAMSIZE (DRAM_SIZE - PLUGIN_BUFFER_SIZE - CODEC_SIZE - TTB_SIZE)
#define IRAMORIG (0x00000000 + (48*1024))
#define IRAMSIZE (80*1024)

#elif CONFIG_CPU==S5L8720
/* S5L8720 have 192KB of IRAM */
/* TODO: check if IRAM1 is also slow as in S5L8702 above */
#define DRAMORIG 0x08000000
#define DRAMSIZE (DRAM_SIZE - PLUGIN_BUFFER_SIZE - CODEC_SIZE - TTB_SIZE)
#define IRAMORIG (0x00000000 + (56*1024))
#define IRAMSIZE (136*1024)

#elif CONFIG_CPU == JZ4732 || CONFIG_CPU == JZ4760B
/* fake 'iram' region occupies first 16k */
#define DRAMORIG 0x80004000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE - 0x4000
/* The bit of IRAM that is available is used in the core */

#elif CONFIG_CPU == X1000
#define DRAMORIG X1000_DRAM_BASE
#define DRAMSIZE (X1000_DRAM_SIZE - PLUGIN_BUFFER_SIZE - CODEC_SIZE)

#elif CONFIG_CPU == RK27XX
#define DRAMORIG 0x60000000
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE

#elif CONFIG_CPU == IMX233
/* The IRAM is too small and already partly used by the core */
#define DRAMORIG CACHED_DRAM_ADDR
#define DRAMSIZE (DRAM_SIZE - PLUGIN_BUFFER_SIZE - CODEC_SIZE - FRAME_SIZE - TTB_SIZE)

#elif CONFIG_CPU == STM32H743
#define DRAMORIG STM32_SDRAM1_BASE
#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGIN_BUFFER_SIZE - CODEC_SIZE

#elif CONFIG_CPU == SL6801
/* YP3Box keeps 0x00800000-0x00806000 for the boot ROM. Keep codec and plugin
 * origins aligned with the XIP application linker script. */
#define DRAMORIG 0x00806000
#define DRAMSIZE (512 * 1024) - 0x6000 - PLUGIN_BUFFER_SIZE - CODEC_SIZE

/* Codec text and rodata execute in place from SPI NOR, so codecbuf only has to
 * hold what is actually writable - .data and .bss.
 *
 * codecbuf is a fixed cut off the top of a 512 KB SRAM and is sized by the
 * WORST codec, so read-only bytes sitting in it are pure loss: mpa is 105,784
 * bytes of which 24,032 is .text and 43,464 .rodata, and rq_table - libmad's
 * x^(4/3) requantisation lookup - is 32,828 on its own. The application has
 * always run this way (app.lds sends .text and .rodata to FLASH); codecs did
 * not, only because this script declared a single region and there was
 * nowhere else to put them.
 *
 * Each codec is linked at its own slot in the flash region above the
 * application image. The slot base arrives as a link-time symbol so one
 * script serves every codec:
 *
 *     -Wl,--defsym,CODEC_XIP_ORIGIN=0x00caa000
 *
 * tools/codec_slots.txt is the source of truth for those addresses and
 * tools/gen_codec_slots.py turns it into both the link flags and the resident
 * codec table the loader uses. The slot region is proven good: the vendor's
 * own FIRM occupies flash to 0xda2880 and executes in place from it.
 *
 * Layout is .header|.text|.rodata then .data's initialiser image, contiguous,
 * so the whole flash payload for a codec is one objcopy range and .data's LMA
 * is simply the end of .rodata. */
/* Enabled only with HAVE_CODEC_XIP, so the target header turns the whole
 * split on and off in one place. */
#ifdef HAVE_CODEC_XIP
#define CODEC_XIP 1
#endif

#else
# error "DRAM/IRAM memory map not defined!"
#endif

#define PLUGIN_LENGTH PLUGIN_BUFFER_SIZE


#ifndef CODEC_ORIGIN /* targets can specify another origin */
#define CODEC_ORIGIN (DRAMORIG + (DRAMSIZE))
#endif

#ifndef PLUGIN_ORIGIN /* targets can specify another origin */
#define PLUGIN_ORIGIN (CODEC_ORIGIN + CODEC_SIZE)
#endif

#ifdef CODEC
#define THIS_LENGTH CODEC_SIZE
#define THIS_ORIGIN CODEC_ORIGIN
#elif defined OVERLAY_OFFSET
#define THIS_LENGTH (DRAMSIZE - OVERLAY_OFFSET)
#define THIS_ORIGIN (DRAMORIG + OVERLAY_OFFSET)
#elif defined IMGVDECODER_OFFSET
#define THIS_LENGTH (PLUGIN_LENGTH - IMGVDECODER_OFFSET)
#define THIS_ORIGIN (PLUGIN_ORIGIN + IMGVDECODER_OFFSET)
#else /* plugin */
#define THIS_LENGTH PLUGIN_LENGTH
#define THIS_ORIGIN PLUGIN_ORIGIN
#endif

#if defined(USE_IRAM) && (!defined(IRAMORIG) || !defined(IRAMSIZE))
# error "USE_IRAM is set, but no IRAM region is defined!"
#endif

MEMORY
{
   PLUGIN_RAM : ORIGIN = THIS_ORIGIN, LENGTH = THIS_LENGTH
#if defined(USE_IRAM)
   PLUGIN_IRAM : ORIGIN = IRAMORIG, LENGTH = IRAMSIZE
#endif
}

SECTIONS
{
#if defined(CODEC) && defined(CODEC_XIP)
    /* Read-only sections at the codec's flash slot. Placed by address rather
     * than by region: a MEMORY origin must be a constant at script-parse time,
     * an output section address may be a link-time symbol, and that is what
     * lets one script serve every codec. */
    /* A descriptor at the slot base, so the loader can find .data's load
     * address, destination and length by reading flash at run time. Putting it
     * here rather than in a table compiled into the firmware removes a build
     * ordering problem: the firmware would otherwise have to be linked after
     * every codec in order to know addresses only their links produce. The
     * firmware needs to know nothing but the slot base, which comes from
     * tools/codec_slots.txt and is fixed. */
    .codecxip CODEC_XIP_ORIGIN : {
        LONG(0x59503358);               /* 'YP3X' */
        LONG(_codec_xip_dataload);
        LONG(_codec_xip_data_start);
        LONG(_codec_xip_data_end);
    }

    .header : {
        KEEP(*(.header))
    }

    /* The header's load_addr is the codec's RAM base, NOT where the header
     * itself lives: codec_load_ram() checks `hdr->load_addr != codecbuf` and
     * rejects the codec if it differs. With the split the header is in flash,
     * so point these at PLUGIN_RAM's origin, which is codecbuf, and is exactly
     * where .data lands. The companion check on end_addr still guards a .bss
     * that would overflow codecbuf. */
    _plugin_start_addr = ORIGIN(PLUGIN_RAM);
    plugin_start_addr = ORIGIN(PLUGIN_RAM);

    .text :
    {
        *(.text*)
#ifdef CPU_ARM
        *(.glue_7)
        *(.glue_7t)
#endif
    }

    .rodata :
    {
        *(.rodata*)
    }

    /* Place the unwind tables explicitly. Left as orphans ld puts them at the
     * current location counter - which is after .rodata, exactly where .data's
     * load address is - and the link fails with an LMA overlap. In the old
     * single-region script they were orphans too, but everything was flowing
     * into PLUGIN_RAM in order so nothing noticed. */
    .ARM.exidx :
    {
        *(.ARM.exidx*)
    }

    .ARM.extab :
    {
        *(.ARM.extab*)
    }

    . = ALIGN(4);
    _codec_xip_dataload = .;

    /* VMA in codecbuf, initialiser image in flash directly behind the code. */
    .data : AT(_codec_xip_dataload)
    {
        _codec_xip_data_start = .;
        *(.data*)
        . = ALIGN(4);
        _codec_xip_data_end = .;
    } > PLUGIN_RAM
#else
    .header : {
        _plugin_start_addr = .;
        plugin_start_addr = .;
        KEEP(*(.header))
    } > PLUGIN_RAM

    .text :
    {
        *(.text*)
#ifdef CPU_ARM
        *(.glue_7)
        *(.glue_7t)
#endif
    } > PLUGIN_RAM

    .rodata :
    {
        *(.rodata*)
    } > PLUGIN_RAM

    .data :
    {
        *(.data*)
    } > PLUGIN_RAM
#endif

#if defined(NOCACHE_BASE)
    /*
     * Allocate .ncdata based on the following constraints:
     *
     * 1. the LMA must be allocated at the next free LMA in
     *    PLUGIN_RAM and aligned to a cacheline boundary.
     * 2. the VMA - LMA offset must equal NOCACHE_BASE.
     */
    .ncdata ALIGN(. + NOCACHE_BASE, CACHEALIGN_SIZE) : ALIGN(CACHEALIGN_SIZE)
    {
        *(.ncdata*)
        . = ALIGN(CACHEALIGN_SIZE);
    } AT> PLUGIN_RAM
#endif

    /DISCARD/ :
    {
        *(.eh_frame)
#if !defined(USE_IRAM)
		*(.icode)
        *(.irodata)
        *(.idata)
        *(.ibss)
#endif
#ifdef CPU_MIPS
        *(.MIPS.abiflags)
        *(.rel.dyn)
#endif
    }

    .bss (NOLOAD) :
    {
    	plugin_bss_start = .;
    	_plugin_bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(0x4);
    } > PLUGIN_RAM

#if defined(NOCACHE_BASE)
    /*
     * .ncbss has the same constraints as the .ncdata section
     * above but there is an extra complication: because it is
     * a NOLOAD section, unlike .ncdata it cannot consume any
     * VMAs or LMAs in PLUGIN_RAM itself.
     *
     * Because free VMA/LMAs in PLUGIN_RAM are unchanged, any
     * subsequent sections will get placed at the end of the
     * .bss section, overlapping .ncbss, which we don't want.
     *
     * One solution is to manually place the next sections to
     * account for the size of .ncbss, but a cleaner solution
     * is to define a dummy section (.ncbss_vma) allocated at
     * the same VMA as .ncbss and equal in size, and placed in
     * the PLUGIN_RAM region. This allocates the correct VMA
     * range in plugin RAM so that the next section will not
     * overlap .ncbss.
     */
    .ncbss ALIGN(. + NOCACHE_BASE, CACHEALIGN_SIZE) (NOLOAD) : ALIGN(CACHEALIGN_SIZE)
    {
    	*(.ncbss*)
        . = ALIGN(CACHEALIGN_SIZE);
    } AT> PLUGIN_RAM

    .ncbss_vma (NOLOAD) : ALIGN(CACHEALIGN_SIZE)
    {
        . += SIZEOF(.ncbss);
    } > PLUGIN_RAM
#endif

    /* Final end of plugin after IRAM setup. The plugin or codec buffer
       is considered unused by the in-RAM image at this point once IRAM
       is copied. */
    .pluginend (NOLOAD) :
    {
        _plugin_end_addr = .;
        plugin_end_addr = .;
    } > PLUGIN_RAM

#if defined(USE_IRAM)
    .iram IRAMORIG : AT(LOADADDR(.bss))
    {
        iramstart = .;
        *(.icode)
        *(.irodata)
        *(.idata)
        iramend = .;
    } > PLUGIN_IRAM
    iramcopy = LOADADDR(.iram);

    .ibss (NOLOAD) :
    {
        iedata = .;
        *(.ibss)
        . = ALIGN(0x4);
        iend = .;
    } > PLUGIN_IRAM

    loadendaddr = MAX(plugin_end_addr, LOADADDR(.iram) + SIZEOF(.iram));
#else
    loadendaddr = plugin_end_addr;
#endif

    /* This is for ovl_offset.pl and is the highest address that must
       be loaded into the plugin buffer (past the end of last data in
       stored image). */
    .pluginloadend loadendaddr :
    {
    	_plugin_load_end_addr = .;
    	plugin_load_end_addr = .;
    }

    _plugin_ram_end = ORIGIN(PLUGIN_RAM) + LENGTH(PLUGIN_RAM);
    plugin_ram_end = ORIGIN(PLUGIN_RAM) + LENGTH(PLUGIN_RAM);

    /* Special trick to avoid a linker error when no other sections are
       left after garbage collection (plugin not for this platform) */
    .comment 0 :
    {
        KEEP(*(.comment))
    }
}

#if defined(NOCACHE_BASE)
/* Some asserts to make sure nocache sections appear correctly defined */
ASSERT(LOADADDR(.ncdata) == ADDR(.ncdata) - NOCACHE_BASE, ".ncdata has incorrect LMA/VMA address");
ASSERT(LOADADDR(.ncdata) % CACHEALIGN_SIZE == 0,          ".ncdata incorrectly aligned");
ASSERT(LOADADDR(.ncbss) == ADDR(.ncbss) - NOCACHE_BASE,   ".ncbss has incorrect LMA/VMA address");
ASSERT(LOADADDR(.ncbss) % CACHEALIGN_SIZE == 0,           ".ncbss incorrectly aligned");
ASSERT(ADDR(.ncbss_vma) == LOADADDR(.ncbss),              ".ncbss_vma has wrong address");
ASSERT(SIZEOF(.ncbss_vma) == SIZEOF(.ncbss),              ".ncbss_vma has wrong size");
#endif
