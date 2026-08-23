#include "system.h"
#include "system-target.h"
#include "sl6801.h"
#include "panic.h"
#include "breadcrumb.h"


#define ROM_CLK_FREQ ((uint32_t (*)(unsigned))0x3851u)

/* "used" and its own section: without them the compiler folds the constant into
 * the store and emits no symbol, so the reader has nothing to look up. */
__attribute__((used, section(".rodata.buildid")))
/* volatile too: otherwise the compiler folds the value into the store, nothing
 * references the section, and --gc-sections drops it - the symbol was in the
 * object file but not the linked ELF. */
const volatile uint32_t yp3_build_id =
      ((uint32_t)__TIME__[0] <<  0) ^ ((uint32_t)__TIME__[1] <<  4)
    ^ ((uint32_t)__TIME__[3] <<  8) ^ ((uint32_t)__TIME__[4] << 12)
    ^ ((uint32_t)__TIME__[6] << 16) ^ ((uint32_t)__TIME__[7] << 20)
    ^ ((uint32_t)__DATE__[4] << 24) ^ ((uint32_t)__DATE__[5] << 28);

/* The clock tree the vendor bootloader sets up, which we skip entirely.
 *
 * Measured on our boot path: AHB 12MHz, SPI flash 4MHz, core pll 384MHz. The
 * ROM in bootloader mode leaves AHB at 32MHz and flash at 12MHz, and the vendor
 * bootloader then programs the tree properly before it ever touches the panel.
 * We inherit whatever the ROM's normal-boot path leaves, which is a third of the
 * speed - hence ~10s to first light and every delay loop running ~3x long.
 *
 * Transcribed from bootloader 0x8206e4, called as f(2, 384000000, 32000000)
 * from 0x8203ce with the literals at 0x820404/0x820408:
 *
 *   clkbase[0x60] = 0                 clkbase = 0x40085000
 *   clk_set_source(3, 10)             switch to a safe source first
 *   clk_set_source(6, 10)
 *   rom_4c014(6)
 *   flash_timing()                    0x820664: flash controller for the new speed
 *   rom_3904(2, 384000000)            program the PLL
 *   rom_2a7c(8); rom_2a7c(9)
 *   clkbase[0xd8] &= ~2
 *   clk_set_divider(3, 2) (4, 1) (5, 1)
 *   clk_set_divider(6, 384000000/32000000 = 12)
 *   rom_4c014(6)
 *   clk_set_source(3, 2)              back onto the PLL
 *   clk_set_source(6, 2)
 *
 * MUST run from SRAM: it reprograms the clock that fetches our own
 * instructions, and it retimes the flash controller mid-flight.
 */
#define ROM_CLK_SRC2  ((void (*)(unsigned, unsigned))0x3119u)
#define ROM_CLK_DIV2  ((void (*)(unsigned, unsigned))0x2d59u)
#define ROM_CLK_APPLY2 ((void (*)(unsigned))0x27a1u)
#define NORF_CLOCK    0x2bu
#define ROM_4C014     ((void (*)(unsigned))0x4c015u)
#define ROM_3904      ((void (*)(unsigned, unsigned))0x3905u)
#define ROM_2A7C      ((void (*)(unsigned))0x2a7du)
#define CLK2(o)       (*(volatile uint32_t *)(0x40085000u + (o)))
#define FLASHC(o)     (*(volatile uint32_t *)(0x40000000u + (o)))

#define YP3_CLOCK_INIT 1
/* Set to 0 if the flash cannot take 32 MHz - see the block at the end of
 * yp3_clock_init(). Slots 133..135 say whether it survived. */
#define YP3_FLASH_CLOCK 1

#if YP3_CLOCK_INIT
static void __attribute__((section(".icode"), noinline)) yp3_flash_timing(void)
{
    /* bootloader 0x820664 */
    uint32_t v = FLASHC(0x04);
    v &= 0xf8f8f8f8u;
    v &= ~0x00b800b8u;
    v |= 0x04000400u;
    v |= 0x00830083u;
    FLASHC(0x04) = v;
    v = FLASHC(0x08);
    v &= ~0x1fu;
    v |= 0x13u;
    FLASHC(0x08) = v;
}

static void __attribute__((section(".icode"), noinline)) yp3_clock_init(void)
{
    volatile uint32_t d;

    BC_SLOT(126) = 0xC10C1001u;
    CLK2(0x60) = 0;
    ROM_CLK_SRC2(3, 10);
    ROM_CLK_SRC2(6, 10);
    ROM_4C014(6);
    BC_SLOT(126) = 0xC10C1002u;      /* survived the switch to the safe source */

    yp3_flash_timing();
    BC_SLOT(126) = 0xC10C1003u;      /* survived the flash retiming */

    ROM_3904(2, 384000000u);
    ROM_2A7C(8);
    ROM_2A7C(9);
    BC_SLOT(126) = 0xC10C1004u;      /* survived the PLL program */

    CLK2(0xd8) &= ~2u;
    ROM_CLK_DIV2(3, 2);
    ROM_CLK_DIV2(4, 1);
    ROM_CLK_DIV2(5, 1);
    ROM_CLK_DIV2(6, 384000000u / 32000000u);
    ROM_4C014(6);
    BC_SLOT(126) = 0xC10C1005u;

    ROM_CLK_SRC2(3, 2);
    ROM_CLK_SRC2(6, 2);
    for (d = 0; d < 100000u; d++) ;
    BC_SLOT(126) = 0xC10C100Du;      /* complete, running on the PLL */

    /* THE SPI-NOR CLOCK - the other half of a step that was only half ported.
     *
     * yp3_flash_timing() above is bootloader 0x820664: it retimes the flash
     * CONTROLLER for a faster clock. The bootloader then gives the controller
     * that clock, in its flash setup at 0x820914:
     *
     *     r4  = (cfg[0] == 1 && cfg[1] != 0) ? 48000000 : 32000000
     *     div = get_clock_freq(2) / r4          ; 384/48 = 8, 384/32 = 12
     *     if (div <= 1) div = 2
     *     ... programs the controller with div ...
     *     clk_set_source(0x2b, 2)               ; 0x820996 -> 0x8206c2
     *
     * and FIRM's own "set the flash divider" helper at 0xc48096 is the same
     * trio of ROM calls we use here, on the same clock id. We take the
     * conservative 32 MHz branch.
     *
     * So we retimed the controller for a fast clock and then never delivered
     * one: the measured norf rate is 4 MHz. docs/CLOCKS.md's 12 MHz is not a
     * contradiction, it was measured in BOOTLOADER mode, which is a different
     * configuration from the normal-boot path we come up on. Every instruction
     * of this XIP image has been fetched at an eighth of the rate the vendor
     * runs the same flash at, which is where "~10s to first light" comes from.
     *
     * Divider BEFORE source, deliberately: raising the divider can only make
     * the clock slower whatever the current parent is, whereas switching to the
     * 384 MHz PLL first would briefly run the flash at 384/old_div.
     *
     * From .icode, like everything else that touches a clock - and this one
     * feeds our own instruction fetch. */
#if YP3_FLASH_CLOCK
    BC_SLOT(133) = ROM_CLK_FREQ(NORF_CLOCK);      /* before */
    ROM_CLK_DIV2(NORF_CLOCK, 12);
    ROM_CLK_SRC2(NORF_CLOCK, 2);
    ROM_CLK_APPLY2(NORF_CLOCK);
    for (d = 0; d < 100000u; d++) ;
    /* Read a word of flash back from SRAM. If the flash cannot keep up at the
     * new rate this is the last thing that works, and slot 135 stays 0 while
     * 134 holds whatever came off the bus. 0xc0e000 is our own vector table. */
    BC_SLOT(134) = *(volatile uint32_t *)0x00c0e000u;
    BC_SLOT(135) = ROM_CLK_FREQ(NORF_CLOCK);      /* after */
    BC_SLOT(126) = 0xC10C100Eu;
#endif

    /* Clock census. Every "expected" figure this port has been comparing
     * against came from docs/CLOCKS.md, which was measured in bootloader mode -
     * i.e. before the vendor's own clock init has run - so half of them are the
     * wrong baseline. These are the ids the tree actually turns on:
     *
     *   3, 4, 5   the clocks 0x8206e4 sets dividers 2, 1, 1 on
     *   0x29      the parent of sources 8 and 9: ROM 0x3850 answers id 8 with
     *             clk(0x29) >> 1 and id 9 with clk(0x29) >> 2, so this is the
     *             number that decides the LCD pixel clock, and something in
     *             our init halves it (source 8 was 192 MHz before, 96 after)
     *   0x2a      cpu pll, for reference
     *
     * udelay assumes 192 MHz. If clock 3 reads 192 that assumption is right and
     * the "core pll 384" the dump complains about is just the PLL, correctly
     * programmed - 0x8206e4 is called as f(2, 384000000, 32000000) at BOTH of
     * its call sites, so 384 is exactly what the vendor asks for. */
    BC_SLOT(128) = ROM_CLK_FREQ(3);
    BC_SLOT(129) = ROM_CLK_FREQ(4);
    BC_SLOT(130) = ROM_CLK_FREQ(5);
    BC_SLOT(131) = ROM_CLK_FREQ(0x29);
    BC_SLOT(132) = ROM_CLK_FREQ(0x2a);
}
#endif

void system_init(void)
{
    BC(3);

    /* Clock sanity BEFORE any driver touches anything.
     *
     * Every ROM clock query came back 0 in the XIP build while the SRAM build
     * reported 24MHz and 192MHz. If they are already 0 here, the clock tree was
     * broken before the LCD was involved - which would point at the window write
     * in crt0 having side effects beyond opening the aperture. If they are sane
     * here and 0 later, something between here and lcdc_hw_init breaks them.
     *
     * ROM 0x3850 recurses (bl 0x35dc then bl 0x3850), so a corrupt tree can also
     * spin it forever rather than returning a wrong answer. */
    /* Build id, so a dump always states WHICH build produced it.
     * Derived from __TIME__/__DATE__ at compile time; 29_read_breadcrumbs.sh
     * pulls the expected value straight out of the ELF and compares. Two
     * rounds of display testing were spent on observations that may have come
     * from a stale image - never again. */
    BC_SLOT(125) = yp3_build_id;

#if YP3_CLOCK_INIT
    yp3_clock_init();
    /* rates after, in the same slots the earlier probe used */
    BC_SLOT(120) = ROM_CLK_FREQ(0x2a);
    BC_SLOT(121) = ROM_CLK_FREQ(0x02);
    BC_SLOT(122) = ROM_CLK_FREQ(0x06);
    BC_SLOT(123) = ROM_CLK_FREQ(0x2b);
    BC_SLOT(124) = ROM_CLK_FREQ(0x3b);
#endif

    /* The watchdog goes off early: the ROM has left it at 0 on every dump so
     * far, but a reset mid-init reads exactly like a driver hang. */
    WDOG_CTRL = 0;

    BC_SLOT(57) = ROM_CLK_FREQ(8);       /* expect 192000000 */
    BC_SLOT(58) = ROM_CLK_FREQ(0x3f);    /* expect  24000000 */
    BC_SLOT(59) = 0x5A4E0000u;           /* both queries returned at all */

    /* Disable the watchdog FIRST.
     *
     * The boot ROM leaves a watchdog running at 0x40083000 and the vendor
     * firmware services it via /dev/wdog ("watch_watchdog_refresh"). Rockbox
     * never does, so it reset us on a fixed timer no matter what we were doing.
     * That produced an off -> screen -> off loop, and because crt0 wipes the
     * breadcrumbs on every boot, each dump showed progress stopping at a
     * DIFFERENT place - which looked like a series of unrelated driver bugs.
     *
     * ROM 0xb7e4 disables it with a single store; done inline to avoid
     * depending on the ROM this early.
     */
    BC_SLOT(44) = WDOG_CTRL;      /* what the ROM left it set to */
    BC_SLOT(45) = WDOG_LOAD;
    WDOG_CTRL = 0;

    /* (the old flash-window probe loop lived here; it overwrote slots 57-63
     * after the clock probes above had written them, and fabricated a
     * "clock queries did not return" reading) */
    /* No device drivers are written yet, so nothing should be delivering
     * interrupts. Mask them all before anything can fire. */
    yp3_mask_all_irqs();
    /* Clocks are already configured by the boot ROM / bootloader:
     * cpu pll 384 MHz, core pll 192 MHz, ahb 32 MHz (measured). */
}

void system_reboot(void)
{
    /* AIRCR SYSRESETREQ */
    REG32(0xE000ED0C) = (0x5FAu << 16) | (1u << 2);
    while (1) ;
}

void system_exception_wait(void) { while (1) ; }

/* MUST run from SRAM (.icode).
 *
 * A calibrated delay loop executing from XIP flash is not calibrated at all:
 * every iteration waits on a SPI fetch, so it runs many times slower than the
 * constant assumes. lcd_panel_reset plus the 120ms SLPOUT wait come to over four
 * million iterations, which from flash takes tens of seconds - long enough that
 * the boot looks hung when it is merely slow. */
void __attribute__((section(".icode"))) udelay(uint32_t us)
{
    /* Crude busy loop calibrated for a 192 MHz core at ~8 cycles an iteration.
     * That is clock 3 (the 384 MHz PLL over the divider of 2 that 0x8206e4
     * programs), NOT clock 2, which is the PLL itself and reads 384. Slot 128
     * measures clock 3 so this constant stops being an assumption. */
    volatile uint32_t n = us * 24;
    while (n--) ;
}

int system_memory_guard(int newmode) { (void)newmode; return 0; }

/* ---- Panic capture -------------------------------------------------------
 *
 * panicf() already prints the message, pc, sp and an unwound backtrace, but it
 * prints them to a 128x160 panel: 20 characters a line, and the message alone
 * wraps over several of them, so the frames that say WHO ran out of whatever
 * scroll off the bottom. The screen that prompted this showed
 * "file_cache_alloc - OOM" and exactly one frame.
 *
 * Everything panicf() produces is therefore also written here, into breadcrumb
 * slots that survive the pinhole reset, and tools/read_breadcrumbs.py
 * symbolises the addresses against the ELF. Slots 200..252, one owner
 * (breadcrumb.h).
 *
 *   200  magic 0x50414E31 'PAN1' - a panic was logged
 *   201  pc          202  sp          203  frames seen (may exceed 16)
 *   204..219  the first 16 frame return addresses, innermost first
 *   220..251  the 128-byte panic message, as raw little-endian chars
 *   252  panic counter: >1 means a panic panicked, and 201.. are the LAST one
 *
 * These stores must not be optimised away or reordered - BC_SLOT is volatile
 * for that reason - and this runs before panicf touches the LCD, so a panic in
 * a broken display path still leaves a full record.
 */
#define BC_PANIC_BASE   200
#define BC_PANIC_FRAMES 16

void panic_log_target(const char *msg, uint32_t pc, uint32_t sp);
void panic_log_frame(uint32_t addr);

static unsigned panic_frame_count;

void panic_log_target(const char *msg, uint32_t pc, uint32_t sp)
{
    BC_SLOT(252) = BC_SLOT(252) + 1;
    BC_SLOT(BC_PANIC_BASE + 1) = pc;
    BC_SLOT(BC_PANIC_BASE + 2) = sp;
    BC_SLOT(BC_PANIC_BASE + 3) = 0;
    panic_frame_count = 0;

    for (unsigned i = 0; i < BC_PANIC_FRAMES; i++)
        BC_SLOT(BC_PANIC_BASE + 4 + i) = 0;

    /* pack the message four chars to a slot; stop at the terminator but always
     * clear the rest, so a second panic cannot leave a longer first message
     * trailing off the end of a shorter one */
    unsigned done = 0;
    for (unsigned w = 0; w < 32; w++)
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; b++)
        {
            unsigned char c = 0;
            if (!done)
            {
                c = (unsigned char)msg[w * 4 + b];
                if (!c)
                    done = 1;
            }
            word |= (uint32_t)c << (8 * b);
        }
        BC_SLOT(220 + w) = word;
    }

    /* written last: the reader treats the magic as "everything above is here" */
    BC_SLOT(BC_PANIC_BASE) = 0x50414E31u;
}

void panic_log_frame(uint32_t addr)
{
    if (panic_frame_count < BC_PANIC_FRAMES)
        BC_SLOT(BC_PANIC_BASE + 4 + panic_frame_count) = addr;
    panic_frame_count++;
    BC_SLOT(BC_PANIC_BASE + 3) = panic_frame_count;
}
