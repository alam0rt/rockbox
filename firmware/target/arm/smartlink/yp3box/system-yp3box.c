#include "system.h"
#include "blackbox.h"
#include "system-target.h"
#include "sl6801-regs.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"
/* Configure the clock tree used by the XIP image. This sequence is transcribed
 * from the vendor bootloader and must run from SRAM. */

#define ROM_CLK_SRC2  ((void (*)(unsigned, unsigned))0x3119u)
#define ROM_CLK_DIV2  ((void (*)(unsigned, unsigned))0x2d59u)
#define ROM_CLK_APPLY2 ((void (*)(unsigned))0x27a1u)
#define NORF_CLOCK    0x2bu
#define ROM_4C014     ((void (*)(unsigned))0x4c015u)
#define ROM_3904      ((void (*)(unsigned, unsigned))0x3905u)
#define ROM_2A7C      ((void (*)(unsigned))0x2a7du)
/* docs/ROM-API.md; also used by sd-yp3box.c. Computes from the dividers and
 * does NOT look at the gate, so it reports configuration, not motion. */
#define ROM_CLK_FREQ  ((uint32_t (*)(unsigned))0x3851u)
#define CLK2(o)       (*(volatile uint32_t *)(0x40085000u + (o)))
#define FLASHC(o)     (*(volatile uint32_t *)(0x40000000u + (o)))

#define YP3_CLOCK_INIT 1
/* Set to 0 if the flash cannot take 32 MHz. */
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

/* The reference block, and the fastest divider that reproduced it. .bss is
 * SRAM, so reading it never touches the link under test. */
#define FLASH_REF_WORDS 128
static uint32_t yp3_flash_ref[FLASH_REF_WORDS];
static void __attribute__((section(".icode"), noinline))
yp3_flash_set_div(unsigned div)
{
    volatile uint32_t d;

    ROM_CLK_DIV2(NORF_CLOCK, div);
    ROM_CLK_SRC2(NORF_CLOCK, 2);
    ROM_CLK_APPLY2(NORF_CLOCK);
    for (d = 0; d < 20000u; d++) ;
}

/* Read the reference window and say whether it still matches. Open-coded:
 * memcmp lives in flash, and calling it here would be a flash fetch in the
 * middle of deciding whether flash works. */
static bool __attribute__((section(".icode"), noinline))
yp3_flash_verify(void)
{
    const volatile uint32_t *p = (const volatile uint32_t *)0x00c00000u;
    unsigned i;

    for (i = 0; i < FLASH_REF_WORDS; i++)
        if (p[i] != yp3_flash_ref[i])
            return false;
    return true;
}

/* OFF BY DEFAULT. It stopped the device booting, and the reason is the rule
 * this file already states twice: while the flash clock is in doubt, NOTHING
 * may be fetched from flash. Two things in the first version were:
 *
 *   - logf(), which lives in flash. It was called immediately after raising
 *     the clock, so a divider the part could not take crashed inside the very
 *     call meant to report it.
 *   - `static const unsigned divs[]`, which is .rodata and therefore flash.
 *     Reading divs[j] at an unverified clock returns garbage, and the garbage
 *     is then written to the divider. That is also where the impossible
 *     "divider 0 held" came from - it was never a reporting bug.
 *
 * Both are fixed below: the table is built on the stack (SRAM) and nothing is
 * logged until the clock is back to a known-good value and the caller is
 * running normally again. The per-rung results go into .bss for
 * yp3_measure_speed to print afterwards.
 *
 * It stays off until someone deliberately turns it on, because the failure
 * mode is a device that will not boot and cannot say why - the single most
 * expensive outcome available. Recovery is ./tools/27_install_rockbox.sh in
 * bootloader mode, which uses the ROM's own flash settings and is unaffected. */
#define YP3_FLASH_LADDER 0

#define FLASH_RUNGS 4
uint32_t yp3_flash_div;                 /* reported by yp3_measure_speed */
uint32_t yp3_flash_rung_div[FLASH_RUNGS];
uint32_t yp3_flash_rung_hz[FLASH_RUNGS];
uint8_t  yp3_flash_rung_ok[FLASH_RUNGS];
unsigned yp3_flash_rungs;

static void __attribute__((section(".icode"), noinline, unused))
yp3_flash_clock_ladder(void)
{
    /* On the stack, not in .rodata: every byte this function reads while the
     * clock is uncertain has to come from SRAM. */
    unsigned divs[FLASH_RUNGS];
    const volatile uint32_t *p = (const volatile uint32_t *)0x00c00000u;
    unsigned i, j;

    divs[0] = 4u; divs[1] = 6u; divs[2] = 8u; divs[3] = 12u;

    yp3_flash_set_div(12u);
    for (i = 0; i < FLASH_REF_WORDS; i++)
        yp3_flash_ref[i] = p[i];

    yp3_flash_rungs = 0;
    for (j = 0; j < FLASH_RUNGS; j++) {
        bool ok;

        yp3_flash_set_div(divs[j]);
        ok = yp3_flash_verify();
        /* ROM, not flash - safe to call here. */
        yp3_flash_rung_div[j] = divs[j];
        yp3_flash_rung_hz[j]  = ROM_CLK_FREQ(NORF_CLOCK);
        yp3_flash_rung_ok[j]  = ok ? 1u : 0u;
        yp3_flash_rungs = j + 1u;
        if (ok) {
            yp3_flash_div = divs[j];
            return;
        }
    }

    yp3_flash_set_div(12u);
    yp3_flash_div = 12u;
}

static void __attribute__((section(".icode"), noinline)) yp3_clock_init(void)
{
    volatile uint32_t d;

    CLK2(0x60) = 0;
    ROM_CLK_SRC2(3, 10);
    ROM_CLK_SRC2(6, 10);
    ROM_4C014(6);

    yp3_flash_timing();

    ROM_3904(2, 384000000u);
    ROM_2A7C(8);
    ROM_2A7C(9);

    CLK2(0xd8) &= ~2u;
    ROM_CLK_DIV2(3, 2);
    ROM_CLK_DIV2(4, 1);
    ROM_CLK_DIV2(5, 1);
    ROM_CLK_DIV2(6, 384000000u / 32000000u);
    ROM_4C014(6);

    ROM_CLK_SRC2(3, 2);
    ROM_CLK_SRC2(6, 2);
    for (d = 0; d < 100000u; d++) ;

    /* Configure the SPI-NOR clock after the PLL is ready.
     *
     * This is an XIP build with no instruction cache, so EVERY instruction
     * outside .icode is fetched over this link and its rate is a multiplier on
     * the whole machine - the UI, the SD drain, interrupt latency, decode.
     * 384/12 = 32 MHz was a guess, and the comment on YP3_FLASH_CLOCK admits
     * it. The vendor's own divider setter (FIRM 0xc48096, via the clamp at
     * 0xc480bc) accepts anything down to 2, and the vendor executes from this
     * same flash, so 32 MHz is a conservative corner rather than a limit.
     *
     * Raising it is the one change that can stop the device booting with no
     * log: if the part cannot keep up, the next instruction fetch is garbage
     * and there is nothing left running to report it. So it is a ladder, and
     * it verifies BY CONTENT, exactly like the SD one:
     *
     *   - capture a block of flash at the known-good divider into SRAM
     *   - raise the clock, re-read the same block, compare
     *   - keep the fastest divider that reads back identical, else fall back
     *
     * Every step runs from .icode with the reference in .bss, so no flash
     * fetch is needed while the clock is in doubt - the code doing the
     * checking cannot be the code being broken. That is what makes this safe
     * to try rather than something to guess at. */
#if YP3_FLASH_CLOCK
#if YP3_FLASH_LADDER
    yp3_flash_clock_ladder();
#else
    /* The divider this port has always booted at. 384/12 = 32 MHz. */
    yp3_flash_set_div(12u);
    yp3_flash_div = 12u;
#endif
    for (d = 0; d < 100000u; d++) ;
#endif

    /* The clock tree is now configured for the 192 MHz core clock used by
     * udelay(). */
}
#endif

/* A tight loop, once in flash and once in SRAM, timed against SysTick.
 *
 * Everything on this device is slow in a way that is not specific to any one
 * driver: the UI takes seconds to notice a USB disconnect, buffering a 1.4 MB
 * file takes ten, and the black box's own `t=` beacon advances about a third
 * as fast as a wall clock. That pattern is not a storage bug or an audio bug,
 * it is the whole machine, and there are exactly two candidates.
 *
 * ONE: the core clock is not what CPU_FREQ says. tick_start programs SysTick
 * with (CPU_FREQ / 1000) * interval, so if the core runs slower than the
 * declared 192 MHz then every kernel tick stretches by the same ratio and
 * every timeout, poll and debounce in Rockbox stretches with it. The ROM's own
 * get_clock_freq is asked directly rather than trusted from a doc - and note
 * it computes from dividers and does not look at the gate, so it proves the
 * configuration, not that the clock is running.
 *
 * TWO: this is an XIP build. Cortex-M4 has no instruction cache, so EVERY
 * instruction outside .icode is fetched over SPI NOR, whose clock this port
 * sets to 384/12 = 32 MHz. A 192 MHz core fed from a 32 MHz serial link is
 * starved, and CLAUDE.md has already observed the symptom three times without
 * naming the cause: "executed from flash they stall the CPU or crawl".
 *
 * The ratio between the two loops is the XIP penalty, in one number, with no
 * host tooling and no interpretation. SysTick is used bare here - counting
 * down at the core clock with no interrupt - because this runs before
 * tick_start, which reprograms it afterwards. */
#define SYSTICK_CSR REG32(0xE000E010)
#define SYSTICK_RVR REG32(0xE000E014)
#define SYSTICK_CVR REG32(0xE000E018)

#define SPIN_BODY(n) do { \
        uint32_t i = (n); \
        while (i--) __asm__ volatile ("" ::: "memory"); \
    } while (0)

static void __attribute__((noinline)) yp3_spin_flash(uint32_t n)
{
    SPIN_BODY(n);
}

static void __attribute__((section(".icode"), noinline))
yp3_spin_sram(uint32_t n)
{
    SPIN_BODY(n);
}

static uint32_t yp3_time_spin(void (*fn)(uint32_t), uint32_t n)
{
    uint32_t a, b;

    SYSTICK_CSR = 0;
    SYSTICK_RVR = 0x00ffffffu;
    SYSTICK_CVR = 0;
    SYSTICK_CSR = 5u;                   /* core clock, no interrupt, enable */
    a = SYSTICK_CVR;
    fn(n);
    b = SYSTICK_CVR;
    SYSTICK_CSR = 0;
    return (a - b) & 0x00ffffffu;       /* SysTick counts DOWN */
}

static void yp3_measure_speed(void)
{
    static const unsigned ids[] = { 2u, 3u, 6u, 0x2bu };
    const char *names[] = { "pll", "core", "ahb", "norf" };
    uint32_t flash_cyc, sram_cyc;
    unsigned i;

    for (i = 0; i < 4; i++)
        logf("clk %s(%#x) = %lu Hz", names[i], ids[i],
             (unsigned long)ROM_CLK_FREQ(ids[i]));
    logf("flash: divider %lu held (12 = the 32 MHz floor)",
         (unsigned long)yp3_flash_div);
    for (i = 0; i < yp3_flash_rungs; i++)
        logf("flash: div=%lu -> %lu Hz %s",
             (unsigned long)yp3_flash_rung_div[i],
             (unsigned long)yp3_flash_rung_hz[i],
             yp3_flash_rung_ok[i] ? "OK" : "bad");

    /* 20000 iterations is long enough to swamp the call overhead and short
     * enough not to overflow SysTick's 24 bits even at a heavy XIP penalty. */
    sram_cyc  = yp3_time_spin(yp3_spin_sram, 20000u);
    flash_cyc = yp3_time_spin(yp3_spin_flash, 20000u);
    logf("speed: sram=%lu cyc flash=%lu cyc for 20000 iters",
         (unsigned long)sram_cyc, (unsigned long)flash_cyc);
    if (sram_cyc)
        logf("speed: XIP penalty = %lu.%02lux",
             (unsigned long)(flash_cyc / sram_cyc),
             (unsigned long)((flash_cyc % sram_cyc) * 100u / sram_cyc));
}

void system_init(void)
{

    /* Before anything logs: decide whether the black box holds a record from
     * the last run, or is cold and needs clearing. */
    blackbox_init();

    /* Configure the clocks before any device driver accesses hardware. */

#if YP3_CLOCK_INIT
    yp3_clock_init();
#endif



    /* Two numbers, both cheap, both read-only, and between them they settle
     * whether this device is actually running at the speed this port assumes.
     * See yp3_measure_speed. */
    yp3_measure_speed();

    /* The boot ROM leaves a watchdog running and Rockbox does not refresh it.
     * Disable it before any driver starts. */
    WDOG_CTRL = 0;

    /* Mask any pending device interrupts before drivers run. */
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

/* panicf() ends here. It has already painted the panel; write the black box
 * to the card as well, because the panel cannot be scrolled, photographed
 * accurately at 128x160, or read at all if the panic came from the display
 * path. If the write itself wedges - a panic while the filesystem lock is
 * held would do it - the record is still in SRAM, and a reset recovers it. */
void system_exception_wait(void)
{
    blackbox_dump();
    while (1) ;
}

/* MUST run from SRAM (.icode). A calibrated delay loop executing from XIP flash
 * is not calibrated because every iteration waits on a SPI fetch. */
void __attribute__((section(".icode"))) udelay(uint32_t us)
{
    /* Clock 3 is the 384 MHz PLL divided by two, i.e. the 192 MHz core
     * clock. */
    volatile uint32_t n = us * 24;
    while (n--) ;
}

int system_memory_guard(int newmode) { (void)newmode; return 0; }
