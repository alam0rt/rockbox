#include "system.h"
#include "system-target.h"
#include "sl6801-regs.h"
/* Configure the clock tree used by the XIP image. This sequence is transcribed
 * from the vendor bootloader and must run from SRAM. */

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

    /* Configure the SPI-NOR clock after the PLL is ready. */
#if YP3_FLASH_CLOCK
    ROM_CLK_DIV2(NORF_CLOCK, 12);
    ROM_CLK_SRC2(NORF_CLOCK, 2);
    ROM_CLK_APPLY2(NORF_CLOCK);
    for (d = 0; d < 100000u; d++) ;
#endif
#endif

    /* The clock tree is now configured for the 192 MHz core clock used by
     * udelay(). */
}
#endif

void system_init(void)
{

    /* Configure the clocks before any device driver accesses hardware. */

#if YP3_CLOCK_INIT
    yp3_clock_init();
#endif



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

void system_exception_wait(void) { while (1) ; }

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
