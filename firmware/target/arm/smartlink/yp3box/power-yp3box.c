/*
 * Power control.
 *
 * The PMU is not on I2C: it is reached through a mailbox in the clock
 * controller at 0x40085000, decoded from the vendor's HAL helpers at 0x804f28
 * (write) and 0x804f90 (read):
 *
 *     +0x104   command: 0x40000060 | reg << 8   write
 *                       0x40000061 | reg << 8   read
 *              bit 31 set to start, self-clearing when done
 *     +0x108   write data
 *     +0x10c   read data
 *     +0x110   error status, bits 12-15
 *
 * Those helpers live in the SRAM HAL the vendor bootloader loads, which our
 * build overwrites, so they are reproduced here rather than called - the same
 * situation as set_pll in pcm-yp3box.c. The ROM entry points we DO call are at
 * 0x0-0x5c000 and are mask ROM; they are always there.
 */
#include "config.h"
#include "power.h"
#include "system.h"
#include "backlight.h"
#include "backlight-target.h"
#include "sl6801.h"

#define CLK(o) (*(volatile uint32_t *)(CLKCTRL_BASE + (o)))

#define PMU_CMD     0x104
#define PMU_WDATA   0x108
#define PMU_RDATA   0x10c
#define PMU_STATUS  0x110

#define PMU_OP_WRITE    0x40000060u
#define PMU_OP_READ     0x40000061u
#define PMU_GO          0x80000000u

static uint8_t __attribute__((section(".icode"), noinline))
pmu_read(unsigned reg)
{
    CLK(PMU_CMD) = PMU_OP_READ | (reg << 8);
    CLK(PMU_CMD) |= PMU_GO;
    while (CLK(PMU_CMD) & PMU_GO) ;
    return (uint8_t)CLK(PMU_RDATA);
}

static void __attribute__((section(".icode"), noinline))
pmu_write(unsigned reg, uint8_t val)
{
    CLK(PMU_WDATA) = val;
    CLK(PMU_CMD) = PMU_OP_WRITE | (reg << 8);
    CLK(PMU_CMD) |= PMU_GO;
    while (CLK(PMU_CMD) & PMU_GO) ;
}

void power_init(void)
{
}

/* Arm the PMU sleep sequencer - vendor 0xcf7fa0, the tail of "enter poweroff
 * mode".
 *
 * The rail is not switched off by a register write, because after that write
 * there is no CPU left to make it: the clock controller has a sequencer that
 * applies a PMU register value on the way into sleep and another on the way
 * out. +0xb0 and +0xb8 hold those (register number in the low byte, values in
 * the upper halves) and +0xd0 enables them.
 *
 * Transcribed with the two configuration bytes this device actually has, read
 * out of the vendor's own state at 0x0081b034: [11] = 0 selects the plain
 * branch, [12] = 1 selects the extra wake-source enable in PMU register 0.
 * A device with different bytes would need the other branches at 0xcf8016 and
 * 0xcf801c. */
static void __attribute__((section(".icode"), noinline))
yp3_pmu_arm_sleep(void)
{
    uint8_t v = pmu_read(0x21);

    CLK(0xb0) = (uint32_t)((v | 3) & 0xff) << 8 | 0x21;
    CLK(0xd0) |= 1;

    v = (v & 0xe7) | 4;                 /* the value restored on wake */
    CLK(0xb8) = ((uint32_t)v << 16) | 0x21
              | ((uint32_t)(v & 0xfb) << 24);   /* applied on the way down */

    pmu_write(0, (uint8_t)(pmu_read(0) | 8));
    CLK(0xd0) |= 8;
}

/* MUST run from SRAM: it stops the clock the CPU fetches instructions through.
 * The vendor runs the same sequence XIP from FIRM, but this port has paid four
 * times over for ignoring that rule and there is no reason to make this the
 * fifth. */
static void __attribute__((section(".icode"), noinline, noreturn))
yp3_power_down(void)
{
    unsigned i;

    /* stop the tick before the clock it counts goes away */
    REG32(0xE000E010) &= ~3u;                       /* vendor 0x80d278 */
    disable_irq();                                  /* vendor 0x804ec8 */

    /* peripheral clocks 0x11-0x14 and 0x16-0x20, vendor 0xcf8138 */
    for (i = 0x11; i <= 0x20; i++)
        if (i != 0x15)
            ROM_CLK_STOP(i);

    /* park the CPU, bus and flash clocks on source 10 before the PLLs go */
    ROM_CLK_SET_SRC(3, 10);
    ROM_CLK_SET_SRC(6, 10);
    ROM_CLK_SET_SRC(NORF_CLK, 10);
    udelay(100);

    ROM_CLK_DIV(3, 1);
    ROM_CLK_DIV(4, 1);
    ROM_CLK_DIV(5, 1);
    ROM_CLK_DIV(6, 1);
    udelay(100);

    ROM_CLK_STOP(0x29);
    ROM_CLK_STOP(8);
    ROM_CLK_STOP(9);
    ROM_CLK_STOP(0x26);
    udelay(100);
    ROM_CLK_STOP(2);
    ROM_CLK_STOP(0);

    CLK(0x60) = 1;                                  /* vendor 0xcf7d10 */
    CLK(0xd8) &= ~6u;

    /* vendor 0xcf7cf0(0): clear the two-bit field at PMU reg 0 bits 6-7 */
    pmu_write(0, (uint8_t)(pmu_read(0) & ~0xc0u));

    yp3_mask_all_irqs();                            /* vendor 0x80d1b4/0x80d1e0 */
    yp3_pmu_arm_sleep();

    while (1)
        asm volatile ("wfi");
}

void power_off(void)
{
    /* Rockbox does not blank the panel for us unless the target says it can,
     * and a device that "powers off" with the shutdown splash still lit looks
     * exactly like the hang this used to be. lcd_shutdown() is the upstream
     * hook for it (HAVE_LCD_SHUTDOWN); the backlight is ours to kill here. */
    backlight_hw_off();
    yp3_power_down();
}

bool charging_state(void)
{
    return false;   /* TODO: PMU status register; the mailbox above is the way in */
}

unsigned int power_input_status(void)
{
    return POWER_INPUT_NONE;   /* TODO: same */
}
