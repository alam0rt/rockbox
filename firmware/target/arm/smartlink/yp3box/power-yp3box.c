/*
 * Power control.
 *
 * The PMU is not on I2C: it is reached through a mailbox at 0x40085000,
 * decoded from the vendor's HAL helpers at 0x804f28 (write) and 0x804f90
 * (read), and confirmed against the boot ROM's own copies at 0x3c64 and
 * 0x3ca4:
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
#include "sl6801-regs.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

/* PMU_REG is 0x40085000, NOT CLKCTRL_BASE. This file used the clock-controller
 * base for the whole of its life, which put every access 0x5000 low: mailbox
 * commands went into the clock controller, so battery voltage and charger
 * status were whatever that block happened to read back, and power_off()'s
 * sleep-sequencer writes landed on clock registers. The vendor loads
 * 0x40085000 at FIRM 0xcf7d24, at 0xcf8044 and in both ROM mailbox helpers. */

#define PMU_MB_CMD     0x104
#define PMU_MB_WDATA   0x108
#define PMU_MB_RDATA   0x10c
#define PMU_MB_STATUS  0x110

#define PMU_OP_WRITE    0x40000060u
#define PMU_OP_READ     0x40000061u
#define PMU_GO          0x80000000u
#define PMU_ERROR_MASK  0x0000f000u
#define PMU_POLL_LIMIT  1000000u

/* The mailbox has one command register pair and several users: the power
 * thread reads the battery once a second, usb_detect() reads VBUS, and
 * usb_detect() is called from usb_tick() - the TICK INTERRUPT. Two
 * transactions interleaved on one register pair return each other's data, or
 * hang waiting for a GO bit the other side already cleared. The vendor takes
 * a mutex here ("creat pmu_bus_mutex fail", FIRM 0xcf71c0); a mutex is no use
 * to an interrupt, and a transaction is a handful of microseconds, so mask
 * interrupts around it instead.
 *
 * The mailbox is synchronous, but a broken PMU must not wedge the Rockbox
 * power thread or the shutdown path forever. */
bool __attribute__((section(".icode"), noinline))
yp3_pmu_read(unsigned reg, uint8_t *value)
{
    unsigned i;
    int oldlevel = disable_irq_save();
    bool ok;

    PMU_REG(PMU_MB_CMD) = PMU_OP_READ | (reg << 8);
    PMU_REG(PMU_MB_CMD) |= PMU_GO;
    for (i = 0; i < PMU_POLL_LIMIT && (PMU_REG(PMU_MB_CMD) & PMU_GO); i++)
        ;

    ok = !(PMU_REG(PMU_MB_CMD) & PMU_GO)
      && !(PMU_REG(PMU_MB_STATUS) & PMU_ERROR_MASK);
    if (ok)
        *value = (uint8_t)PMU_REG(PMU_MB_RDATA);

    restore_irq(oldlevel);
    return ok;
}

bool __attribute__((section(".icode"), noinline))
yp3_pmu_write(unsigned reg, uint8_t value)
{
    unsigned i;
    int oldlevel = disable_irq_save();
    bool ok;

    PMU_REG(PMU_MB_WDATA) = value;
    PMU_REG(PMU_MB_CMD) = PMU_OP_WRITE | (reg << 8);
    PMU_REG(PMU_MB_CMD) |= PMU_GO;
    for (i = 0; i < PMU_POLL_LIMIT && (PMU_REG(PMU_MB_CMD) & PMU_GO); i++)
        ;

    ok = !(PMU_REG(PMU_MB_CMD) & PMU_GO)
      && !(PMU_REG(PMU_MB_STATUS) & PMU_ERROR_MASK);
    restore_irq(oldlevel);
    return ok;
}

/* Enable the PMU's voltage measurement.
 *
 * Register 0x2e read back 0x80 on hardware: bit 7 set, all seven value bits
 * zero, i.e. 2800 mV - the bottom of the scale - on a fully charged device.
 * The block was not measuring.
 *
 * The vendor's pmu init at FIRM 0xcf795e reads register 0x47 and sets bit 0
 * when its config byte +1 is 1. That config is built on the stack at FIRM
 * 0xd67a4c and its first word is the literal 0x00010102 at 0xd67aa4, so byte
 * +1 IS 1 on this device and the vendor does set the bit. Bit 4 of the same
 * register is the channel select the battery read toggles (FIRM 0xcf7a28),
 * which is a select, not an enable - selecting a channel on a block that is
 * off gives exactly the zero we saw.
 *
 * The rest of that init - registers 0x13, 0x14, 0x15, 0x16, 0x17, 0x1c - is
 * charger and thermal configuration driven by the same struct, and is not
 * reproduced: none of it is on the path from the battery sense to 0x2e. If
 * 0x2e still reads zero, the logged register values below say which of them
 * to look at next. */
void power_init(void)
{
    uint8_t before = 0, after = 0;

    if (yp3_pmu_read(PMU_REG_VOLTAGE_CONFIG, &before)
            && yp3_pmu_write(PMU_REG_VOLTAGE_CONFIG, (uint8_t)(before | 1u)))
        (void)yp3_pmu_read(PMU_REG_VOLTAGE_CONFIG, &after);

    logf("pmu 0x47: %02x -> %02x", before, after);

    /* One line that characterises the block, so a single dump can rule the
     * rest of the vendor's init in or out. */
    {
        static const uint8_t regs[] = { 0x13, 0x17, 0x1c, 0x2a, 0x2e };
        unsigned i;

        for (i = 0; i < sizeof(regs); i++) {
            uint8_t v = 0;
            bool ok = yp3_pmu_read(regs[i], &v);
            logf("pmu %02x = %02x%s", regs[i], v, ok ? "" : " (failed)");
        }
    }
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
static bool __attribute__((section(".icode"), noinline))
yp3_pmu_arm_sleep(void)
{
    uint8_t v;

    if (!yp3_pmu_read(0x21, &v))
        return false;

    PMU_REG(0xb0) = (uint32_t)((v | 3) & 0xff) << 8 | 0x21;
    PMU_REG(0xd0) |= 1;

    v = (v & 0xe7) | 4;                 /* the value restored on wake */
    PMU_REG(0xb8) = ((uint32_t)v << 16) | 0x21
              | ((uint32_t)(v & 0xfb) << 24);   /* applied on the way down */

    if (!yp3_pmu_read(0, &v)
            || !yp3_pmu_write(0, (uint8_t)(v | 8)))
        return false;
    PMU_REG(0xd0) |= 8;
    return true;
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

    PMU_REG(0x60) = 1;                                  /* vendor 0xcf7d10 */
    PMU_REG(0xd8) &= ~6u;
    /* vendor 0xcf7cf0(0): clear the two-bit field at PMU reg 0 bits 6-7 */
    {
        uint8_t v;
        if (yp3_pmu_read(0, &v))
            (void)yp3_pmu_write(0, (uint8_t)(v & ~0xc0u));
    }

    yp3_mask_all_irqs();  /* vendor 0x80d1b4/0x80d1e0 */
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

#if CONFIG_CHARGING
bool charging_state(void)
{
    uint8_t status;

    return yp3_pmu_read(PMU_REG_STATUS, &status)
        && (status & PMU_STATUS_CHARGING) != 0;
}

unsigned int power_input_status(void)
{
    uint8_t status;

    if (!yp3_pmu_read(PMU_REG_STATUS, &status)
            || !(status & PMU_STATUS_POWER_INPUT))
        return POWER_INPUT_NONE;

    /* YP3 exposes one undifferentiated USB charging input. */
    return POWER_INPUT_MAIN_CHARGER;
}
#endif
