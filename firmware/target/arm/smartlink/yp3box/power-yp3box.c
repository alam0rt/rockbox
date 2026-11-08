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
/* Everything this file logs is once per boot: the power-on/power-off reasons
 * and the previous power_off()'s snapshot, two lines that say why the device
 * is awake. The PMU register census in power_init is longer and stays behind
 * YP3_LOGF_POWER. (The polled battery path that once crowded the ring is
 * powermgmt-yp3box.c, not this file.) */
#if defined(ROCKBOX_HAS_LOGF)
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
/* BASEPRI, not PRIMASK. Masking at priority 3 and below still keeps the tick
 * (SysTick, now 0xf0) and every thread off the mailbox, which is all the lock
 * is for, but lets the audio DMA completion (NVIC priority 2) through. With
 * PRIMASK the tick's PMU reads held off the DMA re-arm and the transmitter ran
 * dry for ~2.6 ms of every 20 - 150 transfers a second instead of 172, a
 * 15% stretch and a 49.5 Hz comb on every tone. Nothing at priority 0..2
 * touches the PMU. */
#define PMU_LOCK_PRIO   0x30u

static inline uint32_t pmu_lock(void)
{
    uint32_t old, v = PMU_LOCK_PRIO;
    __asm__ volatile ("mrs %0, basepri" : "=r"(old));
    __asm__ volatile ("msr basepri_max, %0" :: "r"(v) : "memory");
    return old;
}

static inline void pmu_unlock(uint32_t old)
{
    __asm__ volatile ("msr basepri, %0" :: "r"(old) : "memory");
}

/* The mailbox bus init, ROM 0x3c2c: two pin configs, a self-clearing reset
 * at bit 31 of +0x100, then +0x100 = 0x0001b082. usb_hw_enable() used to call
 * it as "the USB PHY init" because the ROM's download mode runs it before
 * USB; it is the PMU bus, and it left every later transaction at half rate -
 * 4690 of 4690 PMU ops over 1 ms, ~3.2 ms each, on a boot that came up into
 * USB mode [L]. Kept as the ROM call, since the USB path has only ever been
 * tested with it, followed by the vendor's word (sl6801-regs.h). Under the
 * lock, because the reset would otherwise cut a transaction from the tick
 * in half. */
#define ROM_PMU_BUS_INIT    ((void (*)(void))0x3c2du)

void yp3_pmu_bus_init(void)
{
    uint32_t oldlevel = pmu_lock();
    ROM_PMU_BUS_INIT();
    PMU_REG(PMU_BUS_CFG) = PMU_BUS_CFG_VENDOR;
    pmu_unlock(oldlevel);
}

/* How long a transaction really takes, in core cycles off SysTick, and how
 * many there are. Read and cleared by the PCM beacon.
 *
 * The max alone was read as "every transaction is 3.4 ms, ~14% CPU". It is
 * 3323-3324 us in EVERY one-second window [L], which looks like one kind of
 * transaction rather than an outlier - so also keep the total, which
 * register the max came from (bit 8 = write), and how many exceeded 1 ms. */
uint32_t yp3_pmu_cost_max, yp3_pmu_ops, yp3_pmu_cost_sum;
uint32_t yp3_pmu_max_reg, yp3_pmu_slow;
#define SYST_RVR_   (*(volatile uint32_t *)0xE000E014)
#define SYST_CVR_   (*(volatile uint32_t *)0xE000E018)
#define PMU_SLOW_CYCLES  (CPU_FREQ / 1000u)        /* 1 ms */

static inline __attribute__((always_inline)) uint32_t pmu_t0(void)
{
    return SYST_CVR_;
}

/* always_inline: the callers are .icode and run with the flash clock parked
 * during power-off. Out of line, this landed in flash (0xc45a94). */
static inline __attribute__((always_inline)) void pmu_t1(uint32_t t0, unsigned tag)
{
    uint32_t t1 = SYST_CVR_;
    uint32_t d = t0 >= t1 ? t0 - t1 : t0 + (SYST_RVR_ + 1u) - t1;
    yp3_pmu_ops++;
    yp3_pmu_cost_sum += d;
    if (d > PMU_SLOW_CYCLES)
        yp3_pmu_slow++;
    if (d > yp3_pmu_cost_max) {
        yp3_pmu_cost_max = d;
        yp3_pmu_max_reg = tag;
    }
}

bool __attribute__((section(".icode"), noinline))
yp3_pmu_read(unsigned reg, uint8_t *value)
{
    unsigned i;
    uint32_t oldlevel = pmu_lock();
    uint32_t t0 = pmu_t0();
    bool ok;

    PMU_REG(PMU_MB_CMD) = PMU_OP_READ | (reg << 8);
    PMU_REG(PMU_MB_CMD) |= PMU_GO;
    for (i = 0; i < PMU_POLL_LIMIT && (PMU_REG(PMU_MB_CMD) & PMU_GO); i++)
        ;

    ok = !(PMU_REG(PMU_MB_CMD) & PMU_GO)
      && !(PMU_REG(PMU_MB_STATUS) & PMU_ERROR_MASK);
    if (ok)
        *value = (uint8_t)PMU_REG(PMU_MB_RDATA);

    pmu_t1(t0, reg);
    pmu_unlock(oldlevel);
    return ok;
}

bool __attribute__((section(".icode"), noinline))
yp3_pmu_write(unsigned reg, uint8_t value)
{
    unsigned i;
    uint32_t oldlevel = pmu_lock();
    uint32_t t0 = pmu_t0();
    bool ok;

    PMU_REG(PMU_MB_WDATA) = value;
    PMU_REG(PMU_MB_CMD) = PMU_OP_WRITE | (reg << 8);
    PMU_REG(PMU_MB_CMD) |= PMU_GO;
    for (i = 0; i < PMU_POLL_LIMIT && (PMU_REG(PMU_MB_CMD) & PMU_GO); i++)
        ;

    ok = !(PMU_REG(PMU_MB_CMD) & PMU_GO)
      && !(PMU_REG(PMU_MB_STATUS) & PMU_ERROR_MASK);
    pmu_t1(t0, reg | 0x100u);
    pmu_unlock(oldlevel);
    return ok;
}

/* Power-off woke straight back up, from Shut down and from the idle timer
 * alike, "instantly" by the user's account [L]. This chip's off IS a PMU
 * sleep (see yp3_pmu_arm_sleep) and a failed one ends in a reset that keeps
 * SRAM, so the device simply boots again.
 *
 * yp3_pmu_quiesce() was the first hypothesis: the vendor never sleeps with a
 * latched PMU interrupt (init FIRM 0xcf7db4 clears all three status registers
 * and programs the enables, 0xcf7b0c; the PMU ISR 0xcf7c6c clears status on
 * every event), and Rockbox let 0x30 and 0x4a latch from boot. It DID NOT FIX
 * IT [L]: with the status clean and stock's enables, the chip still reset out
 * of the first wfi. The fix is pwr_vendor_state() below. The quiesce stays
 * because it was part of the configuration that then stayed off, and nobody
 * has shown it is unnecessary.
 *
 * Enables at power-off: only sources stock also enables. Stock's live enable
 * bytes are SRAM 0x81b034 +17/+18/+19 = 0x1a, 0x7c, 0x00 [L]. 0x31 keeps what
 * the button driver set (the key bits, a subset of stock's 0x7c), so the key
 * that wakes the device is the one it already listens to.
 *
 * The record is in .persist so the NEXT boot can log what was set. After a
 * failed power-off it survives [L]; after a real one SRAM is lost [L] and the
 * next boot says "no snapshot (cold SRAM)" - which is itself the evidence
 * that the power really went off. */
#define PWRLOG_MAGIC  0x504f4646u      /* "POFF" */
#define PMU_INT0_STOCK_ENABLE  0x1au   /* SRAM 0x81b034 +17 [L] */
#define PMU_INT2_STOCK_ENABLE  0x00u   /* SRAM 0x81b034 +19 [L] */
#define PWR_WIN_N  16

struct yp3_pwrlog {
    uint32_t magic;       /* PWRLOG_MAGIC: the rest is valid        */
    uint32_t count;       /* power_off() attempts since a cold SRAM  */
    uint8_t fresh;        /* set by power_off, cleared once logged  */
    uint8_t ok;           /* 1: every quiesce transaction succeeded */
    uint8_t stage;        /* last yp3_power_down step STARTED       */
    uint8_t r00, r21;     /* PMU reg 0 and 0x21 before arming       */
    uint8_t st[3];        /* 0x30, 0x31, 0x4a before the clear      */
    uint8_t en[3];        /* 0x19, 0x1a, 0x4b before the write      */
    uint8_t after[3];     /* 0x30, 0x31, 0x4a after the clear       */
    uint32_t wfi;         /* passes of the final wfi loop           */
    uint8_t r20;          /* PMU reg 0x20, the source of +0xbc        */
    uint32_t win[PWR_WIN_N];          /* PMU_REG window before arming */
    uint32_t arm_b0, arm_b8, arm_d0;  /* sequencer after arming     */
};
static struct yp3_pwrlog yp3_pwrlog __attribute__((section(".yp3snap")));

/* Written BEFORE each step, with a barrier, so a reset in the middle of
 * yp3_power_down leaves the number of the step it was in. SRAM survives the
 * reset that ends a failed power-off [L, 2026-11-09]. */
#define PWR_STAGE(n) do { yp3_pwrlog.stage = (n); \
                          __asm__ volatile ("dsb" ::: "memory"); } while (0)

/* The always-on words the vendor's power init fills and Rockbox does not
 * (docs/RTC.md "The 0x40085080..0xbc words are not the RTC"), plus the sleep
 * sequencer. Stock: +0x80 = 0x3ff00001, +0x84 ~ 0xcf38 (a ~53 kHz LP clock in
 * Hz [I]), +0xdc..+0xe4 computed from +0x84 at FIRM 0xcf8198, +0xbc = 0x8020
 * (0xcf7d82). Open-coded: this also runs with the flash clock parked, and a
 * table would live in .rodata, which is flash. Keep in step with PWR_WIN_N
 * and the log format in yp3_power_reasons. */
static void __attribute__((section(".icode"), noinline))
pwr_window(uint32_t *w)
{
    w[0]  = PMU_REG(0x60);  w[1]  = PMU_REG(0x74);  w[2]  = PMU_REG(0x80);
    w[3]  = PMU_REG(0x84);  w[4]  = PMU_REG(0x88);  w[5]  = PMU_REG(0x8c);
    w[6]  = PMU_REG(0xa4);  w[7]  = PMU_REG(0xac);  w[8]  = PMU_REG(0xb0);
    w[9]  = PMU_REG(0xb8);  w[10] = PMU_REG(0xbc);  w[11] = PMU_REG(0xd0);
    w[12] = PMU_REG(0xd8);  w[13] = PMU_REG(0xdc);  w[14] = PMU_REG(0xe0);
    w[15] = PMU_REG(0xe4);
}

static void pwr_log_window(const char *tag, const uint32_t *w)
{
    logf("%s 60=%08lx 74=%08lx 80=%08lx 84=%08lx", tag,
         (unsigned long)w[0], (unsigned long)w[1],
         (unsigned long)w[2], (unsigned long)w[3]);
    logf("%s 88=%08lx 8c=%08lx a4=%08lx ac=%08lx", tag,
         (unsigned long)w[4], (unsigned long)w[5],
         (unsigned long)w[6], (unsigned long)w[7]);
    logf("%s b0=%08lx b8=%08lx bc=%08lx d0=%08lx", tag,
         (unsigned long)w[8], (unsigned long)w[9],
         (unsigned long)w[10], (unsigned long)w[11]);
    logf("%s d8=%08lx dc=%08lx e0=%08lx e4=%08lx", tag,
         (unsigned long)w[12], (unsigned long)w[13],
         (unsigned long)w[14], (unsigned long)w[15]);
}

/* First thing in yp3_power_down: make the record valid before any step can
 * reset the chip. */
static void __attribute__((section(".icode"), noinline))
pwr_log_begin(void)
{
    struct yp3_pwrlog *p = &yp3_pwrlog;

    if (p->magic != PWRLOG_MAGIC)
        p->count = 0;
    p->count++;
    p->fresh = 1;
    p->ok = 0;
    p->wfi = 0;
    p->stage = 1;
    p->magic = PWRLOG_MAGIC;
    __asm__ volatile ("dsb" ::: "memory");
}

/* The vendor always-on state a power-off needs, found by a ladder on
 * 2026-11-09. Without it every power-off reached the final wfi (stage 13,
 * wfi 1 [L]) and the chip reset straight out of the sleep. Each Shut down then
 * applied one more piece of the vendor's state, cumulatively:
 *
 *   rung 1  vendor power-off step 0x80d66c, which this driver used to skip:
 *           +0x80 bit 31 cleared, then +0x80 = 0xbff00001 from the struct at
 *           SRAM 0x81b060 (01 .. 00 .. 3ff .. 00 01) [C]. It starts the LP
 *           clock measurement: +0x84 read 0xcf27 on the next boot, against
 *           stock's 0xcf38 and Rockbox's untouched 0x8000 [L].
 *           STILL WOKE AT ONCE [L].
 *   rung 2  + vendor power init 0xcf7d28 with this device's config (SRAM
 *           0x81b034 +0 = 0x30, +6..+9 = 01 01 01 02): +0xac = 0x30, +0xa4
 *           bits 1, 0, 4 set and 6:5 = 2 -> 0x53, = stock [C][L]; and +0xbc
 *           as 0xcf806e builds it, (PMU reg 0x20 | 0x80) << 8 | 0x20, stock
 *           0xaf20 [C][L].
 *           STAYED OFF [L]: the user saw it, and the next boot found the
 *           black box cold, which no failed power-off had done.
 *
 * Both are applied, because that pair is what was tested. Whether rung 2
 * alone would do is [U], and so is which of its three words matters. The
 * ladder's rung 3 (sequencer timings from +0x84, FIRM 0xcf8198) was never
 * reached and is not applied.
 *
 * The vendor writes the rung-2 words once at boot; this writes them at
 * power-off only, which is where they were proven, so the running system is
 * unchanged. */
static void __attribute__((section(".icode"), noinline))
pwr_vendor_state(void)
{
    struct yp3_pwrlog *p = &yp3_pwrlog;
    uint8_t v = 0;

    PMU_REG(0x80) &= ~0x80000000u;                  /* 0x80d670..0x80d678 */
    PMU_REG(0x80) = 0xbff00001u;                    /* 0x80d696 */

    PMU_REG(0xac) = 0x30;                           /* 0xcf7d2e */
    PMU_REG(0xa4) |= 0x13;                          /* 0xcf7d3e/4e/60 */
    PMU_REG(0xa4) = (PMU_REG(0xa4) & ~0x60u) | (2u << 5);   /* 0xcf7d7e */
    (void)yp3_pmu_read(0x20, &v);                   /* 0xcf8070 */
    p->r20 = v;
    PMU_REG(0xbc) = ((uint32_t)(uint8_t)(v | 0x80) << 8) | 0x20;
}

/* Runs with the clocks collapsed: no tables in .rodata, which is flash. */
static inline bool __attribute__((always_inline))
pwr_rd(unsigned reg, uint8_t *dst)
{
    uint8_t v = 0;
    bool ok = yp3_pmu_read(reg, &v);
    *dst = v;
    return ok;
}

static void __attribute__((section(".icode"), noinline))
yp3_pmu_quiesce(void)
{
    struct yp3_pwrlog *p = &yp3_pwrlog;
    bool ok = true;

    ok &= pwr_rd(PMU_REG_INT0_STATUS, &p->st[0]);
    ok &= pwr_rd(PMU_REG_KEY_STATUS,  &p->st[1]);
    ok &= pwr_rd(PMU_REG_INT2_STATUS, &p->st[2]);
    ok &= pwr_rd(PMU_REG_INT0_ENABLE, &p->en[0]);
    ok &= pwr_rd(PMU_REG_KEY_ENABLE,  &p->en[1]);
    ok &= pwr_rd(PMU_REG_INT2_ENABLE, &p->en[2]);

    /* enables hold the inverted mask, FIRM 0xcf7b18 */
    ok &= yp3_pmu_write(PMU_REG_INT0_ENABLE, (uint8_t)~PMU_INT0_STOCK_ENABLE);
    ok &= yp3_pmu_write(PMU_REG_INT2_ENABLE, (uint8_t)~PMU_INT2_STOCK_ENABLE);

    /* write-1-to-clear, FIRM 0xcf7c88..0xcf7c9a */
    ok &= yp3_pmu_write(PMU_REG_INT0_STATUS, p->st[0]);
    ok &= yp3_pmu_write(PMU_REG_KEY_STATUS,  p->st[1]);
    ok &= yp3_pmu_write(PMU_REG_INT2_STATUS, p->st[2]);

    ok &= pwr_rd(PMU_REG_INT0_STATUS, &p->after[0]);
    ok &= pwr_rd(PMU_REG_KEY_STATUS,  &p->after[1]);
    ok &= pwr_rd(PMU_REG_INT2_STATUS, &p->after[2]);
    ok &= pwr_rd(0x00, &p->r00);
    ok &= pwr_rd(0x21, &p->r21);

    p->ok = ok;
}

/* Once per boot, before anything else touches the PMU: why it is awake, and
 * what the last power_off() saw. Then clear the off reason as the vendor init
 * does (FIRM 0xcf7e2e). */
static void yp3_power_reasons(void)
{
    struct yp3_pwrlog *p = &yp3_pwrlog;
    uint32_t w[PWR_WIN_N];
    uint8_t on0 = 0, on1 = 0, off = 0;
    bool ok = yp3_pmu_read(PMU_REG_PON_REASON0, &on0)
           && yp3_pmu_read(PMU_REG_PON_REASON1, &on1)
           && yp3_pmu_read(PMU_REG_POFF_REASON, &off);

    logf("pmu pon %02x-%02x poff %02x%s", on0, on1, off, ok ? "" : " (failed)");
    if (ok)
        (void)yp3_pmu_write(PMU_REG_POFF_REASON, off);

    pwr_window(w);
    pwr_log_window("pwin boot", w);

    if (p->magic != PWRLOG_MAGIC) {
        logf("poff: no snapshot (cold SRAM)");
    } else if (!p->fresh) {
        logf("poff: none since last boot (#%lu)", (unsigned long)p->count);
    } else {
        logf("poff#%lu stage %u wfi %lu ok %u r20 %02x",
             (unsigned long)p->count, p->stage,
             (unsigned long)p->wfi, p->ok, p->r20);
        logf("poff st %02x %02x %02x en %02x %02x %02x after %02x %02x %02x",
             p->st[0], p->st[1], p->st[2], p->en[0], p->en[1], p->en[2],
             p->after[0], p->after[1], p->after[2]);
        logf("poff r00 %02x r21 %02x arm b0=%08lx b8=%08lx d0=%08lx",
             p->r00, p->r21, (unsigned long)p->arm_b0,
             (unsigned long)p->arm_b8, (unsigned long)p->arm_d0);
        pwr_log_window("pwin off", p->win);
        p->fresh = 0;
    }
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

    yp3_power_reasons();

    if (yp3_pmu_read(PMU_REG_VOLTAGE_CONFIG, &before)
            && yp3_pmu_write(PMU_REG_VOLTAGE_CONFIG, (uint8_t)(before | 1u)))
        (void)yp3_pmu_read(PMU_REG_VOLTAGE_CONFIG, &after);

#ifdef YP3_LOGF_POWER
    logf("pmu 0x47: %02x -> %02x", before, after);
    /* One line that characterises the block, so a single dump can rule the
     * rest of the vendor's init in or out. Exists only to feed the log. */
    {
        static const uint8_t regs[] = { 0x13, 0x17, 0x1c, 0x2a, 0x2e };
        unsigned i;

        for (i = 0; i < sizeof(regs); i++) {
            uint8_t v = 0;
            bool ok = yp3_pmu_read(regs[i], &v);
            logf("pmu %02x = %02x%s", regs[i], v, ok ? "" : " (failed)");
        }
    }
#endif
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

    pwr_log_begin();                                /* stage 1 */

    /* stop the tick before the clock it counts goes away */
    PWR_STAGE(2);
    REG32(0xE000E010) &= ~3u;                       /* vendor 0x80d278 */
    disable_irq();                                  /* vendor 0x804ec8 */

    /* peripheral clocks 0x11-0x14 and 0x16-0x20, vendor 0xcf8138 */
    PWR_STAGE(3);
    for (i = 0x11; i <= 0x20; i++)
        if (i != 0x15)
            ROM_CLK_STOP(i);

    /* park the CPU, bus and flash clocks on source 10 before the PLLs go */
    PWR_STAGE(4);
    ROM_CLK_SET_SRC(3, 10);
    ROM_CLK_SET_SRC(6, 10);
    ROM_CLK_SET_SRC(NORF_CLK, 10);
    udelay(100);

    PWR_STAGE(5);
    ROM_CLK_DIV(3, 1);
    ROM_CLK_DIV(4, 1);
    ROM_CLK_DIV(5, 1);
    ROM_CLK_DIV(6, 1);
    udelay(100);

    PWR_STAGE(6);
    ROM_CLK_STOP(0x29);
    ROM_CLK_STOP(8);
    ROM_CLK_STOP(9);
    ROM_CLK_STOP(0x26);
    udelay(100);
    PWR_STAGE(7);
    ROM_CLK_STOP(2);
    ROM_CLK_STOP(0);

    PWR_STAGE(8);
    PMU_REG(0x60) = 1;                                  /* vendor 0xcf7d10 */
    PMU_REG(0xd8) &= ~6u;
    /* vendor 0xcf7cf0(0): clear the two-bit field at PMU reg 0 bits 6-7 */
    PWR_STAGE(9);
    {
        uint8_t v;
        if (yp3_pmu_read(0, &v))
            (void)yp3_pmu_write(0, (uint8_t)(v & ~0xc0u));
    }

    PWR_STAGE(10);
    yp3_mask_all_irqs();  /* vendor 0x80d1b4/0x80d1e0 */
    PWR_STAGE(14);
    pwr_vendor_state();
    pwr_window(yp3_pwrlog.win);
    PWR_STAGE(11);
    yp3_pmu_quiesce();    /* NOT vendor here: see the block above it */
    PWR_STAGE(12);
    yp3_pmu_arm_sleep();
    yp3_pwrlog.arm_b0 = PMU_REG(0xb0);
    yp3_pwrlog.arm_b8 = PMU_REG(0xb8);
    yp3_pwrlog.arm_d0 = PMU_REG(0xd0);

    PWR_STAGE(13);
    while (1) {
        yp3_pwrlog.wfi++;
        __asm__ volatile ("dsb" ::: "memory");
        asm volatile ("wfi");
    }
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
