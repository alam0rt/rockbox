/* The rig: getting this device through a flash/boot/observe cycle without a
 * human holding VOL and poking the reset pinhole.
 *
 * The host already has one-command install and verify. What it never had was a
 * way IN (bootloader mode needs a button held across a reset) or a way OUT (a
 * running or hung build needs the pinhole). This file is the way out, and
 * crt0's hook is the way in.
 *
 * Three pieces, in increasing order of how much they are trusted:
 *
 *   yp3_reboot()          software reset. Arms the watchdog AND requests
 *                         SYSRESETREQ, because only one of the two is proven
 *                         on this part - see below.
 *   the rig countdown     YP3_RIG_SECONDS, from the kernel tick, which is an
 *                         exact clock: run for N seconds, then reboot to the
 *                         bootloader. This is what makes a cycle unattended.
 *   the dead-man          YP3_DEADMAN_LOAD, the watchdog left running and fed
 *                         from the tick, so a HANG also comes back by itself.
 *
 * On the two reset mechanisms. The watchdog at 0x40083000 is the one with
 * evidence: it is what was resetting Rockbox in a loop before system_init
 * started disabling it (docs/WATCHDOG.md), so this part demonstrably reboots
 * when it expires. SYSRESETREQ is standard ARMv7-M and app.lds has always
 * asserted .persist survives one, but nothing ever measured either claim here
 * [U]. yp3_reboot does both and lets whichever works land first.
 *
 * The watchdog's LOAD units are unknown, so nothing here invents a number:
 * the reload actually used is the one the boot ROM itself armed, captured in
 * system_init before the watchdog is disabled. It is a value known to expire
 * on this hardware, which is all a reboot needs.
 */
/* Without this every logf() below compiles to NOTHING, even in a --type=al
 * build. CLAUDE.md records a whole hardware cycle lost to exactly this, on a
 * beacon in a file that only ever WROTE the ring - and this file repeated it:
 * the first rig boot produced no "rig:" line at all, so the countdown, the
 * reset and the reboot-to-bootloader all had to be inferred from USB events. */
#define LOGF_ENABLE

#include "config.h"
#include "system.h"
#include "kernel.h"
#include "logf.h"
#include "button.h"
#include "sl6801-regs.h"
#include "yp3-buildstamp.h"
#include <stdint.h>

/* Build switches arrive through yp3-buildstamp.h, which tools/40_build.sh
 * regenerates and force-recompiles every build. */
#ifndef YP3_RIG_SECONDS
#define YP3_RIG_SECONDS 0       /* 0 = run normally, never self-reboot */
#endif
#ifndef YP3_DEADMAN_LOAD
#define YP3_DEADMAN_LOAD 0      /* 0 = no dead-man; else a raw WDOG_LOAD */
#endif
#ifndef YP3_RIG_KEYS_INIT
#define YP3_RIG_KEYS_INIT { { 0, 0, 0 } }   /* no script */
#define YP3_RIG_NKEYS 0
#endif
#ifndef YP3_WTRACE
#define YP3_WTRACE 0            /* 1 = record peripheral writes, see below */
#endif

#if YP3_WTRACE
/* The DWT write recorder. Its own file so the same source can be compiled
 * into a patch for the vendor firmware; included rather than listed in
 * firmware/SOURCES so the port stays inside the target directory. */
#include "wtrace-yp3box.c"
#endif

/* A held POWEROFF_BUTTON becomes sys_poweroff() once button.c has counted
 * REPEAT_START ticks plus POWEROFF_COUNT repeats. A rig that powers the device
 * off looks exactly like a rig that hung it, so injected holds on that button
 * are clamped well below it rather than trusted to the script. */
#define RIG_MAX_PLAY_HOLD_MS 200

#define BOOTCTL_MAGIC       0x42430001u
#define BOOTCTL_REQ_BOOTLDR 0x600710ADu

/* struct yp3_bootctl is declared in system-target.h. app.lds places it at the
 * head of .persist and crt0 addresses its fields by constant byte offset, so
 * the two layouts have to agree: +0x00 magic, +0x04 request, +0x08 warm_flag,
 * +0x0c warm_vec, +0x10 resets, +0x14 boots, +0x18 rom_wdog_load. */

/* .bss, so it is this boot's countdown and not the last one's. */
static volatile uint32_t rig_ticks;

/* --- the key script ----------------------------------------------------
 *
 * The UI is only reachable through button_read_device(), which is target code,
 * so a scripted press goes in there and travels the whole real path: the
 * button queue, repeat and long-press timing, action.c, the screen. Nothing in
 * apps/ has to know the press was synthetic.
 *
 * Times are milliseconds from the end of system_init, which is a long way
 * before the UI is actually up. That is deliberate rather than clever: every
 * injected press logs the elapsed time it fired at, so ONE rig boot with a
 * roughly-timed script tells you what the real timings are and the next one is
 * exact. Guessing them from first principles is not possible - it depends on
 * the card, the font, the tree cache and how long buffering takes.
 *
 * YP3_RIG_KEYS="1500:MENU 2000:NEXT 2500:PLAY" from tools/40_build.sh.
 */
struct rig_key { uint32_t at_ms; uint32_t button; uint32_t hold_ms; };
static const struct rig_key rig_keys[] = YP3_RIG_KEYS_INIT;
#define RIG_NKEYS ((unsigned)YP3_RIG_NKEYS)

static long rig_t0;             /* current_tick when the script starts */
#if YP3_RIG_NKEYS > 0
static uint32_t rig_fired;      /* one bit per step, so each logs once */
static bool rig_done_logged;
#endif

void yp3_watchdog_arm(uint32_t load)
{
    /* ROM 0xb7b0's sequence: stop, enable, reload, run. No IRQ handler is
     * installed, which is what makes expiry a reset rather than an interrupt. */
    WDOG_CTRL = 0;
    REG32(WDOG_BASE) |= 1;
    WDOG_LOAD = load;
    WDOG_CTRL = 1;
}

void yp3_watchdog_feed(void)
{
    WDOG_FEED = 1;              /* ROM 0xb7a4 */
}

void yp3_watchdog_disable(void)
{
    WDOG_CTRL = 0;              /* ROM 0xb7e4 */
}

/* Reboot. Never returns - and must not, because the caller's next instruction
 * would run in a machine that is halfway through resetting. */
void yp3_reboot(void)
{
    uint32_t load = yp3_bootctl.rom_wdog_load;

    yp3_bootctl.resets++;

    /* Belt and braces, cheapest-to-doubt last. If the ROM never armed the
     * watchdog on this boot there is no measured reload to use, so skip it
     * rather than guess a number, and rely on SYSRESETREQ alone. */
    if (load)
        yp3_watchdog_arm(load);

    /* Retire every earlier store BEFORE asking for the reset. yp3_bootctl is
     * ordinary SRAM behind a write buffer, and SYSRESETREQ takes effect in the
     * next few cycles: without this the request word this whole path exists to
     * set can still be in flight when the core goes down, and the next boot
     * reads the old value. That is what a first rig boot did - it set the
     * request, reset, and came back with request == 0. */
    __asm__ volatile ("dsb 0xf" ::: "memory");

    REG32(0xE000ED0C) = (0x5FAu << 16) | (1u << 2);   /* AIRCR SYSRESETREQ */

    while (1)
        ;
}

/* Reboot and come back in the ROM's USB download service instead of Rockbox.
 * crt0 acts on the request before it touches anything else.
 *
 * Note this is belt-and-braces with rig mode (tools/53_rig_mode_on.sh): with
 * the HLKJ magic zeroed the ROM lands in the download service on ANY reset
 * without running crt0 - so the request is NOT consumed, it stays in .persist
 * and the next exec'd image finds it. crt0 ignores it when entered from an
 * exception (entry_ipsr != 0), because exec means the host chose this image.
 * The request is what lets a device with a valid boot header still be driven
 * unattended. */
void yp3_reboot_to_bootloader(void)
{
    yp3_bootctl.request = BOOTCTL_REQ_BOOTLDR;
    yp3_reboot();
}

/* A reset from here is abrupt: it can land in the middle of an SD write, and
 * this port has already had a card's filesystem corrupted badly enough to stop
 * mounting. Two things keep that acceptable. The rig loop reads its evidence
 * out of the .persist ring over USB, not off the card, so a cycle does not
 * depend on the card being sound; and the countdown is opt-in per build. If a
 * cycle is going to write to the card, size YP3_RIG_SECONDS to end after it.
 *
 * Called from systick_handler, so it runs even when the main thread is stuck.
 * That is deliberate: a hang with interrupts still on is the common shape
 * here, and it is the one a tick-driven dead-man can catch. A hang that also
 * kills SysTick is caught by the watchdog itself, which is why the dead-man
 * exists on top of the countdown. */
void yp3_rig_tick(void)
{
    if (YP3_DEADMAN_LOAD)
        yp3_watchdog_feed();

    if (rig_ticks && --rig_ticks == 0)
        yp3_reboot_to_bootloader();
}

/* ORed into button_read_device()'s result. Returns the button the script wants
 * held right now, or 0. Runs from the tick task, like the rest of the button
 * read. */
int yp3_rig_buttons(void)
{
#if YP3_RIG_NKEYS > 0
    unsigned i;
    uint32_t ms;
#endif

#if YP3_RIG_NKEYS == 0
    return 0;
#else
    ms = (uint32_t)((current_tick - rig_t0) * (1000 / HZ));

    for (i = 0; i < RIG_NKEYS && i < 32; i++) {
        uint32_t hold = rig_keys[i].hold_ms;

        if (rig_keys[i].button == BUTTON_PLAY && hold > RIG_MAX_PLAY_HOLD_MS)
            hold = RIG_MAX_PLAY_HOLD_MS;

        if (ms < rig_keys[i].at_ms || ms >= rig_keys[i].at_ms + hold)
            continue;

        if (!(rig_fired & (1u << i))) {
            rig_fired |= 1u << i;
            /* The elapsed time, not the scheduled one: that is the number the
             * next script is written from. */
            logf("rig: key #%u btn=%02lx at %lu ms", i,
                 (unsigned long)rig_keys[i].button, (unsigned long)ms);
        }
        return (int)rig_keys[i].button;
    }

    if (!rig_done_logged && RIG_NKEYS &&
        ms >= rig_keys[RIG_NKEYS - 1].at_ms + rig_keys[RIG_NKEYS - 1].hold_ms) {
        rig_done_logged = true;
        logf("rig: key script done, %u of %u fired",
             (unsigned)__builtin_popcount(rig_fired), RIG_NKEYS);
    }
    return 0;
#endif
}

/* Called at the end of system_init, once logf is usable. */
void yp3_rig_init(void)
{
    /* The reset-path probe, read by crt0 and reported here. warm_flag is what
     * decides, at the NEXT reset, whether the ROM runs main at all: nonzero
     * and it jumps through *(0x30000600) and never reaches our image. boots
     * counting up across a reset is the proof that .persist survives one. */
    logf("rig: boots=%lu resets=%lu warm=%08lx vec=%08lx",
         (unsigned long)yp3_bootctl.boots,
         (unsigned long)yp3_bootctl.resets,
         (unsigned long)yp3_bootctl.warm_flag,
         (unsigned long)yp3_bootctl.warm_vec);
    logf("rig: rom wdog load=%08lx", (unsigned long)yp3_bootctl.rom_wdog_load);

#if YP3_WTRACE
    {
        uint32_t n = wtrace_start();
        logf("wtrace: armed, %lu comparators, mask rb %08lx, dhcsr %08lx, %lu volatile words",
             (unsigned long)n, (unsigned long)yp3_wtrace.mask_rb,
             (unsigned long)yp3_wtrace.dhcsr, (unsigned long)yp3_wtrace.volatile_n);
    }
#endif

    if (YP3_DEADMAN_LOAD) {
        yp3_watchdog_arm(YP3_DEADMAN_LOAD);
        logf("rig: dead-man armed, load=%08lx", (unsigned long)YP3_DEADMAN_LOAD);
    }

    if (YP3_RIG_SECONDS) {
        rig_ticks = (uint32_t)YP3_RIG_SECONDS * HZ;
        logf("rig: self-reboot to bootloader in %d s", (int)YP3_RIG_SECONDS);
    }

    rig_t0 = current_tick;
    if (RIG_NKEYS) {
        uint32_t last = rig_keys[RIG_NKEYS - 1].at_ms +
                        rig_keys[RIG_NKEYS - 1].hold_ms;
        logf("rig: %u keys scripted, last at %lu ms",
             RIG_NKEYS, (unsigned long)last);
        /* A script that outlives the countdown never finishes, and the ring
         * then shows a half-navigated UI with no explanation. */
        if (YP3_RIG_SECONDS && last >= (uint32_t)YP3_RIG_SECONDS * 1000)
            logf("rig: WARNING script ends at %lu ms but reboot is at %d000 ms",
                 (unsigned long)last, (int)YP3_RIG_SECONDS);
    }
}
