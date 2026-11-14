/* Kernel tick via Cortex-M SysTick at the 192 MHz core clock. */
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "timer.h"
#include "yp3-buildstamp.h"
#include <stdint.h>

/* A file that only WRITES the ring still needs this, or its logf() compiles
 * to nothing - the trap CLAUDE.md records twice. */
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#include "logf.h"
#define YP3_STALLWATCH 1
#else
#define YP3_STALLWATCH 0
#endif

#ifndef YP3_PROFILE
#define YP3_PROFILE 0
#endif

#define SYST_CSR   (*(volatile uint32_t *)0xE000E010)
#define SYST_RVR   (*(volatile uint32_t *)0xE000E014)
#define SYST_CVR   (*(volatile uint32_t *)0xE000E018)

#define SYST_SHPR3_SYSTICK (*(volatile uint8_t *)0xE000ED23)  /* SHPR3[31:24] */

#define SYST_CSR_ENABLE     (1u << 0)
#define SYST_CSR_TICKINT    (1u << 1)
#define SYST_CSR_CLKSOURCE  (1u << 2)   /* processor clock */

void tick_start(unsigned int interval_in_ms)
{
    uint32_t reload = (CPU_FREQ / 1000u) * interval_in_ms - 1u;

    SYST_CSR = 0;                       /* stop while reprogramming */
    SYST_RVR = reload & 0x00ffffffu;    /* 24-bit reload */
    SYST_CVR = 0;

    /* The tick at the LOWEST priority, below the audio DMA (NVIC priority 2).
     * SysTick resets to 0, the highest, so every device interrupt waited for
     * the whole tick handler - and call_tick_tasks and nearly every tick task
     * execute from XIP flash at ~130x the cost of SRAM. The audio DMA re-arm
     * is a completion interrupt, so for that window the transmitter ran dry:
     * on the first real MP3 the channel completed 147 transfers a second
     * where 44.1 kHz needs 172, every tone came out as a pair of lines 49.5 Hz
     * apart around a missing carrier, and the track ran 16% long - all three
     * fit "three 1024-byte transfers, then a ~2.6 ms stall, every 20 ms".
     * Rockbox's usual model is the same: audio DMA on FIQ, the tick on IRQ.
     * Nothing in a tick task may assume it cannot be preempted by the PCM
     * interrupt; the PCM side already locks itself with sink_lock. */
    SYST_SHPR3_SYSTICK = 0xf0;

    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
}

/* Worst tick-handler duration since the last read, in SysTick counts (core
 * cycles). The count reloads at the tick, so RVR - CVR on the way out is the
 * time spent in the handler - unless preemption stretched it past a whole
 * period, which would read as a small number. With the tick now at the lowest
 * priority this includes any interrupt that preempted it. Read and cleared by
 * the PCM beacon, so it also covers the beacon's own logf. */
uint32_t yp3_tick_cost_max;

void yp3_systick_body(void)
{
    /* Before the tick tasks, not after: the dead-man and the rig countdown are
     * the things that have to keep running when a tick task is what wedged. */
    yp3_rig_tick();
    call_tick_tasks();

    uint32_t used = SYST_RVR - SYST_CVR;
    if (used > yp3_tick_cost_max)
        yp3_tick_cost_max = used;
}

#if YP3_PROFILE
/* A sampling profiler, because "the codec runs at 1% of real time" has been
 * blamed on XIP fetches while the codec itself already runs from SRAM. Each
 * tick records where the interrupted code was. Nothing is logged: the arrays
 * are .bss, which a reset does not clear, and tools/prof_read.py reads them
 * over USB after the rig reboot and maps buckets to symbols with nm.
 *
 * Regions: 0 core flash (.text, below the codec slots), 1 codec slots in
 * flash, 2 codecbuf (the running codec), 3 .icode/.data, 4 boot ROM,
 * 5 anything else, 6 other SRAM. The first profile put 35% in "SRAM" and
 * 0.1% in the codec, so .icode gets its own fine histogram: it is where the
 * idle loop, udelay and the SD drain live, and those mean opposite things. */
#define PROF_FLASH_BASE 0x00c0e000u
#define PROF_FLASH_END  0x00c70000u     /* codec slots start here */
#define PROF_FLASH_SHIFT 9              /* 512-byte buckets */
#define PROF_CODEC_SHIFT 8              /* 256-byte buckets */
#define PROF_CODEC_SPAN  0x00020000u    /* 128 KB of codecbuf is plenty */
#define PROF_ICODE_BASE 0x00806000u
#define PROF_ICODE_SPAN 0x00008000u
#define PROF_ICODE_SHIFT 5              /* 32-byte buckets */

extern unsigned char codecbuf[];
uint32_t yp3_prof_region[8];
uint16_t yp3_prof_icode[PROF_ICODE_SPAN >> PROF_ICODE_SHIFT];
/* Which thread was running, by slot (thread_self() & 0xff). With the LCD
 * busy-wait gone the codec STILL got 0.1% and the scheduler 23%, so the
 * question is no longer "where is the PC" but "whose turn is it". The host
 * names the slots from __threads[] and the ELF. */
uint16_t yp3_prof_thread[32];
uint32_t yp3_prof_total;
uint16_t yp3_prof_flash[(PROF_FLASH_END - PROF_FLASH_BASE) >> PROF_FLASH_SHIFT];
uint16_t yp3_prof_codec[PROF_CODEC_SPAN >> PROF_CODEC_SHIFT];
/* Who holds the CPU without letting go (yp3-0gn.13). Threads are cooperative,
 * so a thread that computes for longer than the PCM lookahead (a quarter
 * second) silences playback however little CPU the codec needs. run_max is
 * the longest run of consecutive ticks that found the same slot running, and
 * yp3_prof_hog is the flash histogram again, but only of samples taken once
 * a run is already PROF_HOG_TICKS long: the code a thread is in WHILE it
 * hogs, which is what is worth moving to SRAM or breaking up. A thread that
 * yields and is rescheduled inside one tick is not seen as a break, so this
 * overstates a run; it never understates one. */
#define PROF_HOG_TICKS 5
uint16_t yp3_prof_run_max[32];
uint16_t yp3_prof_hog_thread[32];
uint16_t yp3_prof_hog[(PROF_FLASH_END - PROF_FLASH_BASE) >> PROF_FLASH_SHIFT];
static uint8_t prof_run_slot = 0xff;
static uint16_t prof_run;

void __attribute__((section(".icode"), used)) yp3_prof_sample(uint32_t pc)
{
    uint32_t cb = (uint32_t)codecbuf, i;

    yp3_prof_total++;
    i = thread_self() & 0xffu;
    if (i < 32 && yp3_prof_thread[i] != 0xffffu)
        yp3_prof_thread[i]++;
    if (i != prof_run_slot) {
        prof_run_slot = i;
        prof_run = 0;
    }
    if (prof_run != 0xffffu)
        prof_run++;
    if (i < 32 && prof_run > yp3_prof_run_max[i])
        yp3_prof_run_max[i] = prof_run;
    if (prof_run >= PROF_HOG_TICKS) {
        if (i < 32 && yp3_prof_hog_thread[i] != 0xffffu)
            yp3_prof_hog_thread[i]++;
        if (pc >= PROF_FLASH_BASE && pc < PROF_FLASH_END) {
            uint32_t b = (pc - PROF_FLASH_BASE) >> PROF_FLASH_SHIFT;
            if (yp3_prof_hog[b] != 0xffffu)
                yp3_prof_hog[b]++;
        }
    }
    if (pc >= PROF_FLASH_BASE && pc < PROF_FLASH_END) {
        yp3_prof_region[0]++;
        i = (pc - PROF_FLASH_BASE) >> PROF_FLASH_SHIFT;
        if (yp3_prof_flash[i] != 0xffffu)
            yp3_prof_flash[i]++;
    } else if (pc >= PROF_FLASH_END && pc < 0x00e00000u) {
        yp3_prof_region[1]++;
    } else if (pc >= cb && pc < cb + PROF_CODEC_SPAN) {
        yp3_prof_region[2]++;
        i = (pc - cb) >> PROF_CODEC_SHIFT;
        if (yp3_prof_codec[i] != 0xffffu)
            yp3_prof_codec[i]++;
    } else if (pc >= PROF_ICODE_BASE && pc < PROF_ICODE_BASE + PROF_ICODE_SPAN) {
        yp3_prof_region[3]++;
        i = (pc - PROF_ICODE_BASE) >> PROF_ICODE_SHIFT;
        if (yp3_prof_icode[i] != 0xffffu)
            yp3_prof_icode[i]++;
    } else if (pc >= 0x00800000u && pc < 0x00880000u) {
        yp3_prof_region[6]++;
    } else if (pc < 0x00060000u) {
        yp3_prof_region[4]++;
    } else {
        yp3_prof_region[5]++;
    }
}
#endif

#if YP3_STALLWATCH
/* THE STALL WATCH. Threads are cooperative here, so one thread that spins
 * without yielding freezes the UI, the codec and the backlight together while
 * the tick carries on - and the black box then holds beacons and nothing else.
 * That is exactly what the 2026-11-15 boots look like: 'playback:
 * initializing', two 'bl on', and 115 seconds of nothing, with a key press in
 * the middle that never lit the backlight.
 *
 * The tick knows what it interrupted. When the same thread slot has been
 * found running for STALL_TICKS in a row and the interrupted PC is not the
 * scheduler's own idle wait, it logs the slot, the PC, the LR and the first
 * code addresses on that thread's stack - a call chain, in the log, from the
 * hang itself. `arm-elf-eabi-addr2line -f -e rockbox.elf <addr>` names them.
 * Stale frames show too, so read the first few.
 *
 * The idle test is a range around switch_thread(): the wfi is inlined into it
 * and check_tmo_expired_inner sits just below it in thread.o (both SRAM). A
 * build that moves them apart reports idle as a stall, which is loud rather
 * than silent; `nm -n` settles it.
 *
 * logf builds only: nothing here exists in a daily image. */
#define STALL_TICKS     (3 * HZ)
#define STALL_LINES     8
#define STALL_IDLE_LO   0x480u      /* below switch_thread */
#define STALL_IDLE_SPAN 0x5c0u
extern void switch_thread(void);
static uint8_t  stall_slot = 0xff;
static uint8_t  stall_lines;
static uint16_t stall_run;
static uint32_t stall_secs;
#define HOG_TICKS       (HZ / 4)    /* the PCM lookahead */
#define HOG_LINES       60
static uint16_t hog_run;
static uint8_t  hog_lines;
static uint32_t hog_pc, hog_lr;

static void yp3_stall_report(const uint32_t *frame, unsigned slot)
{
    uint32_t a[8];
    unsigned n = 0, i;

    for (i = 8; i < 8 + 160 && n < 8; i++) {
        uint32_t v = frame[i];
        if (!(v & 1u))
            continue;
        if ((v >= 0x00c0e000u && v < 0x00e00000u) ||
            (v >= 0x00806000u && v < 0x00810000u))
            a[n++] = v & ~1u;
    }
    for (; n < 8; n++)
        a[n] = 0;
    logf("stall: slot %u for %lus pc=%08lx lr=%08lx", slot,
         (unsigned long)stall_secs, (unsigned long)frame[6],
         (unsigned long)frame[5]);
    logf("stall: stk %08lx %08lx %08lx %08lx", (unsigned long)a[0],
         (unsigned long)a[1], (unsigned long)a[2], (unsigned long)a[3]);
    logf("stall: stk %08lx %08lx %08lx %08lx", (unsigned long)a[4],
         (unsigned long)a[5], (unsigned long)a[6], (unsigned long)a[7]);
}

void __attribute__((section(".icode"), used))
yp3_stall_sample(const uint32_t *frame)
{
    uint32_t pc = frame[6];
    unsigned s = thread_self() & 0xffu;

    if (s != stall_slot ||
        pc - (((uint32_t)switch_thread & ~1u) - STALL_IDLE_LO) < STALL_IDLE_SPAN) {
        /* The run is over. One that outlasted the PCM lookahead (a quarter
         * second) is a dropout the listener heard, whoever needed the CPU:
         * say which slot, how long, and where it was half-way (yp3-0gn.13). */
        if (hog_run >= HOG_TICKS && hog_lines < HOG_LINES) {
            hog_lines++;
            logf("hog: slot %u %u ticks pc=%08lx lr=%08lx", stall_slot,
                 (unsigned)hog_run, (unsigned long)hog_pc,
                 (unsigned long)hog_lr);
        }
        hog_run = 0;
        stall_slot = s;
        stall_run = 0;
        stall_secs = 0;
        return;
    }
    if (hog_run != 0xffffu)
        hog_run++;
    if (hog_run == HOG_TICKS / 2) {
        hog_pc = pc;
        hog_lr = frame[5];
    }
    if (++stall_run < STALL_TICKS)
        return;
    stall_run = 0;
    stall_secs += STALL_TICKS / HZ;
    if (stall_lines >= STALL_LINES)
        return;
    stall_lines++;
    yp3_stall_report(frame, s);
}
#endif /* YP3_STALLWATCH */

/* The vector table (vectors-arm-micro.S, shared) names this. Naked so the
 * profiler and the stall watch can read the exception frame before any
 * prologue moves the stack: EXC_RETURN bit 2 says which stack it is on. */
void __attribute__((naked)) systick_handler(void)
{
#if YP3_PROFILE || YP3_STALLWATCH || defined(YP3_BT_BLOB)
    __asm__ volatile (
        "tst    lr, #4              \n"
        "ite    eq                  \n"
        "mrseq  r0, msp             \n"
        "mrsne  r0, psp             \n"
        "push   {r4, lr}            \n"
        "mov    r4, r0              \n"    /* r4 = the stacked frame */
#if YP3_STALLWATCH
        "bl     yp3_stall_sample    \n"
#endif
#if YP3_PROFILE
        "ldr    r0, [r4, #24]       \n"    /* stacked PC */
        "bl     yp3_prof_sample     \n"
#endif
#ifdef YP3_BT_BLOB
        "ldr    r0, [r4, #24]       \n"    /* stacked PC */
        "bl     yp3_bt_tick         \n"    /* the BT dead-man, bt-yp3box.c */
#endif
        "pop    {r4, lr}            \n"
        "b      yp3_systick_body    \n");
#else
    __asm__ volatile ("b yp3_systick_body\n");
#endif
}

/* The general-purpose timer is not used by this target. */
bool timer_set(long cycles, bool start)
{
    (void)cycles;
    (void)start;
    return false;
}
bool timer_start(void) { return false; }
void timer_stop(void) { }

/* Any device interrupt we have not written a driver for is masked rather than
 * allowed to loop in the default handler. */
void yp3_spurious_irq(void)
{
    uint32_t ipsr;
    unsigned irq;
    __asm__ volatile ("mrs %0, ipsr" : "=r"(ipsr));
    irq = (ipsr & 0x1ff);
    if (irq >= 16) {
        irq -= 16;
        ((volatile uint32_t *)0xE000E180)[irq >> 5] =
            1u << (irq & 31);  /* NVIC ICER */
    }
}

/* Mask every device interrupt. SysTick is a core exception, not NVIC, so the
 * kernel tick is unaffected. */
void yp3_mask_all_irqs(void)
{
    volatile uint32_t *icer = (volatile uint32_t *)0xE000E180;
    volatile uint32_t *icpr = (volatile uint32_t *)0xE000E280;
    int i;
    for (i = 0; i < 8; i++) { icer[i] = 0xffffffffu; icpr[i] = 0xffffffffu; }
}
