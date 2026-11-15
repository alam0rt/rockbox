/* Bluetooth on the SL6801's integrated radio, through the vendor's own stack.
 *
 * NOTHING IN THIS FILE HAS RUN ON HARDWARE (written 2026-11-15). Every
 * address below was read out of the dumps; none is a hardware result. See
 * docs/BLUETOOTH.md in the yp3box repository.
 *
 * Personal builds only. The vendor code is not in this tree and must not be:
 * tools/bt/btpack.py rebuilds it from the user's own flash dump at build
 * time. This file is the Rockbox side of that seam and nothing else.
 *
 * How the vendor stack is laid out [C]:
 *
 *   mask ROM 0x1c000-0x42000   the controller core and FreeRTOS. State in ROM
 *                              RAM 0x801000-0x804xxx, which Rockbox never
 *                              touches. It calls out only through hook tables
 *                              in that RAM, installed by the code below.
 *   flash 0xd7f5e4-0xd94930    RF driver, controller glue, HCI transport,
 *                              host and profiles - kept at its own address.
 *   SRAM  0x8172e4-0x819694    real-time glue, copied back here at bring-up.
 *
 * That code calls 54 functions in the rest of the vendor firmware, which a
 * Rockbox install has overwritten. btpack.py retargets each call to an
 * 8-byte veneer in SRAM, and bt_install() points each veneer at one of the
 * bt_v_*() shims here: FreeRTOS tasks, queues, mutexes and notifications on
 * the Rockbox kernel, a private heap, NV storage as files, printf as a log.
 *
 * What is different from the vendor's world, and matters:
 *
 *   - Rockbox threads are COOPERATIVE. The vendor defers all its interrupt
 *     work to a priority-19 task that preempts everything; here it runs when
 *     the current thread next yields. Idle that is microseconds, under
 *     playback it is not. The queue latency is measured and logged, because
 *     that number decides whether a live link is possible this way [U].
 *   - The tick is 10 ms, the vendor's 1 ms. Every vendor timeout rounds up.
 *   - The vendor code is HARD-FLOAT and uses the FPU; Rockbox never enabled
 *     it and its thread switch does not save FP registers. bt_fpu_on()
 *     enables it with lazy stacking off, and every place a BT thread can be
 *     switched out saves d8-d15 by hand.
 *   - The vendor has unbounded register waits (0xd80432, 0xd8044a, 0xd7f700,
 *     0xd7f73c, 0xd7fda6, 0xd81f1e). They are in its code, not ours, so the
 *     port's "every wait is bounded" rule is kept by the dead-man in
 *     yp3_bt_tick(): no BT progress for BT_HANG_TICKS reboots, and the card
 *     latch stops the next boot from walking into the same wait.
 *
 * Bring-up is a ladder, and each rung logs before it starts:
 *
 *   S0 census    read-only register dump. Always runs.
 *   S1 install   verify the pack, copy SRAM, fill veneers, FPU on
 *   S2 rf        vendor RF init (0xd801d4)
 *   S3 open      host + controller tasks up, HCI reset, "stack open" event
 *   S4 inquiry   scan and list devices
 *   S5 connect   A2DP open to the address in /.rockbox/bt/peer.txt
 *
 * The limit is YP3_BT_STAGE at build time, or one digit in
 * /.rockbox/bt/stage, which needs no reflash. Audio is not routed yet.
 */
#define LOGF_ENABLE
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "thread.h"
#include "semaphore.h"
#include "logf.h"
#include "file.h"
#include "dir.h"
#include "core_alloc.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "strlcpy.h"
#include "sl6801-regs.h"
#include "system-target.h"
#include "yp3-buildstamp.h"
#include "yp3-bt-veneers.h"

#ifndef YP3_BT_STAGE
#define YP3_BT_STAGE 0          /* census only */
#endif
#ifndef YP3_BT_VERBOSE
#define YP3_BT_VERBOSE 0
#endif
/* tools/40_build.sh greps the ELF for this, so a stage switch that did not
 * reach this object fails the build instead of a hardware cycle. */
#define BT_STR_(x) #x
#define BT_STR(x) BT_STR_(x)
static const char bt_stage_tag[] __attribute__((used)) =
    "yp3-bt-stage=" BT_STR(YP3_BT_STAGE) " verbose=" BT_STR(YP3_BT_VERBOSE);
#define BT_START_DELAY  (12 * HZ)   /* after boot: card mounted, UI drawn */
#define BT_HANG_TICKS   (20 * HZ)   /* no BT progress for this long: reboot */
#define BT_HEAP_EXTRA   (24 * 1024) /* from the core arena, on top of the gaps */
#define BT_DIR          "/.rockbox/bt"
#define LATCH_RUN       BT_DIR "/attempt"
#define LATCH_CENSUS    BT_DIR "/census"

/* --- vendor entry points, all Thumb [C] ------------------------------------ */
#define VFN(addr, type)     ((type)((addr) | 1u))
typedef void (*v_void)(void);
typedef void (*v_u32)(uint32_t);
typedef void (*v_ptr)(const void *);
#define V_RF_SET_DRIFT  VFN(0x00d801c8u, v_u32)  /* *0x81a53c = freq_drift */
#define V_RF_INIT       VFN(0x00d801d4u, v_u32)  /* vendor boot calls (0) */
#define V_CTL_SET_19    VFN(0x00d820b0u, v_u32)  /* *0x81af85 = 0x19 at boot */
#define V_SET_NAME      VFN(0x00d8a4d0u, v_ptr)  /* host ctx+0x80 = name */
#define V_SET_EVENT_CB  VFN(0x00d93430u, v_ptr)  /* *0x81ed98 = cb(ev, data) */
#define V_A2DP_ENABLE   VFN(0x00d8e714u, v_u32)  /* 1 in the vendor's 0x400 mode */
#define V_HFP_ENABLE    VFN(0x00d908f0u, v_u32)  /* 0 in that mode */
#define V_AVRCP_VOL     VFN(0x00d8f710u, v_u32)  /* vendor passes 0x78 */
#define V_OPEN          VFN(0x00d94580u, v_void) /* shell "bt open" */
#define V_INQ_START     VFN(0x00d945ccu, v_void) /* shell "bt inquiry start" */
#define V_A2DP_OPEN     VFN(0x00d9466cu, v_ptr)  /* shell "bt a2dp open", bdaddr */
#define V_DISP_HCI      VFN(0x00d8a550u, v_void) /* shell "bt disp hci" */
#define V_DISP_A2DP     VFN(0x00d8e734u, v_void) /* shell "bt disp a2dp" */
#define V_DISP_KEY      VFN(0x00d9423cu, v_void) /* shell "bt disp key" */
/* 0x81a53c on this device, read from dumps/sram_live.bin. The vendor's NV key
 * is "freq_drift" and its default is 0x53 (FIRM 0xd47194, case 0x12). */
#define BT_FREQ_DRIFT   0x5cu
/* The vendor host's debug verbosity byte; 0xb0 turns its printfs on. */
#define V_VERBOSITY     (*(volatile uint8_t *)0x00824401u)

#define NVIC_ISER   ((volatile uint32_t *)0xE000E100)
#define NVIC_ICER   ((volatile uint32_t *)0xE000E180)
#define NVIC_IPR    ((volatile uint8_t  *)0xE000E400)
#define SCB_ICSR    (*(volatile uint32_t *)0xE000ED04)
#define SCB_CPACR   (*(volatile uint32_t *)0xE000ED88)
#define FPU_FPCCR   (*(volatile uint32_t *)0xE000EF34)
#define SYST_RVR    (*(volatile uint32_t *)0xE000E014)
#define SYST_CVR    (*(volatile uint32_t *)0xE000E018)
#define BT_IRQ      26

/* --- state that survives a reset ------------------------------------------- */
/* In .persist (app.lds, .yp3bt): the stage the last boot reached, readable
 * out of SRAM by nm address after a hang, like the log ring. */
struct bt_crumb {
    uint32_t magic;             /* 'BTcr' */
    uint32_t stage;             /* rung being run */
    uint32_t state;             /* 1 started, 2 done, 3 failed, 4 hung */
    uint32_t step;              /* finer position inside the rung */
    uint32_t hang_pc;           /* interrupted PC when the dead-man fired */
    uint32_t irqs;
    uint32_t boots;
    uint32_t spare;
};
struct bt_crumb yp3_bt_crumb __attribute__((section(".yp3bt")));
#define CRUMB_MAGIC 0x72635442u

static void crumb(unsigned stage, unsigned state)
{
    yp3_bt_crumb.stage = stage;
    yp3_bt_crumb.state = state;
    yp3_bt_crumb.step = 0;
}

/* --- time ------------------------------------------------------------------ */
/* Microseconds, from the tick and SysTick's down-counter. Good to a few us;
 * used only for the queue latency, which is the number this build is for. */
static uint32_t ICODE_ATTR bt_now_us(void)
{
    int old = disable_irq_save();
    uint32_t rvr = SYST_RVR + 1;
    uint32_t cvr = SYST_CVR;
    uint32_t t = (uint32_t)current_tick;
    if (SCB_ICSR & (1u << 26)) {    /* PENDSTSET: wrapped, tick not counted */
        cvr = SYST_CVR;
        t++;
    }
    restore_irq(old);
    return t * 10000u + ((rvr - cvr) / (rvr / 10000u));
}

/* --- the BT log ------------------------------------------------------------ */
/* Vendor printf output and our own detail. It does not go to logf: 731 call
 * sites would wrap the black-box ring in seconds and lose the stage records,
 * which are the part that has to survive. The monitor thread appends this to
 * BT_DIR/log.txt. Milestones go to logf as well. */
#define BTLOG_SIZE 4096
static char btlog[BTLOG_SIZE];
static volatile unsigned btlog_w, btlog_r;
static volatile unsigned btlog_lost, vendor_lines;

static void btlog_put(const char *s, unsigned n)
{
    int old = disable_irq_save();
    for (unsigned i = 0; i < n; i++) {
        unsigned nx = (btlog_w + 1) % BTLOG_SIZE;
        if (nx == btlog_r) { btlog_lost++; break; }
        btlog[btlog_w] = s[i];
        btlog_w = nx;
    }
    restore_irq(old);
}

static void btlogf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void btlogf(const char *fmt, ...)
{
    char b[128];
    va_list ap;
    int n = snprintf(b, sizeof b, "%ld ", (long)current_tick);
    va_start(ap, fmt);
    n += vsnprintf(b + n, sizeof b - n - 1, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof b - 2) n = sizeof b - 2;
    b[n++] = '\n';
    btlog_put(b, n);
}

/* Both: a milestone the black box must keep. */
#define MILESTONE(...) do { logf(__VA_ARGS__); btlogf(__VA_ARGS__); } while (0)

/* --- FPU ------------------------------------------------------------------- */
static bool fpu_on;

static void bt_fpu_on(void)
{
    /* CP10/CP11 full access; ASPEN and LSPEN off, so exception frames stay
     * the 8-word basic frame every handler in this port assumes. Interrupt
     * code must therefore never touch the FPU; the vendor ISR 0x819238 does
     * not [C]. */
    FPU_FPCCR &= ~(3u << 30);
    SCB_CPACR |= 0xfu << 20;
    __asm__ volatile ("dsb\n isb\n");
    fpu_on = true;
}

/* d8-d15 are callee-saved in the vendor's ABI and Rockbox's thread switch
 * knows nothing about them. Encoded by hand: this file is built soft-float
 * and the assembler would refuse the mnemonics. */
static void __attribute__((naked, noinline)) ICODE_ATTR bt_fp_save(uint32_t *buf)
{
    (void)buf;
    __asm__ volatile (".short 0xec80, 0x8b10\n"     /* vstmia r0, {d8-d15} */
                      "bx lr\n");
}
static void __attribute__((naked, noinline)) ICODE_ATTR bt_fp_restore(const uint32_t *buf)
{
    (void)buf;
    __asm__ volatile (".short 0xec90, 0x8b10\n"     /* vldmia r0, {d8-d15} */
                      "bx lr\n");
}
#define FP_GUARD(stmt) do {                                 \
        uint32_t fp_[16];                                   \
        if (fpu_on) bt_fp_save(fp_);                        \
        stmt;                                               \
        if (fpu_on) bt_fp_restore(fp_);                     \
    } while (0)

/* --- the dead-man ---------------------------------------------------------- */
static volatile long bt_progress;       /* last tick a BT thread made progress */
static volatile bool bt_watch;
static volatile uint32_t irq_count, irq_burst, irq_storm;

static inline void progress(void) { bt_progress = current_tick; }

/* From the naked systick_handler, with the interrupted PC. A vendor wait
 * that never ends is a thread that never yields, and in a cooperative kernel
 * that is the whole machine; the tick still runs. It does NOT run if the
 * wait is inside a vendor critical section (BASEPRI 0x50 masks the tick) -
 * that one needs the pinhole, and the crumb and the latch still say where. */
void yp3_bt_tick(uint32_t pc)
{
    irq_burst = 0;
    if (bt_watch && TIME_AFTER(current_tick, bt_progress + BT_HANG_TICKS)) {
        bt_watch = false;
        yp3_bt_crumb.state = 4;
        yp3_bt_crumb.hang_pc = pc;
        yp3_bt_crumb.irqs = irq_count;
        logf("bt: HUNG stage %lu step %lu pc=%08lx - reboot",
             (unsigned long)yp3_bt_crumb.stage,
             (unsigned long)yp3_bt_crumb.step, (unsigned long)pc);
        NVIC_ICER[0] = 1u << BT_IRQ;
        yp3_reboot();
    }
}

/* --- heap ------------------------------------------------------------------ */
/* The vendor mallocs from the FreeRTOS heap in ROM, which nothing has
 * initialised here. First fit over the free parts of the hole plus one block
 * from the core arena; 8-byte headers so payloads are 8-aligned as heap_4's
 * are. */
#define HEAP_REGIONS 8
static struct { uint32_t *lo, *hi; } heap_rgn[HEAP_REGIONS];
static unsigned heap_nrgn;
static uint32_t heap_used, heap_high, heap_total, heap_fail, heap_bigfail;

static void heap_add(uintptr_t lo, uintptr_t hi)
{
    lo = (lo + 7) & ~7u;
    hi &= ~7u;
    if (hi < lo + 64 || heap_nrgn == HEAP_REGIONS)
        return;
    heap_rgn[heap_nrgn].lo = (uint32_t *)lo;
    heap_rgn[heap_nrgn].hi = (uint32_t *)hi;
    heap_nrgn++;
    *(uint32_t *)lo = hi - lo;          /* one free block */
    heap_total += hi - lo;
}

void *bt_v_malloc(uint32_t n)
{
    uint32_t need = (n + 8 + 7) & ~7u;
    void *ret = NULL;
    int old = disable_irq_save();
    for (unsigned r = 0; r < heap_nrgn && !ret; r++) {
        uint32_t *p = heap_rgn[r].lo;
        while (p < heap_rgn[r].hi) {
            uint32_t sz = *p & ~1u;
            if (sz < 8)
                break;                  /* corrupt: stop walking this region */
            if (!(*p & 1)) {
                uint32_t *q = (uint32_t *)((uintptr_t)p + sz);
                while (q < heap_rgn[r].hi && !(*q & 1) && (*q & ~1u) >= 8) {
                    sz += *q & ~1u;     /* coalesce forward */
                    q = (uint32_t *)((uintptr_t)p + sz);
                }
                *p = sz;
                if (sz >= need) {
                    if (sz - need >= 16) {
                        *(uint32_t *)((uintptr_t)p + need) = sz - need;
                        sz = need;
                    }
                    *p = sz | 1;
                    heap_used += sz;
                    if (heap_used > heap_high) heap_high = heap_used;
                    ret = p + 2;
                    break;
                }
            }
            p = (uint32_t *)((uintptr_t)p + sz);
        }
    }
    if (!ret) {
        heap_fail++;
        if (n > heap_bigfail) heap_bigfail = n;
    }
    restore_irq(old);
    /* The vendor rarely checks. Say so before it dereferences the NULL. */
    if (!ret && heap_fail <= 4)
        logf("bt: MALLOC %lu FAILED (used %lu of %lu)", (unsigned long)n,
             (unsigned long)heap_used, (unsigned long)heap_total);
    return ret;
}

void bt_v_free(void *ptr)
{
    if (!ptr)
        return;
    uint32_t *p = (uint32_t *)ptr - 2;
    int old = disable_irq_save();
    for (unsigned r = 0; r < heap_nrgn; r++)
        if (p >= heap_rgn[r].lo && p < heap_rgn[r].hi && (*p & 1)) {
            heap_used -= *p & ~1u;
            *p &= ~1u;
            break;
        }
    restore_irq(old);
}

/* --- threads --------------------------------------------------------------- */
#define BT_THREADS 4
struct bt_thread {
    void (*entry)(void *);
    void *arg;
    unsigned id;
    const char *name;
    volatile uint8_t used, exit_req;
    struct semaphore nsem;              /* task notification */
    volatile uint32_t nval;
    struct semaphore *volatile waiting;
};
static struct bt_thread threads[BT_THREADS];
static uintptr_t stack_lo, stack_hi;    /* bump pool: the largest gap */
static unsigned stack_gap;

static struct bt_thread *bt_cur(void)
{
    unsigned id = thread_self();
    for (int i = 0; i < BT_THREADS; i++)
        if (threads[i].used && threads[i].id == id)
            return &threads[i];
    return NULL;
}

static void bt_thread_die(struct bt_thread *t) NORETURN_ATTR;
static void bt_thread_die(struct bt_thread *t)
{
    btlogf("thread %s exits", t->name ? t->name : "?");
    t->used = 0;
    thread_exit();
}

/* Every place a BT thread can block comes through here. */
static int bt_wait(struct semaphore *s, int ticks)
{
    struct bt_thread *t = bt_cur();
    int r;
    progress();
    if (t) t->waiting = s;
    FP_GUARD(r = semaphore_wait(s, ticks));
    if (t) {
        t->waiting = NULL;
        if (t->exit_req)                /* vTaskDelete: it never comes back */
            bt_thread_die(t);
    }
    progress();
    return r;
}

static void ICODE_ATTR bt_wake(struct semaphore *s)
{
    if (is_thread_context())            /* release can switch threads */
        FP_GUARD(semaphore_release(s));
    else
        semaphore_release(s);
}

static void bt_sleep(int ticks)
{
    struct bt_thread *t = bt_cur();
    progress();
    FP_GUARD(sleep(ticks));
    if (t && t->exit_req)
        bt_thread_die(t);
    progress();
}

/* Vendor timeouts are FreeRTOS ticks at 1 kHz, i.e. milliseconds. */
static int ms_to_ticks(int ms)
{
    if (ms < 0)  return TIMEOUT_BLOCK;
    if (ms == 0) return TIMEOUT_NOBLOCK;
    return (ms + (1000 / HZ) - 1) / (1000 / HZ);
}

static void bt_tramp(void)
{
    struct bt_thread *t = bt_cur();
    if (!t) {
        logf("bt: thread with no slot");
        return;
    }
    t->entry(t->arg);
    bt_thread_die(t);
}

/* 0xcf8f6c: thread_create(entry, arg, name, prio, stack_words). The vendor's
 * returns a 0x18-byte struct it never looks inside; ours is the handle. */
static void *thread_create(void (*entry)(void *), void *arg, const char *name,
                           unsigned prio, unsigned stack_words, bool run_now)
{
    struct bt_thread *t = NULL;
    for (int i = 0; i < BT_THREADS; i++)
        if (!threads[i].used) { t = &threads[i]; break; }
    /* The vendor's tasks never carry an interrupt frame: FreeRTOS handlers
     * run on MSP. Ours do, so add room for nested handlers and the shims. */
    size_t size = stack_words * 4 + 1536;
    if (size > 9216) size = 9216;
    size = (size + 7) & ~7u;
    if (!t || stack_lo + size > stack_hi) {
        MILESTONE("bt: cannot create %s (slot %d, stack %lu of %lu)", name,
                  t != NULL, (unsigned long)size,
                  (unsigned long)(stack_hi - stack_lo));
        return NULL;
    }
    void *stack = (void *)stack_lo;
    stack_lo += size;
    memset(t, 0, sizeof *t);
    t->entry = entry;
    t->arg = arg;
    t->name = name;
    semaphore_init(&t->nsem, 1, 0);
    t->used = 1;
    /* FreeRTOS priorities rise with the number: 19 bottom half, 9 btc, 3
     * host. The bottom half must beat everything else in the system. */
    int rb = prio >= 15 ? PRIORITY_REALTIME - 1
           : prio >= 6  ? PRIORITY_REALTIME + 4
           :              PRIORITY_USER_INTERFACE - 1;
    t->id = create_thread(bt_tramp, stack, size, 0, name IF_PRIO(, rb)
                          IF_COP(, CPU));
    if (!t->id) {
        t->used = 0;
        MILESTONE("bt: create_thread %s failed", name);
        return NULL;
    }
    btlogf("thread %s vprio %u -> %d stack %lu", name, prio, rb,
           (unsigned long)size);
    /* A FreeRTOS task of higher priority runs the moment it is created, and
     * the vendor relies on it: the bottom half creates its own queue before
     * the creator goes on to enable the interrupt that posts to it. */
    if (run_now)
        FP_GUARD(yield());
    return t;
}

void *bt_v_thread_create(void (*entry)(void *), void *arg, const char *name,
                         unsigned prio, unsigned stack_words)
{
    return thread_create(entry, arg, name, prio, stack_words, true);
}

/* 0xcf900c: 0 once the thread has been asked to exit, -1 while it should
 * keep running (the vendor polls a semaphore with a zero timeout). */
int bt_v_thread_poll_exit(struct bt_thread *t)
{
    progress();
    return (t && t->exit_req) ? 0 : -1;
}

/* 0xcf9028 / 0xcf9074: ask it to exit, then vTaskDelete. There is no
 * deleting another thread here, so wake it and let bt_wait() end it. */
void bt_v_thread_destroy(struct bt_thread *t)
{
    if (!t || !t->used)
        return;
    t->exit_req = 1;
    struct semaphore *s = t->waiting;
    if (s)
        bt_wake(s);
    bt_wake(&t->nsem);
}

/* 0x80e018 -> ROM xTaskGenericNotify(handle, value, action). */
int ICODE_ATTR bt_v_notify(struct bt_thread *t, uint32_t value, unsigned action)
{
    if (!t || !t->used)
        return 0;
    int old = disable_irq_save();
    switch (action) {
    case 1: t->nval |= value; break;    /* eSetBits */
    case 2: t->nval++; break;           /* eIncrement */
    case 3: t->nval = value; break;     /* eSetValueWithOverwrite */
    default: break;
    }
    restore_irq(old);
    bt_wake(&t->nsem);
    return 1;
}

/* 0xcf9070 -> ROM xTaskNotifyWait(clear_on_entry, clear_on_exit, *value,
 * ticks). The btc task calls it with a 2 ms timeout as its idle. */
int bt_v_notify_wait(uint32_t clr_entry, uint32_t clr_exit, uint32_t *value,
                     int ticks)
{
    struct bt_thread *t = bt_cur();
    if (!t) {
        bt_sleep(1);
        return 0;
    }
    t->nval &= ~clr_entry;
    int got = bt_wait(&t->nsem, ms_to_ticks(ticks)) == OBJ_WAIT_SUCCEEDED;
    if (value)
        *value = t->nval;
    if (got)
        t->nval &= ~clr_exit;
    return got;
}

/* --- queues ---------------------------------------------------------------- */
#define BT_QUEUES 6
struct bt_queue {
    uint8_t used;
    uint16_t item, depth, head, count;
    uint8_t *buf;
    struct semaphore sem;
    const char *name;
    uint32_t t_post;                    /* when it last went non-empty */
    uint32_t lat_max, lat_sum, lat_n, sent, dropped;
};
static struct bt_queue queues[BT_QUEUES];

/* 0xd7f440: create(handle*, name, item_size, *total_bytes) */
int bt_v_queue_create(void **h, const char *name, int item, int *total)
{
    struct bt_queue *q = NULL;
    for (int i = 0; i < BT_QUEUES; i++)
        if (!queues[i].used) { q = &queues[i]; break; }
    int depth = (item > 0 && total) ? *total / item : 0;
    if (!h || !q || depth <= 0)
        return -1;
    memset(q, 0, sizeof *q);
    q->buf = bt_v_malloc(item * depth);
    if (!q->buf) {
        MILESTONE("bt: no heap for queue %d x %d", item, depth);
        return -1;
    }
    q->item = item;
    q->depth = depth;
    q->name = name;
    semaphore_init(&q->sem, depth, 0);
    q->used = 1;
    *h = q;
    btlogf("queue %s %d x %d", name ? name : "(bh)", item, depth);
    return 0;
}

/* The vendor's handle word for the bottom-half queue (literal at SRAM
 * 0x8193ac and 0x819310). */
#define BT_BH_QUEUE ((void **)0x0081ec8cu)
static volatile uint32_t bh_stat, bh_have, bh_runs, bh_max_us;

/* SRAM 0x81727a: send(handle*, item, timeout). Called from the BT interrupt
 * for the bottom-half queue, so it is in .icode and calls nothing in flash. */
int ICODE_ATTR bt_v_queue_send(void **h, const void *item, int timeout)
{
    if (!h || !*h)
        return -3;
    struct bt_queue *q = *h;
    if (h == BT_BH_QUEUE && !is_thread_context()) {
        /* Not queued: bt_irq() runs the bottom half as soon as the vendor
         * ISR returns. See bt_bh_run(). */
        bh_stat = *(const uint32_t *)item;
        bh_have = 1;
        q->sent++;
        return 0;
    }
    int tries = is_thread_context() ? ms_to_ticks(timeout) : 0;
    for (;;) {
        int old = disable_irq_save();
        if (q->count < q->depth) {
            uint8_t *d = q->buf + ((q->head + q->count) % q->depth) * q->item;
            const uint8_t *s = item;
            for (unsigned i = 0; i < q->item; i++)
                d[i] = s[i];
            if (q->count++ == 0)
                q->t_post = bt_now_us();
            q->sent++;
            restore_irq(old);
            bt_wake(&q->sem);
            return 0;
        }
        restore_irq(old);
        if (tries == 0) {
            q->dropped++;
            return -1;
        }
        if (tries > 0)
            tries--;
        bt_sleep(1);
    }
}

/* 0xd7f45e: recv(handle*, buf, timeout): 0, -1 timeout, -3 no queue */
int bt_v_queue_recv(void **h, void *buf, int timeout)
{
    if (!h || !*h)
        return -3;
    struct bt_queue *q = *h;
    if (bt_wait(&q->sem, ms_to_ticks(timeout)) != OBJ_WAIT_SUCCEEDED)
        return -1;
    int old = disable_irq_save();
    if (q->count == 0) {
        restore_irq(old);
        return -1;
    }
    uint32_t lat = bt_now_us() - q->t_post;
    memcpy(buf, q->buf + q->head * q->item, q->item);
    q->head = (q->head + 1) % q->depth;
    if (--q->count)
        q->t_post = bt_now_us();
    if (lat < 0x40000000u) {
        if (lat > q->lat_max) q->lat_max = lat;
        q->lat_sum += lat;
        q->lat_n++;
    }
    restore_irq(old);
    return 0;
}

/* 0xd7f49e: delete(handle*) */
int bt_v_queue_delete(void **h)
{
    if (h && *h) {
        struct bt_queue *q = *h;
        bt_v_free(q->buf);
        q->used = 0;
        *h = NULL;
    }
    return 0;
}

/* --- mutexes --------------------------------------------------------------- */
/* Two exist, both host-side TX locks (0x817e56 HCI commands, 0xd822f0 ACL);
 * the bottom half takes neither. Semaphores, because the vendor "unlocks" a
 * mutex it does not hold before deleting it (0xd82412). */
#define BT_MUTEXES 4
struct bt_mutex { uint8_t used; struct semaphore s; };
static struct bt_mutex mutexes[BT_MUTEXES];
#define MTX(h) ((struct bt_mutex *)*(h))

void *bt_v_mutex_create(void)           /* stub 0x8051f8 */
{
    for (int i = 0; i < BT_MUTEXES; i++)
        if (!mutexes[i].used) {
            mutexes[i].used = 1;
            semaphore_init(&mutexes[i].s, 1, 1);
            return &mutexes[i];
        }
    return NULL;
}

int bt_v_mutex_lock(void **h)           /* SRAM 0x80dfe0, waits for ever */
{
    if (!h || !*h)
        return -3;
    return bt_wait(&MTX(h)->s, TIMEOUT_BLOCK) == OBJ_WAIT_SUCCEEDED ? 0 : -1;
}

void bt_v_mutex_unlock(void **h)        /* SRAM 0x80e006 */
{
    if (h && *h)
        bt_wake(&MTX(h)->s);
}

void bt_v_mutex_delete(void **h)        /* 0xcf8e92 */
{
    if (h && *h) {
        MTX(h)->used = 0;
        *h = NULL;
    }
}

/* --- critical sections ----------------------------------------------------- */
/* The ROM controller's own hooks (0x803c1c/0x803c20) point at SRAM thunks
 * that reached ROM vPortEnterCritical. That one keeps its nesting count in
 * FreeRTOS state nobody initialised, so calling it here would leave BASEPRI
 * raised for ever. Same mask the ROM uses: 0x50. */
static volatile uint32_t crit_nest, crit_saved;

void ICODE_ATTR bt_v_crit_enter(void)
{
    uint32_t old, v = 0x50;
    __asm__ volatile ("mrs %0, basepri" : "=r"(old));
    __asm__ volatile ("msr basepri_max, %0" :: "r"(v) : "memory");
    if (crit_nest++ == 0)
        crit_saved = old;
}

void ICODE_ATTR bt_v_crit_exit(void)
{
    if (crit_nest && --crit_nest == 0) {
        uint32_t old = crit_saved;
        __asm__ volatile ("msr basepri, %0" :: "r"(old) : "memory");
    }
}

/* --- time and delays ------------------------------------------------------- */
void bt_v_delay_ms(uint32_t ms)         /* ROM 0xb78c vTaskDelay */
{
    if (!is_thread_context())
        return;
    bt_sleep(ms ? (ms + (1000 / HZ) - 1) / (1000 / HZ) : 0);
}

/* SRAM 0x80cc4c: wait up to n ms for the vendor's tick variable to move. */
void bt_v_wait_tick(int n)
{
    (void)n;
    bt_v_delay_ms(1);
}

uint32_t bt_v_ms_to_ticks(uint32_t ms) { return ms; }      /* 0xcf8d50 */
uint32_t bt_v_tick_get(void)                                /* 0xcf8d92 */
{
    return (uint32_t)current_tick * (1000 / HZ);
}

/* --- the low-power oscillator block, 0x40085000 ---------------------------- */
#define LPO(off) REG32(0x40085000u + (off))
uint32_t ICODE_ATTR bt_v_lpo_r84(void)      { return LPO(0x84); }  /* 0xc48030 */
uint32_t ICODE_ATTR bt_v_lpo_r78_get(void)  { return LPO(0x78); }  /* 0xc4804c */
void     ICODE_ATTR bt_v_lpo_r78_set(uint32_t v) { LPO(0x78) = v; } /* 0xc48058 */

static struct { uint32_t a, b, c; uint8_t d, e; } lpo_cfg;
/* SRAM 0x80d654 stores five fields, 0x80d66c writes them */
void bt_v_lpo_cfg_set(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e)
{
    lpo_cfg.a = a; lpo_cfg.b = b; lpo_cfg.c = c; lpo_cfg.d = d; lpo_cfg.e = e;
}
void ICODE_ATTR bt_v_lpo_cfg_apply(void)
{
    LPO(0x80) &= 0x7fffffffu;
    LPO(0x80) = lpo_cfg.c << 20 | lpo_cfg.b << 16 | lpo_cfg.a |
                (uint32_t)lpo_cfg.d << 30 | (uint32_t)lpo_cfg.e << 31;
}

/* --- efuse RF trim, 0xcf89dc ----------------------------------------------- */
/* 0xcf88ac: module 0x58 on, clock 0x1b started; then plain reads of the
 * shadow words at 0x40099020.. (0xc481b0), no waits. Field 0xb is word 3
 * bit 31 ("trimmed"), field 5 is word 1 bits 17:14. */
static void ICODE_ATTR bt_efuse_clock(void)
{
    if (!ROM_CLK_IS_ON(0x58))
        ROM_CLK_ENABLE(0x58);
    ROM_CLK_APPLY(0x1b);
}

uint32_t bt_v_efuse_rf_trim(void)
{
    bt_efuse_clock();
    uint32_t w1 = REG32(0x40099024u), w3 = REG32(0x4009902cu);
    uint32_t trim = (w3 >> 31) ? (w1 >> 14) & 0xf : 4;
    MILESTONE("bt: efuse w1=%08lx w3=%08lx trim=%lu", (unsigned long)w1,
              (unsigned long)w3, (unsigned long)trim);
    return trim;
}

/* --- the interrupt --------------------------------------------------------- */
static void (*vendor_isr)(void);
#define BT_IRQ_STORM 600                /* per 10 ms tick; a slot is 625 us */

/* d0-d7 and FPSCR, for code that runs in the interrupt and may use the FPU:
 * ASPEN is off, so nothing stacks them. Hand-encoded like bt_fp_save. */
static void __attribute__((naked, noinline)) ICODE_ATTR bt_fp_isr_save(uint32_t *buf)
{
    (void)buf;
    __asm__ volatile (".short 0xec80, 0x0b10\n"     /* vstmia r0, {d0-d7} */
                      ".short 0xeef1, 0x1a10\n"     /* vmrs r1, fpscr */
                      "str r1, [r0, #64]\n"
                      "bx lr\n");
}
static void __attribute__((naked, noinline)) ICODE_ATTR bt_fp_isr_restore(const uint32_t *buf)
{
    (void)buf;
    __asm__ volatile ("ldr r1, [r0, #64]\n"
                      ".short 0xeee1, 0x1a10\n"     /* vmsr fpscr, r1 */
                      ".short 0xec90, 0x0b10\n"     /* vldmia r0, {d0-d7} */
                      "bx lr\n");
}

/* The vendor's bottom half, the body of the loop at SRAM 0x819320,
 * transcribed. The vendor runs it as a FreeRTOS task of priority 19 that
 * preempts everything the moment the ISR posts to its 3-deep queue. A
 * Rockbox thread cannot preempt, and the first hardware run (2026-11-16 [L])
 * showed what that costs: the baseband interrupts every slot, the queue
 * filled before the thread ran once, and the ISR's "Err isr" printf on every
 * full queue then took longer than a slot, so the interrupt never let go
 * (625 interrupts inside one tick, 593 dropped).
 *
 * So it runs here, in the interrupt, straight after the vendor ISR. That
 * keeps the vendor's ordering (the bottom half preempts btc and the host and
 * is held off by their critical sections, BASEPRI 0x50 against priority
 * 0x70). What it calls is mask ROM and SRAM glue, never flash. NOT YET RUN.
 * Risk [U]: ROM 0x2f3a4 opens with an unbounded wait on 0x4002001c, and a
 * hang in here also stops the tick, so the dead-man cannot see it. */
#define VCALL0(a)    (((void (*)(void))((a) | 1u))())
#define VCALL1(a, x) (((void (*)(int))((a) | 1u))(x))
static void ICODE_ATTR bt_bh_run(uint32_t st)
{
    uint32_t fp[17];
    uint32_t t0 = bt_now_us();
    bt_fp_isr_save(fp);
    (*(volatile uint32_t *)0x0081ec88u)++;
    if (st & 0x0001) { VCALL0(0x00817514u); VCALL0(0x0002f3a4u); }
    if (st & 0x0002) VCALL0(0x0002f388u);
    if (st & 0x1000) {
        REG32(0x40020030u) |= 0x10;
        *(volatile uint32_t *)0x0081a550u = 0xefef0003u;
    }
    if (st & 0x0004) {
        VCALL0(0x00817504u);
        *(volatile uint32_t *)0x0081a550u = 0xefef0005u;
        VCALL1(0x00817328u, 0x40);
    }
    if (st & 0x0400) VCALL1(0x00030dc0u, 1);
    bt_fp_isr_restore(fp);
    uint32_t dt = bt_now_us() - t0;
    if (dt < 0x40000000u && dt > bh_max_us) bh_max_us = dt;
    bh_runs++;
}

/* Threads in this port run on MSP, so a handler runs on whatever thread it
 * interrupted. With the bottom half in the interrupt that is the ROM's
 * controller code on a 512-byte stack: the second hardware run panicked with
 * "Stkov power" (2026-11-16 [L]). The interrupt therefore moves to a stack
 * of its own. Higher-priority handlers that nest land on it too. */
#define BT_IRQ_STACK 2048
static uint32_t bt_irq_stack[BT_IRQ_STACK / 4] __attribute__((aligned(8)));
static void bt_irq_body(void);

static unsigned bt_irq_stack_used(void)
{
    unsigned i = 0;
    while (i < BT_IRQ_STACK / 4 && bt_irq_stack[i] == 0xdeadbeefu)
        i++;
    return BT_IRQ_STACK - i * 4;
}

static void __attribute__((naked)) ICODE_ATTR bt_irq(void)
{
    __asm__ volatile (
        "push   {r4, lr}            \n"
        "mov    r4, sp              \n"
        "ldr    r0, =bt_irq_stack   \n"
        "add    r0, r0, #2048       \n"
        "mov    sp, r0              \n"
        "bl     bt_irq_body         \n"
        "mov    sp, r4              \n"
        "pop    {r4, pc}            \n"
        ".ltorg                     \n");
}

static void __attribute__((used)) ICODE_ATTR bt_irq_body(void)
{
    irq_count++;
    if (++irq_burst > BT_IRQ_STORM) {
        NVIC_ICER[0] = 1u << BT_IRQ;    /* logged by the monitor thread */
        irq_storm = 1;
        return;
    }
    if (vendor_isr)
        vendor_isr();
    if (bh_have) {
        bh_have = 0;
        bt_bh_run(bh_stat);
    }
}

/* ROM 0x2348 writes the RAM vector table at 0x800000. Do the same, with a
 * wrapper in front so a stuck source cannot take the machine. */
void bt_v_irq_register(unsigned irq, void (*handler)(void))
{
    btlogf("irq %u handler %08lx", irq, (unsigned long)(uintptr_t)handler);
    if (irq != BT_IRQ)
        return;
    vendor_isr = handler;
    for (unsigned i = 0; i < BT_IRQ_STACK / 4; i++)
        bt_irq_stack[i] = 0xdeadbeefu;
    ((uint32_t *)0x00800000u)[16 + irq] = (uint32_t)bt_irq;
}

void bt_v_irq_priority(unsigned irq, unsigned prio)     /* stub 0x804dbc */
{
    if (irq == BT_IRQ)
        NVIC_IPR[irq] = prio << 4;      /* vendor: 7. Audio DMA is 2 */
}

void bt_v_irq_enable(unsigned irq)                      /* stub 0x804ce0 */
{
    if (irq == BT_IRQ && vendor_isr)
        NVIC_ISER[irq >> 5] = 1u << (irq & 31);
}

void bt_v_irq_disable(unsigned irq)                     /* stub 0x804d18 */
{
    if (irq == BT_IRQ)
        NVIC_ICER[irq >> 5] = 1u << (irq & 31);
}

/* --- the vendor's ring buffer, 0xd6c50c-0xd6c65x --------------------------- */
/* Transcribed as written, including "used" answering the full size when the
 * ring is full. One writer (controller side), one reader (host), no lock. */
struct vring { uint32_t size, wr, rd; uint8_t *buf; };

static int ring_full(const struct vring *r)
{
    return (r->wr + 1) % r->size == r->rd;
}

uint32_t bt_v_ring_used(struct vring *r)                /* 0xd6c552 */
{
    if (!r || !r->size || r->wr == r->rd) return 0;
    if (ring_full(r)) return r->size;
    return (r->wr + r->size - r->rd) % r->size;
}

uint32_t bt_v_ring_free(struct vring *r)                /* 0xd6c526 */
{
    if (!r || !r->size) return 0;
    if (r->wr == r->rd) return r->size;
    if (ring_full(r)) return 0;
    return (r->rd + r->size - r->wr) % r->size;
}

uint32_t bt_v_ring_peek(struct vring *r, uint8_t *dst, uint32_t n)
{
    if (!r || !dst || bt_v_ring_used(r) < n) return 0;
    uint32_t at = r->rd;
    for (uint32_t i = 0; i < n; i++) {
        dst[i] = r->buf[at];
        at = (at + 1) % r->size;
    }
    return n;
}

uint32_t bt_v_ring_read(struct vring *r, uint8_t *dst, uint32_t n)
{
    if (!r || !dst || bt_v_ring_used(r) < n) return 0;
    for (uint32_t i = 0; i < n; i++) {
        if (r->wr == r->rd) {
            dst[i] = 0xff;
        } else {
            dst[i] = r->buf[r->rd];
            r->rd = (r->rd + 1) % r->size;
        }
    }
    return n;
}

uint32_t bt_v_ring_write(struct vring *r, const uint8_t *src, uint32_t n)
{
    if (!r || !src || bt_v_ring_free(r) < n) return 0;
    for (uint32_t i = 0; i < n; i++)
        if (!ring_full(r)) {
            r->buf[r->wr] = src[i];
            r->wr = (r->wr + 1) % r->size;
        }
    return n;
}

/* --- the service framework, 0xd6c844 --------------------------------------- */
/* A "service" is a thread, a queue and a timer described by a 0x58-byte
 * record the vendor code reads back by offset: +0x04 thread, +0x0c entry,
 * +0x18 arg, +0x1c name, +0x20 priority, +0x24 stack words, +0x28 queue,
 * +0x2c queue name, +0x30 item size, +0x34 total bytes, +0x38 timer,
 * +0x3c timer name, +0x40 period ms, +0x44 callback, +0x4c autoreload,
 * +0x4d 0 = start now, +0x50 index, +0x54 id. Bluetooth has one: id 4,
 * "bt_host_task", queue 4 x 288, a 1 ms timer (0xd944c0). */
#define SVC_N 2
static uint32_t svc[SVC_N][0x58 / 4];
#define SVC_W(s, off)   (*(uint32_t *)((uint8_t *)(s) + (off)))
#define SVC_B(s, off)   (*((uint8_t *)(s) + (off)))
static void (*volatile svc_timer_cb)(void *);
static void *svc_timer_h;
static long svc_timer_last;

int bt_v_svc_create(uint32_t *cfg)
{
    const char *name = (const char *)SVC_W(cfg, 0x1c);
    if (SVC_B(cfg, 0) != 0 || !name)
        return -1;
    int idx = -1;
    for (int i = 0; i < SVC_N; i++) {
        const char *have = (const char *)SVC_W(svc[i], 0x1c);
        if (have && !strcmp(have, name))
            return -1;
        if (!have && idx < 0)
            idx = i;
    }
    if (idx < 0)
        return -1;
    /* Queue before thread, where the vendor does thread first inside a
     * critical section: here the new thread may run inside create. */
    if (SVC_W(cfg, 0x2c) &&
        bt_v_queue_create((void **)&SVC_W(cfg, 0x28), (const char *)SVC_W(cfg, 0x2c),
                          SVC_W(cfg, 0x30), (int *)&SVC_W(cfg, 0x34)) < 0)
        return -1;
    SVC_W(cfg, 0x50) = idx;
    memcpy(svc[idx], cfg, 0x58);
    /* Not run yet: its first act is to look this record up. */
    void *t = thread_create((void (*)(void *))SVC_W(cfg, 0x0c),
                            (void *)SVC_W(cfg, 0x18), name,
                            SVC_B(cfg, 0x20), SVC_W(cfg, 0x24), false);
    SVC_W(cfg, 0x04) = SVC_W(svc[idx], 0x04) = (uint32_t)t;
    SVC_W(cfg, 0x10) = SVC_W(svc[idx], 0x10) = 0;
    if (!t) {
        bt_v_queue_delete((void **)&SVC_W(svc[idx], 0x28));
        memset(svc[idx], 0, 0x58);
        return -1;
    }
    if (SVC_W(cfg, 0x3c) && SVC_W(cfg, 0x44)) {
        svc_timer_h = svc[idx];         /* any non-zero handle */
        SVC_W(cfg, 0x38) = SVC_W(svc[idx], 0x38) = (uint32_t)svc_timer_h;
        svc_timer_last = current_tick;
        if (SVC_B(cfg, 0x4d) == 0)
            svc_timer_cb = (void (*)(void *))SVC_W(cfg, 0x44);
    }
    MILESTONE("bt: service %s up", name);
    FP_GUARD(yield());
    return idx;
}

int bt_v_svc_destroy(const char *name)                  /* 0xd6c910 */
{
    for (int i = 0; i < SVC_N; i++) {
        const char *have = (const char *)SVC_W(svc[i], 0x1c);
        if (have && !strcmp(have, name)) {
            svc_timer_cb = NULL;
            bt_v_queue_delete((void **)&SVC_W(svc[i], 0x28));
            bt_v_thread_destroy((void *)SVC_W(svc[i], 0x04));
            memset(svc[i], 0, 0x58);
            return 0;
        }
    }
    return -1;
}

void *bt_v_svc_find(const char *name)                   /* 0xd6c9a0 */
{
    for (int i = 0; i < SVC_N; i++) {
        const char *have = (const char *)SVC_W(svc[i], 0x1c);
        if (have && !strcmp(have, name))
            return svc[i];
    }
    return NULL;
}

void *bt_v_svc_by_id(uint32_t id)                       /* 0xd6c9d0 */
{
    for (int i = 0; i < SVC_N; i++)
        if (SVC_W(svc[i], 0x1c) && SVC_B(svc[i], 0x54) == id)
            return svc[i];
    return NULL;
}

/* --- NV storage: files ----------------------------------------------------- */
/* Vendor keys seen: bt_addr, bt_name, btlinknum, btlinkinfo (0x60-byte
 * records). The vendor store is in PSMP, which this port does not read. */
static int nv_path(char *path, size_t n, const char *key)
{
    if (!key || !*key || strlen(key) > 24 || strchr(key, '/') ||
        !is_thread_context())
        return -1;
    snprintf(path, n, BT_DIR "/%s.nv", key);
    return 0;
}

int bt_v_nv_get(const char *key, void *buf, uint32_t len)      /* 0xd6d22c */
{
    char path[48];
    int n = -1;
    if (nv_path(path, sizeof path, key) < 0 || !buf || !len)
        return -0x17;
    FP_GUARD({
        int fd = open(path, O_RDONLY);
        if (fd >= 0) {
            n = read(fd, buf, len);
            close(fd);
        }
    });
    btlogf("nv get %s -> %d", key, n);
    progress();
    return n > 0 ? n : -1;
}

int bt_v_nv_set(const char *key, const void *buf, uint32_t len) /* 0xd6d240 */
{
    char path[48];
    int n = -1;
    if (nv_path(path, sizeof path, key) < 0 || !buf)
        return -0x17;
    FP_GUARD({
        int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (fd >= 0) {
            n = write(fd, buf, len);
            close(fd);
        }
    });
    btlogf("nv set %s %lu -> %d", key, (unsigned long)len, n);
    progress();
    return n == (int)len ? 0 : -1;
}

int bt_v_nv_del(const char *key)                                /* 0xd6d254 */
{
    char path[48];
    if (nv_path(path, sizeof path, key) < 0)
        return -0x17;
    FP_GUARD(remove(path));
    progress();
    return 0;
}

/* --- libc the vendor linked ------------------------------------------------ */
/* ROM 0x2b0. The ROM's own putc spins on a UART status bit with no bound. */
int bt_v_printf(const char *fmt, ...)
{
    char b[120];
    va_list ap;
    int n = snprintf(b, sizeof b, "%ld v ", (long)current_tick);
    va_start(ap, fmt);
    n += vsnprintf(b + n, sizeof b - n - 1, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof b - 2) n = sizeof b - 2;
    while (n > 0 && (b[n - 1] == '\n' || b[n - 1] == '\r'))
        n--;
    b[n++] = '\n';
    btlog_put(b, n);
    vendor_lines++;
    return 0;
}

int bt_v_sprintf(char *buf, const char *fmt, ...)       /* 0xda1f8c */
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, 256, fmt, ap);
    va_end(ap);
    return n;
}

char *bt_v_strncpy(char *d, const char *s, size_t n)    /* 0xda2064 */
{
    size_t i = 0;
    for (; i < n && s[i]; i++) d[i] = s[i];
    for (; i < n; i++) d[i] = 0;
    return d;
}

/* 0xda054c, used twice in the RF calibration (0xd81bb0, 0xd81be0) and
 * narrowed to float straight after. Newton from a power-of-two guess. */
double __attribute__((used, noinline)) bt_sqrt_soft(double x)
{
    union { double d; uint64_t u; } v = { x }, g;
    if (!(x > 0))
        return 0;
    int e = (int)((v.u >> 52) & 0x7ff) - 1023;
    g.u = (uint64_t)(e / 2 + 1023) << 52;
    double y = g.d;
    for (int i = 0; i < 8; i++)
        y = 0.5 * (y + x / y);
    return y;
}

/* The vendor passes and returns it in d0. */
void __attribute__((naked)) bt_v_sqrt(void)
{
    __asm__ volatile (
        "push   {r4, lr}            \n"
        ".short 0xec51, 0x0b10      \n"     /* vmov r0, r1, d0 */
        "bl     bt_sqrt_soft        \n"
        ".short 0xec41, 0x0b10      \n"     /* vmov d0, r0, r1 */
        "pop    {r4, pc}            \n");
}

extern void __aeabi_d2f(void);          /* libgcc; base PCS, as the vendor's */
extern void __aeabi_d2ulz(void);

/* --- install --------------------------------------------------------------- */
struct blob_hdr {
    uint32_t magic, version, nrec, ven_base, ven_count, table_hash, flash_sum,
             total;
};
struct blob_rec { uint32_t dst, len, off; };

static uint32_t sum32(const uint32_t *p, const uint32_t *end)
{
    uint32_t s = 0;
    while (p < end) {
        s += *p++;
        if (!((uintptr_t)p & 0xfff))
            progress();
    }
    return s;
}

#define X(i, fn) (uint32_t)(uintptr_t)fn,
static const uint32_t veneer_target[BT_VEN_COUNT] = { BT_VENEERS(X) };
#undef X
#define X(lo, hi) { lo, hi },
static const struct { uint32_t lo, hi; } clusters[] = { BT_CLUSTERS(X) };
static const struct { uint32_t lo, hi; } gaps[] = { BT_GAPS(X) };
#undef X

/* A fake UART for the printfs inside the mask ROM, which cannot be
 * retargeted: ROM 0x210 waits on dev->regs[2] bit 4 and writes regs[0]. */
static uint32_t fake_uart[4] = { 0, 0, 0x10, 0 };
static uint32_t *fake_uart_dev[1] = { fake_uart };

static bool bt_install(void)
{
    const struct blob_hdr *h = (const struct blob_hdr *)BT_BLOB_ADDR;

    yp3_bt_crumb.step = 1;
    if (h->magic != BT_BLOB_MAGIC || h->version != BT_PACK_VERSION ||
        h->ven_base != BT_VEN_BASE || h->ven_count != BT_VEN_COUNT ||
        h->table_hash != BT_TABLE_HASH || h->total > 0x10000) {
        MILESTONE("bt: no pack at %08lx (magic %08lx ver %lu hash %08lx)",
                  (unsigned long)BT_BLOB_ADDR, (unsigned long)h->magic,
                  (unsigned long)h->version, (unsigned long)h->table_hash);
        return false;
    }
    const uint32_t *b = (const uint32_t *)h;
    if (sum32(b, b + h->total / 4) != b[h->total / 4]) {
        MILESTONE("bt: pack blob checksum");
        return false;
    }
    yp3_bt_crumb.step = 2;
    uint32_t s = sum32((const uint32_t *)BT_STR_LO, (const uint32_t *)BT_STR_HI) +
                 sum32((const uint32_t *)BT_CODE_LO, (const uint32_t *)BT_CODE_HI);
    if (s != h->flash_sum) {
        MILESTONE("bt: vendor flash sum %08lx, pack says %08lx",
                  (unsigned long)s, (unsigned long)h->flash_sum);
        return false;
    }

    yp3_bt_crumb.step = 3;
    for (unsigned i = 0; i < ARRAYLEN(clusters); i++)
        memset((void *)clusters[i].lo, 0, clusters[i].hi - clusters[i].lo);
    const struct blob_rec *r = (const struct blob_rec *)(h + 1);
    for (unsigned i = 0; i < h->nrec; i++, r++) {
        bool ok = false;
        for (unsigned c = 0; c < ARRAYLEN(clusters); c++)
            if (r->dst >= clusters[c].lo && r->dst + r->len <= ((clusters[c].hi + 3) & ~3u))
                ok = true;
        if (!ok || r->off + r->len > h->total) {
            MILESTONE("bt: record %u outside the clusters (%08lx+%lu)", i,
                      (unsigned long)r->dst, (unsigned long)r->len);
            return false;
        }
        memcpy((void *)r->dst, (const uint8_t *)h + r->off, r->len);
    }
    /* ldr.w pc, [pc] ; .word shim. The address carries the Thumb bit. */
    uint32_t *v = (uint32_t *)BT_VEN_BASE;
    for (unsigned i = 0; i < BT_VEN_COUNT; i++) {
        v[2 * i] = 0xf000f8dfu;
        v[2 * i + 1] = veneer_target[i] | 1u;
    }

    yp3_bt_crumb.step = 4;
    for (unsigned i = 0; i < ARRAYLEN(gaps); i++)
        if (i != stack_gap)
            heap_add(gaps[i].lo, gaps[i].hi);
    /* The unused tail of the low region, below the hole (app.lds). */
    extern char _bssloend[];
    heap_add((uintptr_t)_bssloend, BT_HOLE_LO);
    /* Best effort: audio took the arena at boot and its shrink callback
     * refuses below AUDIO_BUFFER_RESERVE, so expect "extra NO" [I]. */
    int extra = core_alloc_ex(BT_HEAP_EXTRA, &buflib_ops_locked);
    if (extra > 0) {
        uintptr_t p = (uintptr_t)core_get_data(extra);
        heap_add(p, p + BT_HEAP_EXTRA);
    }
    bt_fpu_on();
    /* ROM printf context at 0x80082c: device, lock fn, unlock fn. The ROM
     * skips the two functions when they are NULL (0x2be, 0x2ce). */
    ((uint32_t *)0x0080082cu)[0] = (uint32_t)fake_uart_dev;
    ((uint32_t *)0x0080082cu)[1] = 0;
    ((uint32_t *)0x0080082cu)[2] = 0;
    V_VERBOSITY = YP3_BT_VERBOSE ? 0xb0 : 0;
    MILESTONE("bt: installed, %u veneers, heap %lu (%u regions, extra %s)",
              BT_VEN_COUNT, (unsigned long)heap_total, heap_nrgn,
              extra > 0 ? "yes" : "NO");
    return true;
}

/* --- S0: the census -------------------------------------------------------- */
/* Read-only. The gate words are listed here and read through the table so
 * no line of this file pairs a gate address with an assignment. */
static const uint32_t census_addr[] = {
    0x40080030u, 0x40080064u, 0x40080074u, 0x400800e8u,     /* gates RF init flips */
    0x40085078u, 0x40085080u, 0x40085084u,                  /* LPO */
    0x40082104u, 0x40082108u,                               /* 64-bit timer */
    0x40099020u, 0x40099024u, 0x40099028u, 0x4009902cu,     /* efuse shadow */
    0x40010000u, 0x40011004u, 0x40011040u,                  /* RF / analogue */
    0x40020000u, 0x40020004u, 0x4002000cu, 0x40020010u,     /* BT core */
    0x4002001cu,
};

static void bt_census(const char *when)
{
    for (unsigned i = 0; i < ARRAYLEN(census_addr); i++) {
        yp3_bt_crumb.step = 0x100 + i;  /* a fault here names the register */
        uint32_t v = REG32(census_addr[i]);
        MILESTONE("bt: %s %08lx=%08lx", when, (unsigned long)census_addr[i],
                  (unsigned long)v);
    }
    yp3_bt_crumb.step = 0x200;
    /* Does the time base run? The vendor ISR timestamps with it. */
    uint32_t t0 = REG32(census_addr[7]);
    bt_sleep(2);
    uint32_t t1 = REG32(census_addr[7]);
    MILESTONE("bt: %s timer %08lx -> %08lx in 20 ms", when, (unsigned long)t0,
              (unsigned long)t1);
    /* ROM RAM the vendor's init is expected to fill: the controller's
     * critical-section hooks, the ke event word, the printf context. */
    MILESTONE("bt: %s hooks %08lx %08lx ke=%08lx printf=%08lx clk6=%lu",
              when, (unsigned long)REG32(0x00803c1cu),
              (unsigned long)REG32(0x00803c20u),
              (unsigned long)REG32(0x008041e4u),
              (unsigned long)REG32(0x0080082cu),
              (unsigned long)((uint32_t (*)(unsigned))0x3851u)(6));
}

/* --- the application side -------------------------------------------------- */
/* What the vendor's bt manager (0xcf9e74, not kept) receives. Events seen
 * there: 1 stack open (data[0] status), 2 stack close, 3 inquiry complete,
 * 4 device {bdaddr[6], pad, cod u32 at +8, name at +12}, 5 connection
 * complete (result at +6), 6 disconnected. */
#define BT_DEVS 8
static struct { uint8_t addr[6]; uint32_t cod; char name[32]; } devs[BT_DEVS];
static volatile unsigned ndevs;
static volatile int ev_open = -1, ev_inq_done, ev_conn = -1, ev_disc, ev_count;

static void bt_app_event(uint32_t ev, const uint8_t *d)
{
    ev_count++;
    btlogf("event %lu data %08lx", (unsigned long)ev, (unsigned long)(uintptr_t)d);
    if (ev >= 5 && d)                   /* 7 and up are not decoded yet */
        btlogf("event %lu: %02x %02x %02x %02x %02x %02x %02x %02x "
               "%02x %02x %02x %02x", (unsigned long)ev, d[0], d[1], d[2],
               d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10], d[11]);
    switch (ev) {
    case 1: ev_open = d ? (int8_t)d[0] : 0; break;
    case 3: ev_inq_done = 1; break;
    case 4:
        if (d && ndevs < BT_DEVS) {
            unsigned n = ndevs;
            for (unsigned i = 0; i < n; i++)
                if (!memcmp(devs[i].addr, d, 6))
                    return;
            memcpy(devs[n].addr, d, 6);
            memcpy(&devs[n].cod, d + 8, 4);
            strlcpy(devs[n].name, (const char *)d + 12, sizeof devs[n].name);
            ndevs = n + 1;
        }
        break;
    case 5: ev_conn = d ? d[6] : 0xff; break;
    case 6: ev_disc = 1; break;
    }
}

/* --- the monitor thread ---------------------------------------------------- */
static char local_name[] = "Rockbox YP3";       /* SRAM: the host keeps the pointer */

static void bt_flush(void)
{
    if (btlog_r == btlog_w)
        return;
    int fd = open(BT_DIR "/log.txt", O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0)
        return;
    unsigned w = btlog_w;
    if (w < btlog_r) {
        write(fd, btlog + btlog_r, BTLOG_SIZE - btlog_r);
        btlog_r = 0;
    }
    if (w > btlog_r)
        write(fd, btlog + btlog_r, w - btlog_r);
    btlog_r = w;
    close(fd);
}

/* One pass of housekeeping: the host's 1 ms timer, the storm report, the
 * log file. Everything that waits in this thread waits through here. */
static void bt_service(void)
{
    static long last_flush, last_beacon;
    progress();
    void (*cb)(void *) = svc_timer_cb;
    if (cb) {
        long ms = (current_tick - svc_timer_last) * (1000 / HZ);
        svc_timer_last = current_tick;
        if (ms > 200) ms = 200;
        while (ms-- > 0)                /* 0xd948c8: one call per ms */
            cb(svc_timer_h);
    }
    if (irq_storm) {
        irq_storm = 0;
        MILESTONE("bt: IRQ STORM, irq 26 masked after %lu", (unsigned long)irq_count);
    }
    if (TIME_AFTER(current_tick, last_beacon + 5 * HZ)) {
        last_beacon = current_tick;
        /* The bottom-half queue is the one the vendor creates unnamed
         * (SRAM 0x819336); its latency is the number that matters. */
        struct bt_queue *q = &queues[0];
        for (int i = 0; i < BT_QUEUES; i++)
            if (queues[i].used && !queues[i].name)
                q = &queues[i];
        logf("bt: irq=%lu bh n=%lu max=%luus avg=%luus drop=%lu heap=%lu/%lu "
             "fail=%lu v=%u", (unsigned long)irq_count, (unsigned long)q->lat_n,
             (unsigned long)q->lat_max,
             (unsigned long)(q->lat_n ? q->lat_sum / q->lat_n : 0),
             (unsigned long)q->dropped, (unsigned long)heap_high,
             (unsigned long)heap_total, (unsigned long)heap_fail, vendor_lines);
        btlogf("irq=%lu heap used=%lu high=%lu fail=%lu big=%lu lost=%u",
               (unsigned long)irq_count, (unsigned long)heap_used,
               (unsigned long)heap_high, (unsigned long)heap_fail,
               (unsigned long)heap_bigfail, btlog_lost);
        btlogf("bh in irq: runs=%lu max=%lu us stack %u/%u",
               (unsigned long)bh_runs, (unsigned long)bh_max_us,
               bt_irq_stack_used(), BT_IRQ_STACK);
        logf("bt: bh irq runs=%lu max=%luus stk=%u", (unsigned long)bh_runs,
             (unsigned long)bh_max_us, bt_irq_stack_used());
        bh_max_us = 0;
        for (int i = 0; i < BT_QUEUES; i++)
            if (queues[i].used)
                btlogf("q%d %s sent=%lu drop=%lu lat max=%lu avg=%lu us", i,
                       queues[i].name ? queues[i].name : "bh",
                       (unsigned long)queues[i].sent,
                       (unsigned long)queues[i].dropped,
                       (unsigned long)queues[i].lat_max,
                       (unsigned long)(queues[i].lat_n ?
                           queues[i].lat_sum / queues[i].lat_n : 0));
        q->lat_max = 0;
    }
    if (TIME_AFTER(current_tick, last_flush + HZ)) {
        last_flush = current_tick;
        FP_GUARD(bt_flush());
    }
    progress();
}

static bool bt_wait_for(volatile int *flag, int unset, int seconds)
{
    long end = current_tick + seconds * HZ;
    while (*flag == unset && TIME_BEFORE(current_tick, end)) {
        bt_service();
        bt_sleep(1);
    }
    return *flag != unset;
}

/* The latch. A rung that hangs must not hang the next boot too: the device
 * is in daily use and a boot loop is a pinhole reset every time. The file
 * names the build, so a new build gets one fresh attempt by itself. */
static bool latch_set(const char *path)
{
    char b[48] = "";
    int fd = open(path, O_RDONLY);
    if (fd < 0)
        return false;
    int n = read(fd, b, sizeof b - 1);
    close(fd);
    return n > 0 && !strncmp(b, YP3_BUILD_STAMP, strlen(YP3_BUILD_STAMP));
}

static void latch_write(const char *path, bool on)
{
    if (!on) {
        remove(path);
        return;
    }
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        write(fd, YP3_BUILD_STAMP "\n", sizeof YP3_BUILD_STAMP);
        close(fd);
    }
}

static int read_stage_limit(void)
{
    char b[4] = "";
    int limit = YP3_BT_STAGE;
    int fd = open(BT_DIR "/stage", O_RDONLY);
    if (fd >= 0) {
        if (read(fd, b, 1) == 1 && b[0] >= '0' && b[0] <= '5')
            limit = b[0] - '0';
        close(fd);
    }
    return limit;
}

/* "AA:BB:CC:DD:EE:FF" as a headset prints it; the stack wants it reversed. */
static bool read_peer(uint8_t addr[6])
{
    char b[24] = "";
    int fd = open(BT_DIR "/peer.txt", O_RDONLY);
    if (fd < 0)
        return false;
    int n = read(fd, b, 17);
    close(fd);
    if (n < 17)
        return false;
    for (int i = 0; i < 6; i++) {
        unsigned v = 0;
        for (int k = 0; k < 2; k++) {
            char c = b[i * 3 + k];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= c - '0';
            else if ((c | 0x20) >= 'a' && (c | 0x20) <= 'f') v |= (c | 0x20) - 'a' + 10;
            else return false;
        }
        addr[5 - i] = v;
    }
    return true;
}

#define STAGE_BEGIN(n, what) do { crumb(n, 1); progress();              \
        MILESTONE("bt: S%d %s start", n, what); bt_flush(); } while (0)
#define STAGE_END(n)         do { crumb(n, 2); progress();              \
        MILESTONE("bt: S%d done", n); bt_flush(); } while (0)

static void bt_main(void)
{
    sleep(BT_START_DELAY);
    mkdir(BT_DIR);
    remove(BT_DIR "/log.1.txt");
    rename(BT_DIR "/log.txt", BT_DIR "/log.1.txt");

    int limit = read_stage_limit();
    bool prev_hung = yp3_bt_crumb.magic == CRUMB_MAGIC &&
                     (yp3_bt_crumb.state == 1 || yp3_bt_crumb.state == 4);
    MILESTONE("bt: build %s limit S%d; last boot S%lu state %lu step %lx pc %08lx",
              YP3_BUILD_STAMP, limit, (unsigned long)yp3_bt_crumb.stage,
              (unsigned long)yp3_bt_crumb.state, (unsigned long)yp3_bt_crumb.step,
              (unsigned long)yp3_bt_crumb.hang_pc);
    if (yp3_bt_crumb.magic != CRUMB_MAGIC)
        memset(&yp3_bt_crumb, 0, sizeof yp3_bt_crumb);
    yp3_bt_crumb.magic = CRUMB_MAGIC;
    yp3_bt_crumb.boots++;

    if (limit >= 1 && (latch_set(LATCH_RUN) || prev_hung)) {
        MILESTONE("bt: the last attempt did not finish - census only. "
                  "Delete " LATCH_RUN " to retry");
        limit = 0;
    }
    /* The census only reads, but a read of a block whose module is off has
     * not been tried on every block it touches. If one faults, it must not
     * fault on every boot of a device that is in daily use. */
    if (latch_set(LATCH_CENSUS)) {
        MILESTONE("bt: the census did not finish last time (step %lx) - "
                  "Bluetooth off. Delete " LATCH_CENSUS " to retry",
                  (unsigned long)yp3_bt_crumb.step);
        bt_flush();
        goto idle;
    }

    latch_write(LATCH_CENSUS, true);
    STAGE_BEGIN(0, "census");
    bt_census("s0");
    STAGE_END(0);
    latch_write(LATCH_CENSUS, false);
    if (limit < 1)
        goto idle;

    latch_write(LATCH_RUN, true);
    bt_watch = true;

    STAGE_BEGIN(1, "install");
    if (!bt_install()) {
        crumb(1, 3);
        goto done;
    }
    STAGE_END(1);
    if (limit < 2)
        goto done;

    STAGE_BEGIN(2, "rf init");
    V_RF_SET_DRIFT(BT_FREQ_DRIFT);
    yp3_bt_crumb.step = 1;
    V_RF_INIT(0);
    yp3_bt_crumb.step = 2;
    V_CTL_SET_19(0x19);
    bt_census("s2");
    STAGE_END(2);
    if (limit < 3)
        goto done;

    STAGE_BEGIN(3, "host open");
    V_SET_NAME(local_name);
    V_SET_EVENT_CB((const void *)(uintptr_t)bt_app_event);
    V_A2DP_ENABLE(1);
    V_HFP_ENABLE(0);
    V_AVRCP_VOL(0x78);
    yp3_bt_crumb.step = 1;
    V_OPEN();
    yp3_bt_crumb.step = 2;
    if (!bt_wait_for(&ev_open, -1, 20)) {
        MILESTONE("bt: no stack-open event in 20 s (irq=%lu, events=%d)",
                  (unsigned long)irq_count, ev_count);
        crumb(3, 3);
        goto done;
    }
    MILESTONE("bt: STACK OPEN status %d, irq=%lu", ev_open,
              (unsigned long)irq_count);
    bt_census("s3");
    V_DISP_HCI();
    if (ev_open != 0) {
        crumb(3, 3);
        goto done;
    }
    STAGE_END(3);
    if (limit < 4)
        goto done;

    STAGE_BEGIN(4, "inquiry");
    V_INQ_START();
    bt_wait_for(&ev_inq_done, 0, 30);
    MILESTONE("bt: inquiry %s, %u devices", ev_inq_done ? "complete" : "TIMED OUT",
              ndevs);
    for (unsigned i = 0; i < ndevs; i++)
        MILESTONE("bt: dev %02X:%02X:%02X:%02X:%02X:%02X cod %06lx %s",
                  devs[i].addr[5], devs[i].addr[4], devs[i].addr[3],
                  devs[i].addr[2], devs[i].addr[1], devs[i].addr[0],
                  (unsigned long)devs[i].cod, devs[i].name);
    STAGE_END(4);
    if (limit < 5)
        goto done;

    STAGE_BEGIN(5, "a2dp open");
    static uint8_t peer[8];             /* SRAM: the host copies from it later */
    if (!read_peer(peer)) {
        MILESTONE("bt: no " BT_DIR "/peer.txt - put AA:BB:CC:DD:EE:FF in it");
        crumb(5, 3);
        goto done;
    }
    V_A2DP_OPEN(peer);
    bt_wait_for(&ev_conn, -1, 30);
    MILESTONE("bt: connection result %d (0 = connected)", ev_conn);
    V_DISP_A2DP();
    V_DISP_KEY();
    /* Watch the link settle: the first S5 run (2026-11-16 [L]) reported
     * "connected", then two event 7s and silence, and the headset never
     * chimed. Observation only. */
    for (int i = 0; i < 6 && !ev_disc; i++) {
        bt_wait_for(&ev_disc, 0, 4);
        btlogf("bt: watch %d disc=%d events=%d", i, ev_disc, ev_count);
        V_DISP_HCI();
        V_DISP_A2DP();
    }
    STAGE_END(5);

done:
    /* Reached the end of what was asked without hanging: the next boot may
     * try again. The dead-man stays on while the stack is up. */
    bt_flush();
    latch_write(LATCH_RUN, false);
idle:
    for (;;) {
        bt_service();
        bt_sleep(1);
    }
}

/* From button_init_device(): the kernel is up, the card is not yet. */
void yp3_bt_boot(void)
{
    /* The largest free part of the hole is the stack pool. Nothing else in
     * Rockbox is linked there (app.lds), so it is usable before install. */
    unsigned big = 0;
    for (unsigned i = 1; i < ARRAYLEN(gaps); i++)
        if (gaps[i].hi - gaps[i].lo > gaps[big].hi - gaps[big].lo)
            big = i;
    stack_gap = big;
    stack_lo = gaps[big].lo;
    stack_hi = gaps[big].hi;
    size_t size = 5120;                 /* runs the vendor RF calibration */
    void *stack = (void *)stack_lo;
    stack_lo += size;
    if (!create_thread(bt_main, stack, size, 0, "bt" IF_PRIO(, PRIORITY_REALTIME + 5)
                       IF_COP(, CPU)))
        logf("bt: no monitor thread");
}
