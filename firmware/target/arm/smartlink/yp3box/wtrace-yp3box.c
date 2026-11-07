/***************************************************************************
 * YP3Box: a register-write recorder built on the DWT and DebugMonitor.
 *
 * WHY. Every silent audio cycle on this port compared one transcription of
 * the vendor firmware with another. The vendor firmware makes sound on this
 * board; what nobody has is the ORDERED SEQUENCE of what it writes. This
 * records it - every CPU write into four peripheral windows, with the PC
 * that made it - into SRAM that survives a reset.
 *
 * HOW. The Cortex-M4 DWT has four comparators. Each one watches a window for
 * data writes; with DEMCR.MON_EN set and no debugger attached, a match raises
 * the DebugMonitor exception (vector 12). The handler reads the matched
 * window, diffs it against a shadow copy and appends (trap no., pc, address,
 * new value) for every word that changed. Writes that change nothing (a
 * repeated value, a write-only register) are logged as a "touch" so the PC
 * is still on record.
 *
 * It is written to run in two hosts:
 *   - Rockbox (YP3_WTRACE, rig-yp3box.c includes this file): proves the
 *     mechanism and records OUR sequence in the same format.
 *   - the vendor FIRM, patched (WTRACE_STANDALONE): records THEIRS.
 * So everything here is plain C on volatile pointers: no libc, no Rockbox
 * calls on the trap path, nothing that would need a symbol from either host.
 *
 * RULES THIS OBEYS, all learned on this device:
 *   - The trap path is in .icode and calls nothing in flash. A DebugMonitor
 *     exception can land in the middle of a clock change, and a flash fetch
 *     then hangs the core (CLAUDE.md, ".icode").
 *   - The FIFO words AUD+0x10c/+0x20c are never read: a read pops them.
 *   - GPIO input-data words (+0x10 per port) are not diffed: they change on
 *     their own, not by writes.
 *   - Words that change on their own anywhere else are LEARNED at start
 *     (two snapshots apart) and excluded; the header says how many.
 *
 * LIMITS, stated so a reader knows what an empty trace does and does not mean:
 *   - DebugMonitor is masked by PRIMASK. A write made with interrupts off is
 *     taken when they come back on - still caught by the diff, but merged
 *     with anything else written in that section and stamped with a later PC.
 *   - The comparator says a window was written, not which word. A write of
 *     an unchanged value is only a "touch".
 *   - Writes by DMA are invisible to the DWT. That is the point: the audio
 *     FIFO is fed by DMA and must not flood the ring.
 ***************************************************************************/

#include <stdint.h>

#ifndef WTRACE_SECTION_CODE
#define WTRACE_SECTION_CODE __attribute__((section(".icode")))
#endif
#ifndef WTRACE_SECTION_RING
#define WTRACE_SECTION_RING __attribute__((section(".yp3snap")))
#endif
#ifndef WTRACE_QUOTA_OTHER
#define WTRACE_QUOTA_OTHER 0    /* records allowed outside the audio window;
                                 * 0 = no limit. The vendor's boot chatters on
                                 * GPIO and the clocks, and its ring is small. */
#endif
#ifndef WTRACE_RECORDS
#define WTRACE_RECORDS 2048   /* 3072 starved the audio arena: "EOM 42044 > 39784" */
#endif
#ifndef WTRACE_HEAD
#define WTRACE_HEAD WTRACE_RECORDS  /* records kept from the start; the rest
                                     * of the ring is a circular TAIL holding
                                     * the newest. HEAD == RECORDS: linear. */
#endif
#ifndef WTRACE_BULK
#define WTRACE_BULK 24      /* more changed words than this in one trap is a
                             * block powering up or a RAM window mapping in,
                             * not a write: one summary record. The first stock
                             * trace lost 300 records to one such trap. */
#endif
#ifndef WTRACE_SNAP
#define WTRACE_SNAP 0       /* 1: one full-state snapshot while playing - see
                             * wt_snap_take. The bulk summaries hide init
                             * values, so the first stock trace could not say
                             * what the +0x200 file or the DMA engine held. */
#endif
#ifndef WTRACE_SNAP_DELAY
#define WTRACE_SNAP_DELAY 300   /* traps after the trigger: ~1 s of stock play */
#endif
#ifndef WTRACE_CAP_AUD
#define WTRACE_CAP_AUD 24   /* records per audio-block word; 0 = no cap */
#endif
#ifndef WTRACE_CAP_OTHER
#define WTRACE_CAP_OTHER 8  /* records per GPIO/clock word: a calibration
                             * loop at stock boot rewrote 20 words 380 times */
#endif

#define WT_REG(a)        (*(volatile uint32_t *)(a))
#define WT_REG8(a)       (*(volatile uint8_t *)(a))

#define WT_DEMCR         0xE000EDFCu
#define WT_DEMCR_TRCENA  (1u << 24)
#define WT_DEMCR_MON_EN  (1u << 16)
#define WT_DHCSR         0xE000EDF0u
#define WT_DFSR          0xE000ED30u
#define WT_SHPR3         0xE000ED20u    /* byte 0 = DebugMonitor priority */
#define WT_DWT_CTRL      0xE0001000u
#define WT_DWT_CYCCNT    0xE0001004u
#define WT_DWT_COMP(n)   (0xE0001020u + 0x10u * (n))
#define WT_DWT_MASK(n)   (0xE0001024u + 0x10u * (n))
#define WT_DWT_FUNC(n)   (0xE0001028u + 0x10u * (n))
#define WT_FUNC_WRITE    0x6u           /* data address, write */
#define WT_FUNC_MATCHED  (1u << 24)

#define WT_MAGIC         0x43525457u    /* "WTRC" */
#define WT_NWIN          4
#define WT_TOUCH         0xFFFF0000u    /* addr of a touch record: | window */
#define WT_BULKREC       0xFFFE0000u    /* addr of a bulk summary: | window;
                                         * val = words that changed */

/* The four windows. Sizes are what gets diffed; mask bits are what the
 * comparator ignores, so the watched span is 1 << mask. Not const: a const
 * table is .rodata, which is flash, and the trap path must not read flash. */
struct wt_win {
    uint32_t base;
    uint32_t words;     /* diffed span, in words */
    uint32_t mask;      /* comparator MASK bits */
};

/* The explicit section matters: GCC promotes a static table that is never
 * written to .rodata on its own, and the first build of this file had it at
 * 0xc60320 - flash - despite the missing const. */
static struct wt_win wt_win[WT_NWIN] __attribute__((section(".data"))) = {
    { 0x40009000u, 0x600u / 4, 12 },   /* audio block, incl. ASRC window   */
    { 0x40081000u, 0x100u / 4,  8 },   /* GPIO ports 0..3                  */
    { 0x40080000u, 0x200u / 4,  9 },   /* clock tree / module gates        */
    { 0x40085000u, 0x200u / 4,  9 },   /* clock control, PMU mailbox, PHY  */
};
#define WT_SHADOW_WORDS  ((0x600u + 0x100u + 0x200u + 0x200u) / 4)
_Static_assert(WTRACE_RECORDS * 16 >= WT_SHADOW_WORDS * 4,
               "the learning snapshot borrows the ring");
_Static_assert(WTRACE_HEAD <= WTRACE_RECORDS, "head larger than the ring");

struct wt_rec {
    uint32_t ev;        /* trap number: this core has no DWT_CYCCNT
                         * (DWT_CTRL 0x4f000000, NOCYCCNT), and order is
                         * what matters; records of one trap share it */
    uint32_t pc;        /* stacked return address: just after the write */
    uint32_t addr;      /* register, or WT_TOUCH | window */
    uint32_t val;       /* new value */
};

struct wt_ring {
    uint32_t magic;
    uint32_t count;         /* records written */
    uint32_t cap;
    uint32_t dropped;       /* changes not recorded: ring full */
    uint32_t suppressed;    /* repeats folded by the dedupe */
    uint32_t events;        /* DebugMonitor entries */
    uint32_t dwt_ctrl;      /* NUMCOMP in [31:28] */
    uint32_t mask_rb;       /* MASK read back, one byte per window */
    uint32_t volatile_n;    /* words learned volatile, all windows */
    uint32_t dhcsr;         /* C_DEBUGEN set = DebugMonitor cannot fire */
    uint32_t start_cyc;
    uint32_t quota_dropped; /* non-audio records refused by the quota */
    uint32_t head;          /* rec[0..head) is the start; the rest circular */
    uint32_t bulk;          /* traps summarised as bulk */
    uint32_t capped;        /* records refused by the per-word cap */
    uint32_t nwords;        /* entries in changes[] */
    struct wt_rec rec[WTRACE_RECORDS];
#if WTRACE_SNAP
    /* filled once: WTRACE_SNAP_DELAY traps after AUD+0x50 first reads
     * 0x30028fff (stock's analogue fully up, trap 4775 of the 2026-11-07 run) */
    uint32_t snap_magic;    /* "SNAP" when taken */
    uint32_t snap_trigger;  /* trap number of the trigger */
    uint32_t snap_ev;       /* trap number when taken */
    uint32_t snap_aud[0x600 / 4];       /* 0x40009000; FIFO words read as 0 */
    uint32_t snap_gpio[0x100 / 4];      /* 0x40081000 */
    uint32_t snap_clk80[0x200 / 4];     /* 0x40080000 */
    uint32_t snap_clk85[0x200 / 4];     /* 0x40085000 */
    uint32_t snap_dma[0x200 / 4];       /* 0x40001000, the general DMA engine */
#endif
    uint16_t changes[WT_SHADOW_WORDS]; /* per watched word: times it changed (saturating),
                             * window order as wt_win - the hammered registers */
};

struct wt_ring yp3_wtrace WTRACE_SECTION_RING;

static uint32_t wt_shadow[WT_SHADOW_WORDS];
static uint32_t wt_volatile[(WT_SHADOW_WORDS + 31) / 32];
static uint32_t wt_off[WT_NWIN];        /* shadow offset of each window */
static uint32_t wt_other;               /* non-audio records stored */

/* Never read: the read has a side effect, or the word is input data. */
static inline int WTRACE_SECTION_CODE wt_skip(unsigned w, unsigned i)
{
    uint32_t off = i * 4;
    if (w == 0)
        return off == 0x10c || off == 0x20c;
    if (w == 1)
        return (off & 0x3f) == 0x10;
    return 0;
}

#define WT_DEDUPE 16

/* Logical record k -> slot. The head is fixed; past it the ring wraps. */
static inline uint32_t WTRACE_SECTION_CODE wt_slot(uint32_t k)
{
    uint32_t h = WTRACE_HEAD;
    return k < h ? k : h + (k - h) % (WTRACE_RECORDS - h);
}

/* s: the word's shadow index, for the per-word cap; -1 for touch/bulk. */
static void WTRACE_SECTION_CODE wt_put(uint32_t pc, uint32_t addr, uint32_t val, int s)
{
    struct wt_ring *r = &yp3_wtrace;
    uint32_t n = r->count, k, live;

    if (s >= 0) {
        uint32_t c = yp3_wtrace.changes[s];
        uint32_t cap = (uint32_t)s < wt_win[0].words ? WTRACE_CAP_AUD : WTRACE_CAP_OTHER;
        if (cap && c > cap) {
            r->capped++;
            return;
        }
    }
    live = n < WTRACE_RECORDS ? n : WTRACE_RECORDS;
    for (k = 1; k <= WT_DEDUPE && k <= live; k++) {
        struct wt_rec *p = &r->rec[wt_slot(n - k)];
        if (p->pc == pc && p->addr == addr && p->val == val) {
            r->suppressed++;
            return;
        }
    }
    if (n >= r->cap) {
        if (WTRACE_HEAD >= WTRACE_RECORDS) {
            r->dropped++;
            return;
        }
        r->dropped++;                   /* now: overwritten, oldest tail first */
    }
    if (WTRACE_QUOTA_OTHER && (addr >> 12) != 0x40009u && addr != WT_TOUCH) {
        if (wt_other >= WTRACE_QUOTA_OTHER) {
            r->quota_dropped++;
            return;
        }
        wt_other++;
    }
    k = wt_slot(n);
    r->rec[k].ev   = r->events;
    r->rec[k].pc   = pc;
    r->rec[k].addr = addr;
    r->rec[k].val  = val;
    r->count = n + 1;
}

static void WTRACE_SECTION_CODE wt_diff(unsigned w, uint32_t pc)
{
    uint32_t base = wt_win[w].base, nw = wt_win[w].words, o = wt_off[w];
    uint32_t i, changed = 0, bulk;

    /* Count first: a trap that changes a whole block is not a write. */
    for (i = 0; i < nw; i++) {
        uint32_t s = o + i;
        if (wt_skip(w, i) || (wt_volatile[s >> 5] & (1u << (s & 31))))
            continue;
        if (WT_REG(base + i * 4) != wt_shadow[s])
            changed++;
    }
    bulk = changed > WTRACE_BULK;
    if (bulk) {
        yp3_wtrace.bulk++;
        wt_put(pc, WT_BULKREC | w, changed, -1);
    }
    changed = 0;
    for (i = 0; i < nw; i++) {
        uint32_t s = o + i, v;
        if (wt_skip(w, i) || (wt_volatile[s >> 5] & (1u << (s & 31))))
            continue;
        v = WT_REG(base + i * 4);
        if (v != wt_shadow[s]) {
            wt_shadow[s] = v;
            changed++;
            if (!bulk) {
                if (yp3_wtrace.changes[s] != 0xffffu)
                    yp3_wtrace.changes[s]++;
                wt_put(pc, base + i * 4, v, (int)s);
            }
        }
    }
    if (!changed)
        wt_put(pc, WT_TOUCH | w, 0, -1);
    if (w == 3) {
        /* The PMU mailbox repeats commands with identical values, which a
         * diff cannot see; keep the raw words on every trap. */
        wt_put(pc, base + 0x104, WT_REG(base + 0x104), (int)(o + 0x104 / 4));
        wt_put(pc, base + 0x108, WT_REG(base + 0x108), (int)(o + 0x108 / 4));
    }
}

#if WTRACE_SNAP
#define WT_SNAP_MAGIC 0x50414e53u     /* "SNAP" */
static uint32_t wt_snap_trigger;

static void WTRACE_SECTION_CODE wt_copy(uint32_t *dst, uint32_t base, uint32_t bytes)
{
    uint32_t i;
    for (i = 0; i < bytes / 4; i++)
        dst[i] = WT_REG(base + i * 4);
}

static void WTRACE_SECTION_CODE wt_snap_check(void)
{
    struct wt_ring *r = &yp3_wtrace;
    uint32_t i;

    if (r->snap_magic == WT_SNAP_MAGIC)
        return;
    if (!wt_snap_trigger) {
        if (WT_REG(0x40009050u) == 0x30028fffu) {
            wt_snap_trigger = r->events;
            r->snap_trigger = r->events;
        }
        return;
    }
    if (r->events - wt_snap_trigger < WTRACE_SNAP_DELAY)
        return;
    r->snap_ev = r->events;
    for (i = 0; i < 0x600 / 4; i++)                 /* never read a FIFO */
        r->snap_aud[i] = (i * 4 == 0x10c || i * 4 == 0x20c)
                       ? 0 : WT_REG(0x40009000u + i * 4);
    wt_copy(r->snap_gpio, 0x40081000u, sizeof r->snap_gpio);
    wt_copy(r->snap_clk80, 0x40080000u, sizeof r->snap_clk80);
    wt_copy(r->snap_clk85, 0x40085000u, sizeof r->snap_clk85);
    wt_copy(r->snap_dma, 0x40001000u, sizeof r->snap_dma);
    r->snap_magic = WT_SNAP_MAGIC;
}
#endif

/* Called from the naked handler with the stacked PC. Returns through the
 * EXC_RETURN the handler left in lr. */
void WTRACE_SECTION_CODE __attribute__((used)) wtrace_event(uint32_t pc)
{
    unsigned w, hit = 0;

    yp3_wtrace.events++;
    for (w = 0; w < WT_NWIN; w++)
        if (WT_REG(WT_DWT_FUNC(w)) & WT_FUNC_MATCHED)  /* read clears */
            hit |= 1u << w;
    if (!hit)
        hit = (1u << WT_NWIN) - 1;      /* pended without a flag: check all */
    for (w = 0; w < WT_NWIN; w++)
        if (hit & (1u << w))
            wt_diff(w, pc);
#if WTRACE_SNAP
    wt_snap_check();
#endif
    WT_REG(WT_DFSR) = 0x1fu;            /* write-one-to-clear, DWTTRAP etc. */
}

void WTRACE_SECTION_CODE __attribute__((naked)) debugmonitor_handler(void)
{
    asm volatile (
        "tst    lr, #4          \n"
        "ite    eq              \n"
        "mrseq  r0, msp         \n"
        "mrsne  r0, psp         \n"
        "ldr    r0, [r0, #24]   \n"     /* stacked PC */
        "b      wtrace_event    \n"
    );
}

static void wt_snapshot(uint32_t *dst)
{
    unsigned w, i;
    for (w = 0; w < WT_NWIN; w++)
        for (i = 0; i < wt_win[w].words; i++)
            dst[wt_off[w] + i] = wt_skip(w, i) ? 0 : WT_REG(wt_win[w].base + i * 4);
}

/* Arm it. Safe to call once; everything before it is not recorded. Returns
 * the number of comparators the core has, so the caller can say so. */
uint32_t wtrace_start(void)
{
    struct wt_ring *r = &yp3_wtrace;
    /* The second learning snapshot borrows the ring, which is not live yet:
     * 2.8 KB the vendor-side build does not have to spare. */
    uint32_t *again = (uint32_t *)r->rec;
    uint32_t o = 0, i, numcomp, mrb = 0;
    unsigned w;
    volatile uint32_t spin;

    for (w = 0; w < WT_NWIN; w++) {
        wt_off[w] = o;
        o += wt_win[w].words;
    }

    WT_REG(WT_DEMCR) |= WT_DEMCR_TRCENA;
    numcomp = WT_REG(WT_DWT_CTRL) >> 28;

    r->magic = 0;
    r->count = r->dropped = r->suppressed = r->events = 0;
    r->quota_dropped = 0;
    wt_other = 0;
    r->cap = WTRACE_RECORDS;
    r->head = WTRACE_HEAD;
    r->bulk = r->capped = 0;
    r->nwords = WT_SHADOW_WORDS;
#if WTRACE_SNAP
    r->snap_magic = r->snap_trigger = r->snap_ev = 0;
    wt_snap_trigger = 0;
#endif
    for (i = 0; i < WT_SHADOW_WORDS; i++)
        yp3_wtrace.changes[i] = 0;
    r->dwt_ctrl = WT_REG(WT_DWT_CTRL);
    r->dhcsr = WT_REG(WT_DHCSR);

    /* Learn what moves by itself: two snapshots ~1 ms apart. */
    for (i = 0; i < (WT_SHADOW_WORDS + 31) / 32; i++)
        wt_volatile[i] = 0;
    wt_snapshot(wt_shadow);
    for (spin = 0; spin < 50000; spin++)
        ;
    wt_snapshot(again);
    r->volatile_n = 0;
    for (i = 0; i < WT_SHADOW_WORDS; i++)
        if (again[i] != wt_shadow[i]) {
            wt_volatile[i >> 5] |= 1u << (i & 31);
            wt_shadow[i] = again[i];
            r->volatile_n++;
        }

    r->start_cyc = WTRACE_SNAP;     /* reused as a layout flag: 1 = a snapshot
                                     * block sits between rec[] and changes[]
                                     * (this core has no cycle counter) */

    for (w = 0; w < WT_NWIN && w < numcomp; w++) {
        WT_REG(WT_DWT_FUNC(w)) = 0;
        WT_REG(WT_DWT_COMP(w)) = wt_win[w].base;
        WT_REG(WT_DWT_MASK(w)) = wt_win[w].mask;
        mrb |= (WT_REG(WT_DWT_MASK(w)) & 0xffu) << (8 * w);
        (void)WT_REG(WT_DWT_FUNC(w));           /* clear a stale MATCHED */
        WT_REG(WT_DWT_FUNC(w)) = WT_FUNC_WRITE;
    }
    r->mask_rb = mrb;

    WT_REG8(WT_SHPR3) = 0;                      /* DebugMonitor: highest */
    r->magic = WT_MAGIC;
    WT_REG(WT_DEMCR) |= WT_DEMCR_MON_EN;
    return numcomp;
}
