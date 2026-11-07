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
#ifndef WTRACE_RECORDS
#define WTRACE_RECORDS 2048   /* 3072 starved the audio arena: "EOM 42044 > 39784" */
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
    uint32_t reserved;
    struct wt_rec rec[WTRACE_RECORDS];
};

struct wt_ring yp3_wtrace WTRACE_SECTION_RING;

static uint32_t wt_shadow[WT_SHADOW_WORDS];
static uint32_t wt_volatile[(WT_SHADOW_WORDS + 31) / 32];
static uint32_t wt_off[WT_NWIN];        /* shadow offset of each window */

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

static void WTRACE_SECTION_CODE wt_put(uint32_t pc, uint32_t addr, uint32_t val)
{
    struct wt_ring *r = &yp3_wtrace;
    uint32_t n = r->count, k;

    for (k = 1; k <= WT_DEDUPE && k <= n; k++) {
        struct wt_rec *p = &r->rec[n - k];
        if (p->pc == pc && p->addr == addr && p->val == val) {
            r->suppressed++;
            return;
        }
    }
    if (n >= r->cap) {
        r->dropped++;
        return;
    }
    r->rec[n].ev   = r->events;
    r->rec[n].pc   = pc;
    r->rec[n].addr = addr;
    r->rec[n].val  = val;
    r->count = n + 1;
}

static void WTRACE_SECTION_CODE wt_diff(unsigned w, uint32_t pc)
{
    uint32_t base = wt_win[w].base, nw = wt_win[w].words, o = wt_off[w];
    uint32_t i, changed = 0;

    for (i = 0; i < nw; i++) {
        uint32_t s = o + i, v;
        if (wt_skip(w, i) || (wt_volatile[s >> 5] & (1u << (s & 31))))
            continue;
        v = WT_REG(base + i * 4);
        if (v != wt_shadow[s]) {
            wt_shadow[s] = v;
            wt_put(pc, base + i * 4, v);
            changed++;
        }
    }
    if (!changed)
        wt_put(pc, WT_TOUCH | w, 0);
    if (w == 3) {
        /* The PMU mailbox repeats commands with identical values, which a
         * diff cannot see; keep the raw words on every trap. */
        wt_put(pc, base + 0x104, WT_REG(base + 0x104));
        wt_put(pc, base + 0x108, WT_REG(base + 0x108));
    }
}

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
    static uint32_t again[WT_SHADOW_WORDS];
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
    r->cap = WTRACE_RECORDS;
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

    r->start_cyc = 0;

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
