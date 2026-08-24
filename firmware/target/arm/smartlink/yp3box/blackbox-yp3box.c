/*
 * The black box: a log that outlives the crash.
 *
 * Rockbox has no flight recorder. panicf() paints the screen - including the
 * last few logf lines, via logf_panic_dump() - and then calls the target's
 * system_exception_wait(), and nothing ever reaches storage. Debug -> Dump
 * Log File is the only way to get a log off this device, and it needs a
 * working UI and a working keypad, which is precisely what a hang, a hard
 * fault or a broken button driver take away.
 *
 * So: put the log ring somewhere nothing clears, and write it out on the way
 * past. app.lds carves a .persist region for it, outside every range crt0
 * touches; the boot ROM only initialises 0x800000-0x806000, so it survives a
 * pinhole reset and a SYSRESETREQ too. This port used to do the same thing
 * with the breadcrumb region and its panic record; that was removed in the
 * cleanup, and ROADMAP B3 said the panic record was worth keeping.
 *
 * Three ways out, covering the three ways a device dies:
 *
 *   panic   panicf() ends in system_exception_wait(), which is ours. Write
 *           the box to the card there and then halt. The filesystem is
 *           usually still fine at that point.
 *   fault   a hard fault cannot safely touch the filesystem, so record the
 *           registers and reset. The next boot writes it out.
 *   hang    nothing runs, so nothing can write. Pinhole-reset it; the ring is
 *           still in SRAM and the next boot writes it out. No keypad needed.
 *
 * In all three cases the file lands at /.rockbox/blackbox.txt before the UI
 * comes up, with the previous runs rotated to blackbox.1.txt and friends -
 * BLACKBOX_KEEP of them, five by default, because the interesting failure is
 * often the one before the one you are looking at.
 */
#include "config.h"
#include "system.h"
#include "file.h"
#include "string-extra.h"
#include "blackbox.h"
#include "sl6801-regs.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "kernel.h"
#include "logf.h"
#include "version.h"
#include "yp3-buildstamp.h"

/* "YP3B", plus a layout revision in the last nibble. BUMP THIS whenever the
 * header or the .persist layout changes: the logf ring sits directly behind
 * this struct, so growing the struct moves the ring, and a stale magic would
 * make the next boot read the old ring at the new offset and dump convincing
 * garbage. A changed magic forces a clean cold init instead, costing exactly
 * one log - the one spanning the change, which is never the interesting one. */
#define BLACKBOX_MAGIC      0x59503344u
/* Stop a fault that reproduces immediately from rebooting for ever. */
#define BLACKBOX_MAX_FAULTS 3

/* How many runs to keep. The newest is blackbox.txt and the rest are
 * blackbox.1.txt .. blackbox.<KEEP-1>.txt, oldest last - the usual rotation.
 * A target can override it; five is enough to see a pattern in a reboot loop
 * without filling the card. */
#ifndef BLACKBOX_KEEP
#define BLACKBOX_KEEP       5
#endif

#define BLACKBOX_NEWEST     ROCKBOX_DIR "/blackbox.txt"
/* Written to first, renamed into place last; see blackbox_dump. */
#define BLACKBOX_PENDING    ROCKBOX_DIR "/blackbox.new"
/* The digit's position in the template below, patched in place rather than
 * formatted: this runs from panic and fault context. */
#define BLACKBOX_ROTATED    ROCKBOX_DIR "/blackbox.0.txt"
#define BLACKBOX_DIGIT_POS  (sizeof(ROCKBOX_DIR "/blackbox.") - 1)

enum blackbox_reason {
    BB_NONE = 0,
    BB_PANIC,
    BB_FAULT,
};

struct yp3_blackbox {
    uint32_t magic;
    uint32_t boots;
    uint32_t reason;
    uint32_t faults;        /* consecutive, reset by a clean boot */
    uint32_t pc, sp, lr, psr;
    uint32_t cfsr, hfsr, mmfar, bfar;
    char     msg[64];
    /* Which firmware filled the ring. Kept in .persist rather than read from
     * rbversion at dump time, because the boot dump writes the PREVIOUS run's
     * log: the version doing the writing is not the version that produced the
     * lines. Getting that backwards is how a log from an old image gets read
     * as evidence about a new one. */
    char     version[48];
    /* Spare. Held the resume point for the audio module probe, which walked
     * every module the vendor enables and proved none of them gates the audio
     * block - it is a clock, not a module. The probe is gone; the field stays
     * because removing it would move the logf ring behind it and force another
     * BLACKBOX_MAGIC bump, costing the log that spans the change. Reuse it for
     * the next probe that has to survive a reset. */
    uint32_t probe_idx;
};

/* Placed by app.lds at the head of .persist; never defined in C, so nothing
 * is emitted for it and nothing initialises it. */
extern struct yp3_blackbox yp3_blackbox;

/* .bss, so it IS cleared every boot: it says what this boot found, not what
 * the last one left. */
static bool blackbox_have_previous;
static bool blackbox_written;
/* The version that filled the ring we inherited, saved before this run stamps
 * its own over it. */
static char blackbox_prev_version[48];
/* Set only on the boot dump, where the record describes the previous run. A
 * panic or fault dump describes THIS run, so it reports this run's version. */
static bool blackbox_dump_is_previous;

/* rbversion is the git hash plus a dirty marker; YP3_BUILD_STAMP is generated
 * per build by tools/40_build.sh. Both are needed: during a debugging session
 * every build comes from the same modified tree, so the hash alone never
 * changes, and __DATE__/__TIME__ are pinned by SOURCE_DATE_EPOCH. The stamp
 * carries a real clock time and a hash of the target sources. */

/* strlcpy/strlcat rather than snprintf: this struct is also written from
 * panic context, and the same helper is used on both paths. */
static void bb_stamp_version(struct yp3_blackbox *bb)
{
    strlcpy(bb->version, rbversion, sizeof(bb->version));
    strlcat(bb->version, " ", sizeof(bb->version));
    strlcat(bb->version, YP3_BUILD_STAMP, sizeof(bb->version));
}

unsigned blackbox_probe_idx(void)
{
    return yp3_blackbox.magic == BLACKBOX_MAGIC ? yp3_blackbox.probe_idx : 0;
}

void blackbox_set_probe_idx(unsigned idx)
{
    if (yp3_blackbox.magic == BLACKBOX_MAGIC)
        yp3_blackbox.probe_idx = idx;
}


/* A clock in the log.
 *
 * The ring records the ORDER of events and nothing about their spacing, so a
 * boot that takes three minutes and a boot that takes three seconds produce
 * the same lines. "Slow somewhere between storage_init and the file browser"
 * is not a finding, and the two counters we do print - storage_init=169
 * mount=12 ticks - account for under two seconds of it.
 *
 * So put one line per second into the same ring everything else uses. It
 * interleaves with the existing logf output, which turns every gap into a
 * number without touching shared logf.c: the last t= line before a stall and
 * the first one after it bracket whatever ran in between.
 *
 * Bounded on purpose. The ring is 6 KB and wraps, so an unbounded beacon
 * would eventually be the only thing in it; BB_TICK_MAX seconds is longer
 * than any boot we are trying to explain and stops well before that. */
/* Report SILENCE, not time.
 *
 * A line per second buried the interesting boot: the ring is 6 KB, it carries
 * several runs, and 75 t= lines per boot pushed the slow run off the front -
 * the one boot we needed was the one that got overwritten.
 *
 * So watch logfindex instead. If it has not moved since the last tick, nothing
 * logged that second; when it moves again, emit one line naming how long the
 * quiet lasted. A busy boot produces almost nothing and a stalled one produces
 * exactly the line that brackets the stall, next to the log entry that ended
 * it. */
#define BB_GAP_MIN 2            /* seconds of silence worth reporting */

static long bb_tick_next;
static int  bb_last_index;
static int  bb_quiet;

static void bb_tick_beacon(void)
{
    if (TIME_BEFORE(current_tick, bb_tick_next))
        return;
    bb_tick_next = current_tick + HZ;

#ifdef ROCKBOX_HAS_LOGF
    if (logfindex == bb_last_index) {
        bb_quiet++;
        return;
    }
    bb_last_index = logfindex;

    if (bb_quiet >= BB_GAP_MIN) {
        int q = bb_quiet;
        bb_quiet = 0;
        /* logf moves the index, so re-sync after it or the next tick counts
         * this line as activity that already happened. */
        logf("... %ds quiet, now t=%ld", q, (long)(current_tick / HZ));
        bb_last_index = logfindex;
        return;
    }
    bb_quiet = 0;
#endif
}

void blackbox_init(void)
{
    struct yp3_blackbox *bb = &yp3_blackbox;

    if (bb->magic != BLACKBOX_MAGIC) {
        /* Cold boot, or the region has never been valid. Clear the whole of
         * .persist, which is the header AND the logf ring behind it - the
         * ring's index and wrap flag live there too and would otherwise be
         * garbage. */
        extern char _persistbegin[], _persistend[];

        memset(_persistbegin, 0, _persistend - _persistbegin);
        bb->magic = BLACKBOX_MAGIC;
        bb->boots = 1;
        bb_stamp_version(bb);
        tick_add_task(bb_tick_beacon);
        return;
    }

    bb->boots++;
    strlcpy(blackbox_prev_version, bb->version,
            sizeof(blackbox_prev_version));
    bb_stamp_version(bb);

    /* A crash is not the only failure worth reading. The symptom that costs
     * the most cycles here - a track that never starts, a hang, a peripheral
     * that goes quiet - leaves no panic and no fault, so keying the dump off
     * bb->reason would write nothing for exactly the runs we reset the device
     * to investigate. Any warm boot that finds log in the ring dumps it, with
     * reason "none (log only)"; pinhole-reset the device and the next boot
     * has the previous run's log on the card. No keypad required. */
    blackbox_have_previous = (bb->reason != BB_NONE);
#ifdef ROCKBOX_HAS_LOGF
    /* logfindex and logfwrap live in .persist too, so they arrive from the
     * previous run UNVALIDATED - and the runs this code exists to record are
     * exactly the ones that may have scribbled on them. An out-of-range index
     * makes bb_write_log read outside logfbuffer and write a file until the
     * card fills. Distrust them: a bad index means the ring is not
     * reconstructable, so drop it rather than dump garbage. */
    if (logfindex < 0 || logfindex >= MAX_LOGF_SIZE) {
        logfindex = 0;
        logfwrap = false;
    }
    else if (logfindex != 0 || logfwrap)
        blackbox_have_previous = true;

    /* The dump happens after this boot has already logged, so the file holds
     * the previous run's log followed by the start of this one. Mark the seam
     * in the ring itself rather than making the reader guess. */
    logf("--- boot %lu ---", (unsigned long)bb->boots);
#endif

    tick_add_task(bb_tick_beacon);
}

/* Formatting by hand: this runs from panic and fault context, where the
 * printf machinery and the heap are not to be trusted. */
static int bb_puts(int fd, const char *s)
{
    return write(fd, s, strlen(s));
}

static void bb_puthex(int fd, const char *label, uint32_t v)
{
    static const char hex[] = "0123456789abcdef";
    char buf[16];
    int i;

    bb_puts(fd, label);
    for (i = 0; i < 8; i++)
        buf[i] = hex[(v >> ((7 - i) * 4)) & 0xf];
    buf[8] = '\n';
    write(fd, buf, 9);
}

/* The ring, oldest first. logf.c keeps entries as NUL-separated strings and
 * wraps; turn the NULs into newlines so the file reads as lines. */
static void bb_write_log(int fd)
{
#ifdef ROCKBOX_HAS_LOGF
    int start = logfwrap ? logfindex : 0;
    int count = logfwrap ? MAX_LOGF_SIZE : logfindex;
    char line[128];
    int n = 0;
    int i;

    for (i = 0; i < count; i++) {
        char c = logfbuffer[(start + i) % MAX_LOGF_SIZE];

        if (c == '\0') {
            if (n) {
                line[n++] = '\n';
                write(fd, line, n);
                n = 0;
            }
            continue;
        }

        line[n++] = c;
        if (n >= (int)sizeof(line) - 1) {
            line[n++] = '\n';
            write(fd, line, n);
            n = 0;
        }
    }
    if (n) {
        line[n++] = '\n';
        write(fd, line, n);
    }
#else
    bb_puts(fd, "(built without logf)\n");
#endif
}

static const char *bb_reason_name(uint32_t reason)
{
    switch (reason) {
    case BB_PANIC: return "panic";
    case BB_FAULT: return "fault";
    default:       return "none (log only)";
    }
}

/* Shift the kept logs down one and drop the oldest, so blackbox.txt is always
 * free for the run being written. Best effort: a rename that fails costs one
 * old log, not this one. */
static void bb_rotate(void)
{
    char older[] = BLACKBOX_ROTATED;
    char newer[] = BLACKBOX_ROTATED;
    int i;

    older[BLACKBOX_DIGIT_POS] = '0' + BLACKBOX_KEEP - 1;
    remove(older);

    for (i = BLACKBOX_KEEP - 1; i > 1; i--) {
        older[BLACKBOX_DIGIT_POS] = '0' + i;
        newer[BLACKBOX_DIGIT_POS] = '0' + i - 1;
        (void)rename(newer, older);
    }

    if (BLACKBOX_KEEP > 1) {
        older[BLACKBOX_DIGIT_POS] = '1';
        (void)rename(BLACKBOX_NEWEST, older);
    }
}

/* Every path out of here is a return, never a panic: this runs on the boot
 * path and from panic and fault context, and a flight recorder that takes the
 * device down when the card is missing is worse than no flight recorder. With
 * no card the open below simply fails and the record stays in SRAM, where a
 * later boot with a card can still write it - nothing is lost by failing. */
bool blackbox_dump(void)
{
    struct yp3_blackbox *bb = &yp3_blackbox;
    int fd;

    if (blackbox_written)
        return false;
    /* Set before any I/O, so a panic raised from inside this function cannot
     * recurse through system_exception_wait back into it. */
    blackbox_written = true;

    /* Write to a scratch name, and only rotate once it is safely closed.
     *
     * This used to rotate first, "so the newest name is free". That loses a
     * run whenever the write fails after the renames: the card had
     * blackbox.1..4 and no blackbox.txt at all, because the rotation succeeded
     * and the write into the freed name did not. Which is the worst possible
     * time to lose a log - the write failed because something had just gone
     * wrong, and that run is the one worth reading.
     *
     * The rename is deliberately not gated on a probe - opendir would take an
     * entry out of the disk cache, and running that out IS a panic (see
     * DC_NUM_ENTRIES_OVERRIDE in the target config). */
    fd = open(BLACKBOX_PENDING, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0)
        return false;       /* no card, no filesystem, full card: all fine */

    bb_puts(fd, "\n=== yp3 blackbox ===\n");
    bb_puts(fd, "reason: ");
    bb_puts(fd, bb_reason_name(bb->reason));
    bb_puts(fd, "\n");
    bb_puthex(fd, "boots:  ", bb->boots);

    {
        const char *v = blackbox_dump_is_previous ? blackbox_prev_version
                                                  : bb->version;
        bb_puts(fd, "build:  ");
        bb_puts(fd, v[0] ? v : "(unknown - cold boot)");
        bb_puts(fd, "\n");
    }

    if (bb->msg[0]) {
        bb->msg[sizeof(bb->msg) - 1] = '\0';
        bb_puts(fd, "msg:    ");
        bb_puts(fd, bb->msg);
        bb_puts(fd, "\n");
    }

    if (bb->reason == BB_FAULT) {
        bb_puthex(fd, "pc:     ", bb->pc);
        bb_puthex(fd, "lr:     ", bb->lr);
        bb_puthex(fd, "sp:     ", bb->sp);
        bb_puthex(fd, "psr:    ", bb->psr);
        bb_puthex(fd, "cfsr:   ", bb->cfsr);
        bb_puthex(fd, "hfsr:   ", bb->hfsr);
        bb_puthex(fd, "mmfar:  ", bb->mmfar);
        bb_puthex(fd, "bfar:   ", bb->bfar);
        bb_puthex(fd, "faults: ", bb->faults);
    }

    bb_puts(fd, "--- log ---\n");
    bb_write_log(fd);
    close(fd);

    /* Everything is on the card under the scratch name. Now shuffle the old
     * runs down and put this one at the head. A failure anywhere above leaves
     * the previous set completely untouched. */
    bb_rotate();
    if (rename(BLACKBOX_PENDING, BLACKBOX_NEWEST) < 0)
        return false;

    /* Consumed. Keep the magic so the next reset is still recognised as warm,
     * but start the ring and the record clean. */
    bb->reason = BB_NONE;
    bb->msg[0] = '\0';
#ifdef ROCKBOX_HAS_LOGF
    logfindex = 0;
    logfwrap = false;
#endif
    return true;
}

/* Called from apps/main.c once the disk is mounted, which is the earliest
 * moment anything can be written. */
void blackbox_boot_dump(void)
{
    if (blackbox_have_previous) {
        blackbox_dump_is_previous = true;
        (void)blackbox_dump();
    }
    else
        yp3_blackbox.faults = 0;    /* a boot that got this far is clean */
}

void blackbox_record_panic(const char *msg)
{
    struct yp3_blackbox *bb = &yp3_blackbox;

    bb->reason = BB_PANIC;
    if (msg) {
        strlcpy(bb->msg, msg, sizeof(bb->msg));
    }
}

/* --- fault handlers --------------------------------------------------------
 *
 * These override the weak aliases to UIE() in system-arm-micro.c, whose
 * definition of a fault is while(1) - the exact hang that leaves nothing
 * behind. Record and reset instead; the next boot writes the file.
 *
 * The stacked frame is r0 r1 r2 r3 r12 lr pc xpsr, at PSP or MSP depending on
 * which stack was in use, which EXC_RETURN bit 2 says.
 */
static const char * const fault_names[] = {
    "hardfault", "memmanage", "busfault", "usagefault", "nmi",
};

void __attribute__((used, noreturn))
yp3_fault(uint32_t *frame, unsigned code)
{
    struct yp3_blackbox *bb = &yp3_blackbox;

    bb->reason = BB_FAULT;
    strlcpy(bb->msg,
            code < ARRAYLEN(fault_names) ? fault_names[code] : "fault",
            sizeof(bb->msg));
    bb->pc   = frame[6];
    bb->lr   = frame[5];
    bb->psr  = frame[7];
    bb->sp   = (uint32_t)frame;
    bb->cfsr = REG32(0xE000ED28);
    bb->hfsr = REG32(0xE000ED2C);
    bb->mmfar = REG32(0xE000ED34);
    bb->bfar  = REG32(0xE000ED38);
    bb->faults++;

    /* A fault that reproduces on every boot would reset for ever and never
     * reach the code that writes the file. Stop after a few and leave the
     * record in SRAM; a later clean boot still finds it. */
    if (bb->faults <= BLACKBOX_MAX_FAULTS)
        system_reboot();

    while (1)
        ;
}

/* These override the weak aliases to UIE() in system-arm-micro.c, whose idea
 * of a fault is while(1) - the exact hang that leaves nothing behind.
 *
 * The stacked frame is r0 r1 r2 r3 r12 lr pc xpsr, on MSP or PSP according to
 * EXC_RETURN bit 2. The handler passes an index rather than a string so that
 * a naked function needs no literal pool. */
#define YP3_FAULT_HANDLER(name, code)                                   \
    void __attribute__((naked, noreturn)) name(void)                    \
    {                                                                   \
        asm volatile (                                                  \
            "tst    lr, #4          \n"                                 \
            "ite    eq              \n"                                 \
            "mrseq  r0, msp         \n"                                 \
            "mrsne  r0, psp         \n"                                 \
            "movs   r1, %0          \n"                                 \
            "b      yp3_fault       \n"                                 \
            : : "I" (code));                                            \
    }

YP3_FAULT_HANDLER(hardfault_handler,  0)
YP3_FAULT_HANDLER(memmanage_handler,  1)
YP3_FAULT_HANDLER(busfault_handler,   2)
YP3_FAULT_HANDLER(usagefault_handler, 3)
YP3_FAULT_HANDLER(nmi_handler,        4)
