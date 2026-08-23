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
 * comes up.
 */
#include "config.h"
#include "system.h"
#include "file.h"
#include "string-extra.h"
#include "blackbox.h"
#include "sl6801-regs.h"
#include "logf.h"

#define BLACKBOX_MAGIC      0x59503342u     /* "YP3B" */
#define BLACKBOX_PATH       ROCKBOX_DIR "/blackbox.txt"
/* Stop a fault that reproduces immediately from rebooting for ever. */
#define BLACKBOX_MAX_FAULTS 3

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
};

/* Placed by app.lds at the head of .persist; never defined in C, so nothing
 * is emitted for it and nothing initialises it. */
extern struct yp3_blackbox yp3_blackbox;

/* .bss, so it IS cleared every boot: it says what this boot found, not what
 * the last one left. */
static bool blackbox_have_previous;
static bool blackbox_written;

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
        return;
    }

    bb->boots++;
    blackbox_have_previous = (bb->reason != BB_NONE);
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

/* Append, so a reboot loop leaves a trail rather than one survivor. */
bool blackbox_dump(void)
{
    struct yp3_blackbox *bb = &yp3_blackbox;
    int fd;

    if (blackbox_written)
        return false;
    blackbox_written = true;

    fd = open(BLACKBOX_PATH, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0)
        return false;

    bb_puts(fd, "\n=== yp3 blackbox ===\n");
    bb_puts(fd, "reason: ");
    bb_puts(fd, bb_reason_name(bb->reason));
    bb_puts(fd, "\n");
    bb_puthex(fd, "boots:  ", bb->boots);

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
    if (blackbox_have_previous)
        (void)blackbox_dump();
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
