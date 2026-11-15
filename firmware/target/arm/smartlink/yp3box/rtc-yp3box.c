/* The SL6801's RTC: a binary calendar in the always-on block at 0x40085000.
 *
 * Included by system-yp3box.c under #if CONFIG_RTC rather than listed in
 * firmware/SOURCES, like wtrace-yp3box.c, so the port stays inside the target
 * directory. docs/RTC.md (yp3box repo) has the whole reversal; what this file
 * leans on, each re-read from the dumps before it was written down here:
 *
 *   decode  SRAM 0x80d988:  t = [base+8]: sec [5:0], min [13:8], hour [20:16],
 *                           weekday [26:24]; d = [base+4]: year-2000 [22:16],
 *                           month [11:8] (1..12), day [4:0]
 *   encode  FIRM 0xcf85d0:  [base+8] = wday<<24 | hour<<16 | min<<8 | sec
 *                           FIRST, then [base+4] = leap<<24 | year<<16 |
 *                           month<<8 | day, leap by the Gregorian rule on
 *                           year + 2000 (0xcf85f6..0xcf861c)
 *   base    SRAM 0x81b078 = 0x40085000, the /dev/rtc HAL handle
 *
 * Weekdays count from MONDAY = 0 (FIRM 0xd67dac; the vendor default
 * 2025-02-11, a Tuesday, is stored as 1). The hardware rolls the weekday
 * itself `[L]`. The block needs no module enable and no clock commit: the
 * vendor driver never calls clk_enable, and under Rockbox, which had never
 * touched it, it counted from its reset value `[L]`.
 *
 * WHAT THIS DRIVER DOES NOT DO, on purpose:
 *
 *   - rtc_init() WRITES NOTHING. The vendor's init stores a default date when
 *     backup word 6 is not 0x5a; here a cold clock simply reads 2000-01-01,
 *     which Rockbox shows as it is, and the block is only ever written when
 *     the user sets the time. So a boot of this build is still the read-only
 *     census docs/RTC.md asks for (yp3-y7w.9), and its log lines are that
 *     census.
 *   - no ROM 0x3b04(24, 1, 0). The vendor calls it every boot; what the
 *     +0x90[16:12] field it changes does is `[U]`, and the clock counts
 *     without it `[L]`.
 *   - no +0x28: IRQ 47 has no handler here and the calendar does not need it.
 *   - no HAVE_RTC_ALARM: nothing shows the alarm can power the device on.
 *
 * NOT YET RUN ON HARDWARE `[U]`: that a date written here reads back, that
 * +0x00 bits 1:0 are the write-pending flags they looked like in the stock
 * trace, and how long the clock keeps time with the device off. */
#include "rtc.h"
#include "timefuncs.h"
#include "kernel.h"

#define AON(o)          (*(volatile uint32_t *)(0x40085000u + (o)))
#define RTC_SYNC        0x00    /* bits 1:0: date / time write pending [L]  */
#define RTC_DATE        0x04
#define RTC_TIME        0x08
#define RTC_IRQ_EN      0x28
#define RTC_IRQ_ST      0x2c
#define RTC_SUBSEC      0x64    /* changes between reads; meaning [U]       */
#define RTC_BKP(n)      (0x180 + 4 * (n))
#define RTC_MAGIC_REG   RTC_BKP(6)
/* The vendor's "this clock has been set" mark (FIRM 0xd67ea0). Shared, so
 * stock firmware trusts a time set here instead of resetting it. */
#define RTC_MAGIC       0x5au
#define RTC_SYNC_SPIN   200000u /* bounded: the vendor does not wait at all */

static unsigned rtc_torn, rtc_sync_timeouts;

/* TIME, DATE, TIME again: the vendor reads each once and so can tear across
 * midnight. Three tries is one more than a tear can need. */
static void rtc_read_words(uint32_t *date, uint32_t *time)
{
    uint32_t t1, t2, d;
    int tries = 3;

    do {
        t1 = AON(RTC_TIME);
        d  = AON(RTC_DATE);
        t2 = AON(RTC_TIME);
    } while (t1 != t2 && --tries);
    if (t1 != t2)
        rtc_torn++;
    *date = d;
    *time = t2;
}

int rtc_read_datetime(struct tm *tm)
{
    uint32_t d, t;

    rtc_read_words(&d, &t);
    tm->tm_sec  = t & 0x3f;
    tm->tm_min  = (t >> 8) & 0x3f;
    tm->tm_hour = (t >> 16) & 0x1f;
    tm->tm_mday = d & 0x1f;
    tm->tm_mon  = (int)((d >> 8) & 0xf) - 1;
    tm->tm_year = (int)((d >> 16) & 0x7f) + 100;
    /* Computed, not taken from [26:24]: the device's weekday is whatever was
     * last written plus its own roll-over, and a date set by another
     * firmware need not agree with the calendar. */
    set_day_of_week(tm);
    set_day_of_year(tm);
    tm->tm_isdst = 0;
    return 1;
}

int rtc_write_datetime(const struct tm *tm)
{
    unsigned year = (unsigned)(tm->tm_year - 100) & 0x7f;   /* since 2000 */
    unsigned y = year + 2000u;
    unsigned leap = (!(y % 4) && (y % 100)) || !(y % 400);
    unsigned wday_mon0 = (unsigned)(tm->tm_wday + 6) % 7;
    unsigned spins = 0;

    AON(RTC_TIME) = wday_mon0 << 24 | (unsigned)tm->tm_hour << 16
                  | (unsigned)tm->tm_min << 8 | (unsigned)tm->tm_sec;
    AON(RTC_DATE) = leap << 24 | year << 16
                  | (unsigned)(tm->tm_mon + 1) << 8 | (unsigned)tm->tm_mday;
    while ((AON(RTC_SYNC) & 3u) && ++spins < RTC_SYNC_SPIN)
        ;
    if (spins >= RTC_SYNC_SPIN)
        rtc_sync_timeouts++;
    AON(RTC_MAGIC_REG) = RTC_MAGIC;
    logf("rtc: set date=%08lx time=%08lx sync=%lx spins=%u bkp6=%02lx",
         (unsigned long)AON(RTC_DATE), (unsigned long)AON(RTC_TIME),
         (unsigned long)AON(RTC_SYNC), spins,
         (unsigned long)AON(RTC_MAGIC_REG));
    return 1;
}

#ifdef ROCKBOX_HAS_LOGF
/* --- the census: read-only, logf builds only (yp3-y7w.9) --------------------
 *
 * One RTC second against the kernel tick, twice: over 64 s and over 640 s.
 * The tick comes off the 24 MHz crystal through the PLL, so it is the
 * reference; edges are seen a tick late at worst, which is 156 ppm over 64 s
 * and 16 ppm over 640 s - enough to tell a 32.768 kHz crystal (tens of ppm)
 * from the trimmed RC the stock captures suggest (0.1-0.8 % fast `[I]`).
 *
 * Also at the first edge: +0x64 before and after, to see whether it is a
 * sub-second counter that restarts on the second, and +0x2c with +0x28 = 0,
 * to see whether the 1 Hz status bit latches without its enable - which is
 * what a driver that wanted to poll the second would need.
 *
 * DWT CYCCNT, which docs/RTC.md proposed for this, does not exist on this
 * part (DWT_CTRL NOCYCCNT = 1 `[L]`). */
static struct {
    uint32_t last_time;
    long     t0;                /* tick of the first edge */
    uint16_t edges;
    uint8_t  stage;             /* 0 idle, 1 waiting for the first edge,
                                   2 counting, 3 done */
    uint32_t sub_prev;          /* +0x64 on the tick before the latest edge */
} rtc_cen;

static void rtc_census_tick(void)
{
    uint32_t t, sub;

    if (rtc_cen.stage == 0 || rtc_cen.stage == 3)
        return;
    t = AON(RTC_TIME);
    sub = AON(RTC_SUBSEC);
    if (t == rtc_cen.last_time) {
        rtc_cen.sub_prev = sub;
        return;
    }
    rtc_cen.last_time = t;
    if (rtc_cen.stage == 1) {
        rtc_cen.stage = 2;
        rtc_cen.t0 = current_tick;
        rtc_cen.edges = 0;
        logf("rtc: edge sub %lx -> %lx st2c=%lx en28=%lx",
             (unsigned long)rtc_cen.sub_prev, (unsigned long)sub,
             (unsigned long)AON(RTC_IRQ_ST), (unsigned long)AON(RTC_IRQ_EN));
        return;
    }
    rtc_cen.edges++;
    if (rtc_cen.edges == 64 || rtc_cen.edges == 640) {
        long ticks = current_tick - rtc_cen.t0;
        long want = (long)rtc_cen.edges * HZ;

        /* ppm the RTC runs FAST by: fewer ticks than seconds = fast. */
        logf("rtc: %u s = %ld ticks (%+ld ppm, +-%u)", rtc_cen.edges, ticks,
             (want - ticks) * (1000000L / HZ) / (long)rtc_cen.edges,
             (unsigned)(1000000u / HZ / rtc_cen.edges));
        if (rtc_cen.edges == 640)
            rtc_cen.stage = 3;
    }
}
#endif /* ROCKBOX_HAS_LOGF */

void rtc_init(void)
{
    uint32_t d, t;

    rtc_read_words(&d, &t);
    /* The retention record. A cold block reads date 00000101, time counting
     * from 0 and bkp6 0; boots/resets place this boot in the reset history,
     * so the card's five black-box logs say which kinds of reset keep it. */
    logf("rtc: date=%08lx time=%08lx bkp6=%02lx sync=%lx en=%lx st=%lx",
         (unsigned long)d, (unsigned long)t,
         (unsigned long)AON(RTC_MAGIC_REG), (unsigned long)AON(RTC_SYNC),
         (unsigned long)AON(RTC_IRQ_EN), (unsigned long)AON(RTC_IRQ_ST));
    logf("rtc: 20%02lu-%02lu-%02lu %02lu:%02lu:%02lu wd%lu boots=%lu resets=%lu",
         (unsigned long)((d >> 16) & 0x7f), (unsigned long)((d >> 8) & 0xf),
         (unsigned long)(d & 0x1f), (unsigned long)((t >> 16) & 0x1f),
         (unsigned long)((t >> 8) & 0x3f), (unsigned long)(t & 0x3f),
         (unsigned long)((t >> 24) & 7),
         (unsigned long)yp3_bootctl.boots, (unsigned long)yp3_bootctl.resets);
    /* The low-power oscillator's measured rate (+0x84, Hz `[I]`) and the
     * words around it: zero under Rockbox so far, because the vendor's power
     * init fills them and this port does not run it. */
    logf("rtc: aon 64=%lx 74=%lx 80=%lx 84=%lx 90=%lx",
         (unsigned long)AON(RTC_SUBSEC), (unsigned long)AON(0x74),
         (unsigned long)AON(0x80), (unsigned long)AON(0x84),
         (unsigned long)AON(0x90));
#ifdef ROCKBOX_HAS_LOGF
    rtc_cen.last_time = t;
    rtc_cen.sub_prev = AON(RTC_SUBSEC);
    rtc_cen.stage = 1;
    tick_add_task(rtc_census_tick);
#endif
}
