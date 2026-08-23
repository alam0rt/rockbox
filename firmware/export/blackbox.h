/*
 * The black box: a log that outlives the crash.
 *
 * Rockbox has no flight recorder. panicf() paints the screen and calls the
 * target's system_exception_wait(); nothing reaches storage, and Debug ->
 * Dump Log File needs a working UI and keypad - exactly what a hang, a hard
 * fault or a broken button driver take away.
 *
 * A target opts in with HAVE_BLACKBOX and a linker region that survives
 * reset. See firmware/target/arm/smartlink/yp3box/blackbox-yp3box.c.
 */
#ifndef _BLACKBOX_H_
#define _BLACKBOX_H_

#include <stdbool.h>

#ifdef HAVE_BLACKBOX

/* Validate or clear the region. Call from system_init(), before anything
 * logs. */
void blackbox_init(void);

/* Write a pending record and the log ring out to storage. Call once the disk
 * is mounted; a no-op if there is nothing to report. */
void blackbox_boot_dump(void);

/* Note a panic message so it lands in the file, not just on the panel. */
void blackbox_record_panic(const char *msg);

/* Write now. Returns false if it has already been written this boot or the
 * file could not be opened. */
bool blackbox_dump(void);

#else

static inline void blackbox_init(void) { }
static inline void blackbox_boot_dump(void) { }
static inline void blackbox_record_panic(const char *msg) { (void)msg; }
static inline bool blackbox_dump(void) { return false; }

#endif /* HAVE_BLACKBOX */

#endif /* _BLACKBOX_H_ */
