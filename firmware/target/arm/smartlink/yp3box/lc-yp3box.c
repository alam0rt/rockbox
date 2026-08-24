/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2026 by the YP3Box port
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

/* lc_open() for a target whose codecs are already in memory.
 *
 * Codec .text and .rodata are linked into per-codec slots in SPI NOR and
 * execute in place from there, so there is nothing to read off the card and
 * codecbuf only has to hold .data and .bss. Loading such a codec is copying
 * .data to its home in codecbuf; codec_start() zeroes .bss itself, as it
 * always has.
 *
 * This sits at the lc_open() seam deliberately. apps/codecs.c asks the loader
 * for a binary by path and gets a header back, and that contract is unchanged
 * whether the binary comes from the card or was already resident - so the
 * whole split stays inside the linker script, the build, and this target file,
 * and no shared Rockbox code has to know the target has flash slots.
 *
 * Plugins fall through to the stock card loader, built as lc_open_from_file()
 * when HAVE_LC_OPEN_TARGET is defined. See firmware/export/lc-rock.h.
 *
 * A codec named in the slot table does NOT fall through, and that is
 * deliberate. With the split on, the .codec files this build writes to the
 * card are linked at flash addresses - objcopy emits the slot image, not a
 * loadable RAM image - so reading one into codecbuf would land foreign bytes
 * there and jump into an address that may be erased flash. A slot that fails
 * its descriptor check means the image on the device and tools/codec_slots.txt
 * have drifted apart, and the honest answer to that is to refuse the codec,
 * not to substitute a file that cannot work either. Fail visibly; the log line
 * names the codec and the slot.
 */

#include "config.h"
#include "system.h"
#include "string.h"
#include <stdbool.h>
#include "rbpaths.h"
#include "metadata.h"   /* CODEC_PREFIX, CODEC_EXTENSION */
#include "load_code.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

#ifdef HAVE_CODEC_XIP
#include "codec-slots.h"   /* generated; see tools/gen_codec_slots.py */

/* Written into the slot base by plugin.lds, so the firmware needs to know
 * nothing but the slot address: where .data's initialiser image is, where it
 * goes and how long it is are all products of the codec's own link, and are
 * read back out of flash here. That is what keeps the build single-pass - the
 * firmware never has to be linked after the codecs. */
#define CODEC_XIP_MAGIC 0x59503358u /* 'YP3X' */

struct codec_xip_desc
{
    uint32_t magic;
    uint32_t dataload;      /* .data initialiser image, in flash */
    uint32_t data_start;    /* .data destination, in codecbuf */
    uint32_t data_end;
};

#define YP3_CODEC_SLOT(name, base, size) { name, (base) },
static const struct { const char *name; uint32_t base; } codec_slots[] =
{
    YP3_CODEC_SLOT_LIST
};
#undef YP3_CODEC_SLOT

/* "/.rockbox/codecs/mpa.codec" -> "mpa", or NULL if this path is not a codec.
 * Matching on the path rather than on a name passed down keeps apps/codecs.c
 * stock; it is the only thing lc_open() is given. */
static const char * slot_name_of(const char *filename, char *buf, size_t bufsz)
{
    static const char dir[] = CODECS_DIR "/" CODEC_PREFIX;
    static const char ext[] = "." CODEC_EXTENSION;
    size_t len;

    if (strncmp(filename, dir, sizeof(dir) - 1) != 0)
        return NULL;

    filename += sizeof(dir) - 1;
    len = strlen(filename);

    if (len < sizeof(ext) || strcmp(filename + len - (sizeof(ext) - 1), ext))
        return NULL;

    len -= sizeof(ext) - 1;
    if (len >= bufsz)
        return NULL;

    memcpy(buf, filename, len);
    buf[len] = '\0';
    return buf;
}

/* Returns the resident codec's header. Sets *claimed when the name matched a
 * slot, whether or not the open succeeded, so the caller can tell "not one of
 * ours, try the card" from "ours, and broken". */
static void * lc_open_resident(const char *name, unsigned char *buf,
                               size_t buf_size, bool *claimed)
{
    for (unsigned i = 0; i < ARRAYLEN(codec_slots); i++)
    {
        const struct codec_xip_desc *d;

        if (strcmp(codec_slots[i].name, name) != 0)
            continue;

        *claimed = true;
        d = (const struct codec_xip_desc *)codec_slots[i].base;

        /* A slot that was never packed reads as erased flash. Say so rather
         * than jumping into it: this is the failure mode when the image and
         * tools/codec_slots.txt have drifted apart. */
        if (d->magic != CODEC_XIP_MAGIC)
        {
            logf("lc_open: %s slot %08lx bad magic %08lx", name,
                 (unsigned long)codec_slots[i].base, (unsigned long)d->magic);
            return NULL;
        }

        /* The descriptor is flash written at install time and the buffer is
         * sized by this build; a mismatch means the two disagree about
         * CODEC_SIZE, which would otherwise be a silent overrun. */
        if (d->data_end < d->data_start
            || (unsigned char *)d->data_start != buf
            || (unsigned char *)d->data_end > buf + buf_size)
        {
            logf("lc_open: %s data %08lx..%08lx outside %08lx+%lx", name,
                 (unsigned long)d->data_start, (unsigned long)d->data_end,
                 (unsigned long)buf, (unsigned long)buf_size);
            return NULL;
        }

        if (d->data_end > d->data_start)
            memcpy((void *)d->data_start, (const void *)d->dataload,
                   d->data_end - d->data_start);

        commit_discard_idcache();

        /* Log the hit, not just the failures. The line immediately after this
         * one in the ring is codec_load_ram's "calling entrypoint", so a log
         * that stops between the two says the codec was entered and did not
         * come back - which is the difference between a loader fault and a
         * fault inside the codec, and the log ring survives a hang plus reset
         * in the black box. */
        logf("lc_open: %s resident %08lx, .data %lu, hdr %08lx", name,
             (unsigned long)codec_slots[i].base,
             (unsigned long)(d->data_end - d->data_start),
             (unsigned long)(d + 1));

        /* The header follows the descriptor; see the .codecxip and .header
         * output sections in plugin.lds. */
        return (void *)(d + 1);
    }

    return NULL;
}
#endif /* HAVE_CODEC_XIP */

void * lc_open(const char *filename, unsigned char *buf, size_t buf_size)
{
#ifdef HAVE_CODEC_XIP
    char name[16];
    const char *codec = slot_name_of(filename, name, sizeof(name));

    if (codec != NULL)
    {
        bool claimed = false;
        void *handle = lc_open_resident(codec, buf, buf_size, &claimed);

        if (handle != NULL)
            return handle;
        if (claimed)
            return NULL;   /* ours and broken - see the note at the top */
    }
#endif

    return lc_open_from_file(filename, buf, buf_size);
}
