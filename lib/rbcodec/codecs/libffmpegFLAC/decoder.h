#ifndef _FLAC_DECODER_H
#define _FLAC_DECODER_H

#include "bitstream.h"

/* Maximum supported channels, only left/right will be played back.
 *
 * Every channel costs MAX_BLOCKSIZE * 4 bytes of codec .bss whether or not a
 * file uses it. A target whose codec buffer cannot hold seven may define
 * FLAC_STEREO_ONLY in its config: streams with more than two channels are
 * then refused when STREAMINFO is read (flac.c), instead of downmixed. */
#ifdef FLAC_STEREO_ONLY
#define MAX_CHANNELS   2
#else
#define MAX_CHANNELS   7      /* Maximum supported channels, only left/right will be played back */
#endif
#ifndef MAX_BLOCKSIZE
#define MAX_BLOCKSIZE  4608   /* Maxsize in samples of one uncompressed frame */
#endif
#define MAX_FRAMESIZE  65536  /* Maxsize in bytes of one compressed frame */
#define MIN_FRAME_SIZE 11     /* smallest valid FLAC frame possible */

#define FLAC_OUTPUT_DEPTH 29 /* Provide samples left-shifted to 28 bits+sign */

enum decorrelation_type {
    INDEPENDENT,
    LEFT_SIDE,
    RIGHT_SIDE,
    MID_SIDE,
};

typedef struct FLACContext {
    GetBitContext gb;

    int min_blocksize, max_blocksize;
    int min_framesize, max_framesize;
    int samplerate, channels;
    int blocksize/*, last_blocksize*/;
    int bps;
    unsigned long samplenumber;
    unsigned long totalsamples;
    enum decorrelation_type ch_mode;

    int filesize;
    int length;
    int bitrate;
    int metadatalength;

    int bitstream_size;
    int bitstream_index;

    int sample_skip;
    int framesize;

    int32_t *decoded[MAX_CHANNELS];
} FLACContext;

int flac_decode_frame(FLACContext *s,
                      uint8_t *buf, int buf_size,
                      void (*yield)(void)) ICODE_ATTR_FLAC;

#endif
