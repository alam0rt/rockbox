/***************************************************************************
 * Doom as a built-in, for targets where it cannot be a loadable plugin.
 *
 * 228 KB of engine code will not fit a 64 KB plugin buffer, but on an XIP
 * target it does not have to be loaded at all: it is linked into the firmware
 * image and executes from flash. What is left is what a plugin loader would
 * have done anyway - place the writable data and hand over a heap - and that is
 * this file.
 *
 * The writable half lives in an overlay on the codec and plugin buffers
 * (app.lds, .doomdata/.doombss), which are free the moment playback stops.
 * .data is copied in from its flash image on every launch, so a second game
 * starts from the same state as the first.
 ****************************************************************************/
#include "plugin.h"

/* the pointer every doom source reaches the core through */
const struct plugin_api *rb;

/* placed by app.lds */
extern unsigned char __doom_data_start[], __doom_data_end[], __doom_data_load[];
extern unsigned char __doom_bss_start[], __doom_bss_end[];

enum plugin_status doom_builtin_main(const void *parameter)
{
    rb = plugin_get_api();

    /* The overlay is the codec buffer plus the plugin buffer. The codec buffer
     * is only ours once nothing is decoding into it. */
    rb->audio_stop();

    rb->memcpy(__doom_data_start, __doom_data_load,
               __doom_data_end - __doom_data_start);
    rb->memset(__doom_bss_start, 0, __doom_bss_end - __doom_bss_start);

    return plugin_start(parameter);
}
