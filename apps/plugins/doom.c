/***************************************************************************
 * Doom launcher for targets that build the engine into the core image.
 *
 * Doom is 228 KB of code. On a target whose entire plugin buffer is 64 KB it
 * cannot be loaded, but it can be executed in place from flash - so the engine
 * is linked into the firmware and this stub is all that gets loaded. The
 * browser still sees a doom.rock in rocks/games and starts it the usual way.
 ****************************************************************************/
#include "plugin.h"

enum plugin_status plugin_start(const void *parameter)
{
    return rb->doom_builtin_main(parameter);
}
