#ifndef INC_LIB_UI_CHANNEL_H_
#define INC_LIB_UI_CHANNEL_H_

#include "ux_setup.h"
#include "ux_state.h"

// The interaction half of a channel: what its button and its encoder do, which
// depends on the active mode. Split from channel.c, which is now signal path
// only - the two were one 500-line file holding DSP, UX dispatch, config
// mutation and a driver call.

void ui_channel_update(const ChannelSetup* ch, UxState* state);

// The hue that stands for a frequency parameter value. A ratio is not a
// magnitude, so the renderer shows it as a coded colour rather than as a level.
//
// Derived from the stored value on every frame rather than cached when the
// encoder moves: the cache was only ever filled for the channel being turned,
// so anything that lit all eight at once painted the untouched ones with hue
// zero - red - and that is what flashed the whole row on leaving a mode.
uint8_t ui_channel_freq_hue(int16_t value);

#endif /* INC_LIB_UI_CHANNEL_H_ */
