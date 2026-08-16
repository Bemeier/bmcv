#ifndef INC_LIB_UI_CHANNEL_H_
#define INC_LIB_UI_CHANNEL_H_

#include "color_presets.h"
#include "ux_setup.h"
#include "ux_state.h"

// The interaction half of a channel: what its button and its encoder do, which
// depends on the active mode. Split from channel.c, which is now signal path
// only - the two were one 500-line file holding DSP, UX dispatch, config
// mutation and a driver call.

void ui_channel_update(const ChannelSetup* ch, UxState* state);

// The colour that stands for a frequency parameter value. A ratio is not a
// magnitude, so the renderer shows it as a coded colour rather than as a level.
//
// Two facts on two axes. Hue is the ratio's prime limit - straight, triplet or
// quintuplet, see HUE_FREQ_* - because that is what makes one division feel
// different from another. Saturation is how far the value sits off the grid,
// full for a snapped ratio and washing out to a pastel between two of them, so
// a fine adjust is visible as one.
//
// `.v` is the caller's: the renderer pulses it at the channel's output rate.
//
// Derived from the stored value on every frame rather than cached when the
// encoder moves: the cache was only ever filled for the channel being turned,
// so anything that lit all eight at once painted the untouched ones with hue
// zero - red - and that is what flashed the whole row on leaving a mode.
UiColor ui_channel_freq_color(int16_t value);

// The division hue for a whole number of parts: its prime limit once the
// octaves are divided out, as HUE_FREQ_*. Shared with the stepped
// pattern length, which is a division of the beat in exactly the same sense -
// a 12-step pattern subdivides in threes, and wears what a triplet wears.
uint8_t ui_division_hue(uint32_t n);

#endif /* INC_LIB_UI_CHANNEL_H_ */
