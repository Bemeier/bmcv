#ifndef INC_LIB_CHANNEL_H_
#define INC_LIB_CHANNEL_H_

#include "config.h"
#include "engine_state.h"
#include "hw_state.h"
#include <stdint.h>

// The signal path for one channel. Everything here takes exactly the state it
// touches - the config it reads, the engine state it writes, the hardware
// frame it samples - and a plain channel index.
//
// It used to take UxState*, which is the whole module: the interaction layer,
// the LED setup tables and the preset vtable came along with it, and nothing
// structural stopped a DSP function from reading shift_state. Narrow
// parameters are what actually enforce the split the layering describes, and
// they are what an audio thread in a plugin host wants to be handed.

// Zero the oscillator state and open the mute gate. Called at power-on and
// whenever a channel is reset.
void channel_init(uint8_t ch, EngineState* es);

// Back to phase zero, leaving everything else alone. What a reset input does.
void channel_reset_phase(uint8_t ch, EngineState* es);

// Restore one parameter, or a whole channel, to its default value.
// `scene` < 0 means every scene.
void channel_reset_param(uint8_t ch, EngineConfig* cfg, int8_t scene, int8_t param);
void channel_reset(uint8_t ch, EngineState* es, EngineConfig* cfg, int8_t scene);

// One tick of this channel: blend the scene parameters, advance the phase,
// lock it to the beat, generate, cross-modulate, quantize. Lands in
// es->channels_output_level[] and es->channels_effective[].
void channel_compute(uint8_t ch, EngineState* es, const EngineConfig* cfg, const HwState* hw);

// Update this channel's own output-trigger edge state, so other channels can
// use it as a trigger source.
void channel_detect_trigger(uint8_t ch, EngineState* es);

// Advance the output mute ramp and publish es->channels_gated_level[], which
// is what actually leaves the module.
//
// Mute is an output-stage gain, not a zeroed channels_output_level, so a muted
// channel still cross-modulates and still triggers - the two levels genuinely
// differ. engine_tick calls this once per tick; a host reads
// channels_gated_level[] and needs to know nothing else.
void channel_apply_mute(uint8_t ch, EngineState* es, uint8_t muted, uint32_t dt_us);

// Consume this channel's pending output trigger, if any.
uint8_t channel_take_trig(uint8_t ch, EngineState* es);

#endif /* INC_LIB_CHANNEL_H_ */
