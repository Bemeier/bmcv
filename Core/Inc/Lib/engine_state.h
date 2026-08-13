#ifndef INC_LIB_ENGINE_STATE_H_
#define INC_LIB_ENGINE_STATE_H_

#include "clock_sync.h"
#include "hw_setup.h"
#include "led_curve.h" // IWYU pragma: keep - LED_UNIT is the unit of the fields below
#include <stdint.h>

// The engine's running state: phases, output levels, trigger edges, the scene
// blend, the LED framebuffer. None of it is persisted and all of it is
// reproducible from the config plus the input stream.

// 8.8 fixed point duty per primary - LED_UNIT is one step of what the WS2812
// takes. The framebuffer holds the colour that was meant and the flush is where
// it becomes eight bits, so the fractional part survives long enough to be
// dithered. See led_curve.h.
typedef struct
{
  uint16_t r, g, b;
} LedRgb;

typedef struct
{
  float freq_hz;      // oscillator rate in Hz
  float freq_ratio;   // multiple of the beat rate that produced it
  float phase;        // 0..1, after the phase-shift parameter
  float phase_offset; // the phase-shift parameter itself, in turns
  float shape;        // -1..1
  float mod;          // -1..1
  float amp;          // peak swing, DAC units
  float offset;       // DC offset, DAC units
  int16_t gcd;        // cycle length in beats the phase locks to, 0 if free

  // How far this channel is from where the clock says it should be, in cycles
  // of its own waveform, wrapped into +/- half a super-period. Zero when there
  // is nothing to lock to (no beat, or a ratio with no whole-beat period).
  //
  // Published because it is the quantity the sync loop is actually minimising,
  // and tuning that loop means measuring it - tests/test_pll.c does, and
  // re-deriving it outside would be measuring something subtly different from
  // what the loop reacts to. Divide by freq_ratio for the same error in beats.
  float phase_error;
} ChannelEffective;

// Signal path only. Interaction and view state live in UiState (ui_state.h);
// the two used to be one struct, which is how DSP code came to read
// shift_state and render code came to mutate timers.
typedef struct
{
  // Scene
  uint8_t scenes_contribution[N_SCENES];

  // Channel
  int16_t channels_output_level[N_CHANNELS];
  float channels_shared_phase[N_CHANNELS];
  float channels_phase_correction[N_CHANNELS];

  // The alignment period the sync loop is working to, in beats, and the beat it
  // counts that period from. Latched rather than recomputed every tick - see
  // channel_compute. 0 means "not taken yet"; -1 is find_denominator's "this
  // ratio has no whole-beat period", which is a latched answer like any other.
  int16_t channels_gcd[N_CHANNELS];
  uint64_t channels_beat_origin[N_CHANNELS];

  // Which cycle of the alignment period the oscillator is on, and how many
  // cycles that period holds.
  //
  // channels_shared_phase[] is the phase within one cycle and wraps at one
  // cycle, because that is the only wrap the output cannot see. The super-period
  // position the sync loop needs is this counter plus that phase - kept apart so
  // that the thing which wraps oddly is a counter nobody hears rather than the
  // phase everybody does.
  int16_t channels_cycle[N_CHANNELS];
  int16_t channels_period_cycles[N_CHANNELS];

  // The ratio as it was when it last stopped moving, and when that was. A new
  // alignment period is only taken once the ratio has held still, so a
  // crossfade does not acquire and discard one every few ticks on its way past.
  float channels_ratio_seen[N_CHANNELS];
  uint32_t channels_ratio_still_since[N_CHANNELS];

  // Stepped-random pattern length actually in use, held steady for the rest
  // of the cycle. Changing it mid-cycle shifts the step grid under the
  // playhead and jumps the output; -1 means "not yet latched".
  int8_t channels_length_idx[N_CHANNELS];
  float channels_prev_phase[N_CHANNELS]; // to spot the cycle wrap

  // Channel output trigger detection (edge state carried between ticks)
  int16_t channels_prev_out[N_CHANNELS];
  uint8_t channels_trig_state[N_CHANNELS];
  uint8_t channels_trig_flag[N_CHANNELS];

  // Timestamp of the last encoder movement on this channel. Written only by
  // ux_note_channel_edit() - channel_compute reads it to decide whether a
  // stepped-pattern length change applies immediately or waits for the cycle
  // wrap, which is the one place the DSP needs to know the user is fiddling.
  uint32_t channels_last_delta[N_CHANNELS];

  // Ramped output gate, 0..1, slewed toward UiState.muted[] once per tick so
  // muting does not click.
  float channels_mute_gain[N_CHANNELS];

  // What actually leaves the module: channels_output_level[] through the mute
  // gain. Mute is an output-stage gain rather than a zeroed output level, so a
  // muted channel still cross-modulates and still triggers - which means the
  // two differ and a host must read the right one.
  //
  // Published by engine_tick. It used to be returned by a function that
  // advanced the ramp as a side effect, so a host that never called it got
  // ungated audio and a mute ramp frozen in place.
  int16_t channels_gated_level[N_CHANNELS];

  // What each channel is actually doing, after the scene crossfade and the
  // parameter maths: the stored per-scene numbers say what was dialled in, this
  // says what is coming out. Written by channel_compute every tick and read by
  // the UI and by hosts; nothing in the signal path depends on it.
  ChannelEffective channels_effective[N_CHANNELS];

  // Which scene currently dominates the crossfade. Derived from the slider
  // (and the momentary scene the UI passes in), so it is an engine output the
  // UI reads, not UI state.
  int8_t active_scene;

  // LED framebuffer. The UX layer renders into this; a flush step pushes it
  // to the driver. Keeps LED behaviour assertable without any hardware.
  LedRgb leds[LED_COUNT];

  // Tempo/phase tracking for the external clock input. Per instance rather
  // than a global so two simulated modules can run at different tempos.
  ClockState clock;

  // Error codes, one bit per scene button, drawn by ui_render as an overlay.
  // Was a file static in error.c for the same reason the clock was global.
  uint8_t error_flags;

  float engine_fps;
  float dac_fps;

  // How often the framebuffer actually reaches the LED driver. Not the same as
  // the rate it is rendered at: the flush is gated on the WS2811 DMA being
  // free, so a busy transfer drops frames silently and this is the only thing
  // that says how many. Anything on the panel that moves - the FRQ pulse most
  // of all - can only be as fast as this number allows.
  float led_fps;

} EngineState;

#endif /* INC_LIB_ENGINE_STATE_H_ */
