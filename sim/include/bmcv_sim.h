#ifndef BMCV_SIM_H_
#define BMCV_SIM_H_

#include <stdint.h>

// A whole BMCV behind a flat C API.
//
// Everything behind this runs the firmware's own code - the same engine_tick,
// the same UX layer, the same LED renderer - so what the sim does is what the
// module does. The API is deliberately flat and free of firmware types so a
// JS frontend can call it across the wasm boundary without knowing any of
// them.
//
// Inputs are in the units a person thinks in (volts, 0..1 slider, encoder
// detents); the conversion to converter counts happens inside.

#ifdef __cplusplus
extern "C"
{
#endif

#define BMCV_SIM_CHANNELS 8
#define BMCV_SIM_INPUTS 4
#define BMCV_SIM_BUTTONS 24
#define BMCV_SIM_ENCODERS 8
#define BMCV_SIM_LEDS 21

// Per-channel scope history, in frames. Power of two: the ring wraps by mask.
#define BMCV_SIM_SCOPE_LEN 4096

typedef struct BmcvSim BmcvSim;

BmcvSim* bmcv_sim_create(void);
void bmcv_sim_destroy(BmcvSim* s);

// Back to power-on, keeping stored presets. Pass 1 to wipe those too.
void bmcv_sim_reset(BmcvSim* s, int32_t wipe_storage);

/* ---- input -------------------------------------------------------------- */

void bmcv_sim_set_button(BmcvSim* s, int32_t button, int32_t down);

// Encoders are relative: add detents, positive is clockwise. The absolute
// position the firmware sees is free-running and wraps, exactly as on hardware.
void bmcv_sim_add_encoder(BmcvSim* s, int32_t encoder, int32_t detents);

// 0.0 = bottom of travel, 1.0 = top.
void bmcv_sim_set_slider01(BmcvSim* s, float pos01);

// CV in volts on an input *jack* (0..3), not a converter channel. Gate edges
// are latched with the hardware's hysteresis, so a pulse shorter than one tick
// is still seen.
void bmcv_sim_set_cv(BmcvSim* s, int32_t input, float volts);

// Fire a single trigger on an input jack without synthesising a voltage ramp.
void bmcv_sim_fire_gate(BmcvSim* s, int32_t input);

/* ---- running ------------------------------------------------------------ */

// The most engine time one call may advance, as a tick count. Nothing
// legitimate comes close: at the default 250us tick this is over 16 seconds.
#define BMCV_SIM_MAX_TICKS 65536

// Advance the module by n_ticks of dt_us each. dt_us is the engine's tick
// period - 250 (4kHz) is a reasonable default and is roughly what the hardware
// achieves. The engine is dt-driven, so any value is correct.
//
// Both counts are signed and validated: zero or negative does nothing, and
// n_ticks is capped at BMCV_SIM_MAX_TICKS. This is the boundary a host calls
// across, and a host that computes a bad count must not be able to wedge
// itself - an unsigned n_ticks turned a JavaScript -20, which a frame timer
// produces the moment a timestamp goes backwards, into 4.29 billion ticks and
// a permanently frozen browser tab.
void bmcv_sim_run(BmcvSim* s, int32_t dt_us, int32_t n_ticks);

uint32_t bmcv_sim_now_us(const BmcvSim* s);

/* ---- output ------------------------------------------------------------- */
//
// These return pointers into the sim's own memory, valid until the next call
// that mutates it. A wasm frontend wraps each once with a typed-array view and
// re-reads it every frame, so a full render costs a handful of calls no matter
// how fast the module is ticking.

// 8 floats, volts.
const float* bmcv_sim_outputs_v(const BmcvSim* s);

// 63 entries, r,g,b per LED in chain order, in the framebuffer's own 8.8 fixed
// point (LED_UNIT per duty step) - not the driver's bytes. A display wants the
// colour that was meant, at full precision; see led_curve.h.
const uint16_t* bmcv_sim_leds_rgb(const BmcvSim* s);

// 8 * BMCV_SIM_SCOPE_LEN floats in volts, channel-major. Index
// [ch * BMCV_SIM_SCOPE_LEN + i]; the newest sample is at head-1 (mod len).
const float* bmcv_sim_scope(const BmcvSim* s);
uint32_t bmcv_sim_scope_head(const BmcvSim* s);

// The same history for the 4 CV inputs, sharing bmcv_sim_scope_head(). This is
// what the engine actually saw - the value latched into HwState each tick, not
// the voltage the host last set - so a gate too short to survive a tick shows
// up as nothing here, exactly as the module experienced it.
const float* bmcv_sim_input_scope(const BmcvSim* s);

/* ---- introspection ------------------------------------------------------ */

int32_t bmcv_sim_shift_state(const BmcvSim* s);

// The three-letter name of a shift mode ("STA", "QNT", ... and "---" for none),
// straight out of the firmware's own mode table. Exposed because that list was
// otherwise retyped in the CLI, in tools/gen_panel_spec.py and in the web
// frontend, and four copies of a table can disagree. Returns NULL out of range.
// Valid for the lifetime of the process; it is static storage in the firmware.
const char* bmcv_sim_mode_name(int32_t shift_state);
int32_t bmcv_sim_mode_count(void);

// Measured loop rates, as the firmware computes them: the engine tick rate and
// the DAC service rate. In the simulator both are driven by bmcv_sim_run's
// dt_us, so they report what the host asked for rather than what a board
// achieves - useful as a check that a frontend's timing maths is right.
float bmcv_sim_engine_fps(const BmcvSim* s);
float bmcv_sim_dac_fps(const BmcvSim* s);
int32_t bmcv_sim_selected_param(const BmcvSim* s);
int32_t bmcv_sim_active_scene(const BmcvSim* s);

// Two different tempos, and they are only equal while a clock is locked.
// bpm is what the clock input measured; active_bpm is the rate the oscillators
// are actually running against, which free-runs at the last measured tempo once
// the pulses stop. have_beat says whether the first is still live.
float bmcv_sim_bpm(const BmcvSim* s);
float bmcv_sim_active_bpm(const BmcvSim* s);
int32_t bmcv_sim_have_beat(const BmcvSim* s);
int32_t bmcv_sim_scene_contribution(const BmcvSim* s, int32_t scene);
int32_t bmcv_sim_channel_muted(const BmcvSim* s, int32_t channel);
int32_t bmcv_sim_error_flags(const BmcvSim* s);

// One channel parameter (CH_PARAM_FRQ..CH_PARAM_OFS) in the active scene: the
// number that was dialled in, not the one in use.
int32_t bmcv_sim_channel_param(const BmcvSim* s, int32_t channel, int32_t param);

// The channel's waveshape mode, and the name of one. Not per scene: shape is a
// channel property. Names are here rather than in a frontend for the same
// reason the mode names are - one table, asserted against the enum it came from.
int32_t bmcv_sim_channel_shape_mode(const BmcvSim* s, int32_t channel);
const char* bmcv_sim_shape_mode_name(int32_t mode);

// What each channel is *actually* doing, after the scene crossfade and the
// parameter maths, in units a person can read. 8 * BMCV_EFF_COUNT floats,
// channel-major: index [channel * BMCV_EFF_COUNT + field].
enum
{
  BMCV_EFF_FREQ_HZ,    // oscillator rate
  BMCV_EFF_FREQ_RATIO, // multiple of the beat rate
  BMCV_EFF_PHASE,      // 0..1, where the oscillator is right now
  BMCV_EFF_SHAPE,      // -1..1
  BMCV_EFF_MOD,        // -1..1
  BMCV_EFF_AMP_V,      // peak swing, volts
  BMCV_EFF_OFFSET_V,   // DC offset, volts
  BMCV_EFF_GCD,        // cycle length in beats the phase locks to, 0 if free
  BMCV_EFF_PHASE_OFS,  // the phase-shift parameter, in turns
  BMCV_EFF_COUNT,
};
const float* bmcv_sim_effective(const BmcvSim* s);

/* ---- midi --------------------------------------------------------------- */
//
// What the module is saying on the MIDI bus: the eight channel outputs and four
// CV inputs as control changes, plus a clock. Decided by the firmware's own
// midi_out.c, so a host only has to carry the bytes.

// One drained message: status, data 1, data 2, and how many of those three are
// real - 1 for a System Real-Time byte, 3 for a control change.
#define BMCV_SIM_MIDI_MSG_BYTES 4

// Take up to `max_msgs` queued messages into `dst`, which needs
// max_msgs * BMCV_SIM_MIDI_MSG_BYTES of room. Returns the number of messages
// written, so a caller iterates `dst` with a stride of four and needs no MIDI
// parser of its own.
//
// A host that never calls this simply never sends anything: the queue drops its
// oldest-unsent when full and the values re-state themselves on the next slot.
int32_t bmcv_sim_midi_drain(BmcvSim* s, void* dst, int32_t max_msgs);

/* ---- persistence -------------------------------------------------------- */
//
// The whole preset store as an opaque blob, for a host to keep in a patch file
// or in browser storage. It is the simulator's own slot array, not the FRAM
// record format, so it is not interchangeable with a chip dump. A host should
// reject a blob whose length differs from bmcv_sim_storage_size(); anything
// that gets past that is still run through config_validate() on the next boot.

int32_t bmcv_sim_storage_size(void);
void bmcv_sim_storage_get(const BmcvSim* s, void* dst);
int32_t bmcv_sim_storage_set(BmcvSim* s, const void* src, int32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BMCV_SIM_H_ */
