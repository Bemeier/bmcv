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

/* ---- driving another module --------------------------------------------- */
//
// The setters above are this instance's own panel. These fill a *mailbox* to be
// written into a different module's memory - the write direction of the debug
// probe bridge, where the page in front of you drives the board on the bench.
// See RemoteInput in Core/Inc/Lib/input_fold.h for what the far end does with
// it, and docs/plans/remote-input.md for why it is shaped this way.
//
// Nothing here touches this instance. A host drives one or the other.
//
// The bytes are built here rather than in the caller for the same reason
// bmcv_sim_import() decodes a snapshot here: a JS frontend that laid out this
// struct itself would be a second definition of it, free to drift from the one
// the firmware compiles.

void bmcv_sim_remote_button(BmcvSim* s, int32_t button, int32_t down);

// Detents to add, exactly like bmcv_sim_add_encoder. The absolute position this
// accumulates into is an origin of this host's choosing - the far end reads
// only how far it moves - so it may wrap and never has to match anything.
void bmcv_sim_remote_encoder(BmcvSim* s, int32_t encoder, int32_t detents);

// 0.0..1.0 to take the far module's crossfader, negative to hand it back. The
// claim is sticky: it holds until handed back, or until the physical fader is
// moved far enough to take itself back.
void bmcv_sim_remote_slider01(BmcvSim* s, float pos01);

// Let go of everything - buttons up, crossfader released, encoder origin reset.
// For entering and leaving a session, so a press held when a connection ended
// is not still held when the next one starts.
void bmcv_sim_remote_clear(BmcvSim* s);

// Where the mailbox sits inside a BmcvInstance, and how big it is. The layout
// assertions in layout_target.h are what make this offset the *target's*
// offset, so a host that has read an instance's address knows where to write
// without the firmware publishing anything further.
int32_t bmcv_sim_remote_offset(void);
int32_t bmcv_sim_remote_size(void);

// The mailbox as bytes, stamped with a fresh sequence number. Call it for every
// write, including ones that change nothing: the far end treats the sequence as
// a heartbeat and stops believing a mailbox that has gone quiet.
//
// A writer that cannot deliver all of it at once must send the last four bytes
// - the sequence number - after the rest.
const void* bmcv_sim_remote_blob(BmcvSim* s);

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

// Measured loop rates, as the module computes them. Only the first is a number
// every host has.
//
// engine_fps is measured inside engine_tick (see engine.c), so it exists
// wherever the engine runs. In the simulator it is driven by bmcv_sim_run's
// dt_us and reports what the host asked for rather than what a board achieves -
// useful as a check that a frontend's timing maths is right.
//
// dac_fps and led_fps are **zero in the simulator and always will be**. They
// measure peripherals rather than loops: the firmware's DAC service pass and
// the flush to the WS2811, which is gated on that driver's DMA being free.
// Neither peripheral exists off the board, so neither number can be invented
// here - which is the point. Read over a debug probe they are the two worth
// having, and led_fps most of all: it is the ceiling on how fast anything drawn
// on the panel can actually move, and the only thing that says how many frames
// a busy transfer is dropping.
float bmcv_sim_engine_fps(const BmcvSim* s);
float bmcv_sim_dac_fps(const BmcvSim* s);
float bmcv_sim_led_fps(const BmcvSim* s);
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

// Where the panel is, as the engine last saw it - the folded hardware frame,
// not the sample a host most recently pushed in.
//
// A frontend that draws its own controls from its own bookkeeping is keeping a
// second copy of the panel's position, and the two part company the moment
// anything but that frontend moves it: a snapshot from a physical module has
// its encoders wherever someone left them, and a drawing that tracks only
// mouse drags would sit at zero and stay there. Reading them back means the
// drawn panel follows the module, whichever one it is.
//
// Encoder position is free-running and wraps, exactly as the hardware's does;
// what it is good for is the difference between two of them. The slider comes
// back as 0..1 over its travel, matching bmcv_sim_set_slider01 - though not
// exactly through it, since an input in slider mode sums CV into what the
// engine reads.
int32_t bmcv_sim_encoder_pos(const BmcvSim* s, int32_t encoder);
float bmcv_sim_slider01(const BmcvSim* s);

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

/* ---- snapshots ----------------------------------------------------------- */
//
// The whole running module as a flat blob: not the preset store below, which is
// what survives a power cycle, but everything - phases, LED framebuffer, clock,
// interaction state, the lot.
//
// This exists because a physical module's is reachable. The firmware keeps its
// one instance in a single global at a fixed address (`bmcv`, see bmcv.h), so a
// debug probe can read those bytes out of RAM while the module plays and hand
// them here. Every accessor above then reports the hardware instead of the
// simulation, using the firmware's own code to interpret its own struct - no
// duplicate decoder, and nothing for a frontend to know about the layout.
//
// The blob is a BmcvInstance exactly as this build lays it out, so it is only
// interchangeable with a module whose build agrees byte for byte.
// sim/include/layout_target.h is what holds the two together, and `just
// layout-check` is what regenerates it; the size below is the cheap first
// check, and a caller talking to real hardware should confirm the firmware
// version as well before believing any of it.

int32_t bmcv_sim_instance_size(void);

// Copy this module's state out. `dst` needs bmcv_sim_instance_size() bytes.
void bmcv_sim_export(const BmcvSim* s, void* dst);

// Adopt a snapshot: the state becomes what the blob says, the pointers inside
// it are re-pointed at this instance, and every published reading is recomputed
// from it - so the effect is visible without ticking, which is the point, since
// a snapshot of a running module must not be advanced by a host.
//
// Returns 1 if it was taken, 0 if `len` is not the instance size. A blob that
// passes the length check but came from a build with a different layout cannot
// be detected here and will read as plausible nonsense; see above.
//
// The simulator's clock follows the snapshot's own timestamp, so
// bmcv_sim_now_us() reports where the module that produced it had got to rather
// than how long this process has been running.
//
// Running an imported instance is allowed and is how a module gets cloned into
// the simulator to carry on playing. The panel positions this host last set are
// re-baselined onto the ones that arrived, so the first tick after an import
// sees no input - without that, importing a module whose encoder sits forty
// detents from this host's would apply forty detents of edit. Volts on the
// input jacks stay the host's, because those are its patch and not the
// module's.
int32_t bmcv_sim_import(BmcvSim* s, const void* src, int32_t len);

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
