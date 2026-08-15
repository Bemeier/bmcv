#include "bmcv_sim.h"
#include "channel.h"
#include "clock_sync.h"
#include "config.h"
#include "engine_state.h"
#include "error.h"
#include "hw_setup.h"
#include "input_fold.h"
#include "instance.h"
// Asserts that BmcvInstance has the same shape here that it has in the module's
// RAM, so a raw snapshot read over a debug probe decodes with these accessors.
// Generated from the firmware ELF; `just layout-check`.
#include "layout_target.h" // IWYU pragma: keep
#include "midi_out.h"
#include "presets.h"
#include "sim_rt.h"
#include "slot_store.h"
#include "ui_mode.h"
#include "ui_state.h"
#include "ux_setup.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// Compile-time agreement between the flat API and the firmware's constants.
// If a count ever changes, this fails here rather than silently truncating a
// frontend's array.
_Static_assert(BMCV_SIM_CHANNELS == N_CHANNELS, "channel count");
_Static_assert(BMCV_SIM_INPUTS == N_INPUTS, "input count");
_Static_assert(BMCV_SIM_BUTTONS == N_BUTTONS, "button count");
_Static_assert(BMCV_SIM_ENCODERS == N_ENCODERS, "encoder count");
_Static_assert(BMCV_SIM_LEDS == LED_COUNT, "led count");

struct BmcvSim
{
  BmcvInstance m;

  InputSample sample;
  SimTrigLatch trig;

  // Outgoing, for a module that is not this one - see the header. Deliberately
  // not s->m.input.remote: that field belongs to whatever instance is loaded
  // here, and in probe mode every snapshot overwrites it.
  RemoteInput remote_out;
  RemoteCommand command_out;

  SlotStore storage;
  PresetIo io;

  uint32_t now_us;

  float outputs_v[N_CHANNELS];
  uint16_t leds_rgb[LED_COUNT * 3];

  float effective[N_CHANNELS * BMCV_EFF_COUNT];
  float scope[N_CHANNELS * BMCV_SIM_SCOPE_LEN];
  float input_scope[N_INPUTS * BMCV_SIM_SCOPE_LEN];
  uint32_t scope_head;
};

static void sim_boot(BmcvSim* s)
{
  slot_store_init(&s->storage, &s->io);

  s->now_us = 0;
  memset(&s->sample, 0, sizeof(s->sample));
  sim_trig_reset(&s->trig);

  bmcv_instance_init(&s->m, &s->io, 0);

  sim_input_slider(&s->sample, 0.0f);
  bmcv_sim_remote_clear(s);
}

BmcvSim* bmcv_sim_create(void)
{
  BmcvSim* s = (BmcvSim*) calloc(1, sizeof(BmcvSim));
  if (!s)
    return NULL;
  sim_boot(s);
  return s;
}

void bmcv_sim_destroy(BmcvSim* s) { free(s); }

void bmcv_sim_reset(BmcvSim* s, int32_t wipe_storage)
{
  if (!s)
    return;
  if (wipe_storage)
    slot_store_clear(&s->storage);

  memset(s->scope, 0, sizeof(s->scope));
  memset(s->input_scope, 0, sizeof(s->input_scope));
  memset(s->effective, 0, sizeof(s->effective));
  s->scope_head = 0;
  memset(s->outputs_v, 0, sizeof(s->outputs_v));
  memset(s->leds_rgb, 0, sizeof(s->leds_rgb));
  sim_boot(s);
}

/* ---- input -------------------------------------------------------------- */

void bmcv_sim_set_button(BmcvSim* s, int32_t button, int32_t down)
{
  if (!s || button < 0 || button >= N_BUTTONS)
    return;
  s->sample.button_down[button] = down ? 1 : 0;
}

void bmcv_sim_add_encoder(BmcvSim* s, int32_t encoder, int32_t detents)
{
  if (!s || encoder < 0 || encoder >= N_ENCODERS)
    return;
  // Free-running and allowed to wrap; input_state computes a wrap-safe delta.
  s->sample.encoder_pos[encoder] = (int16_t) (s->sample.encoder_pos[encoder] + detents);
}

void bmcv_sim_set_slider01(BmcvSim* s, float pos01)
{
  if (!s)
    return;
  sim_input_slider(&s->sample, pos01);
}

void bmcv_sim_set_cv(BmcvSim* s, int32_t input, float volts)
{
  if (!s || input < 0 || input >= N_INPUTS)
    return;
  sim_input_cv(&s->sample, &s->trig, s->m.hw_setup, (uint8_t) input, volts);
}

void bmcv_sim_fire_gate(BmcvSim* s, int32_t input)
{
  if (!s || input < 0 || input >= N_INPUTS)
    return;
  sim_trig_fire(&s->trig, s->m.hw_setup->input_adc_idx[input]);
}

/* ---- driving another module --------------------------------------------- */

// What a writer needs to hold to, and cannot check for itself: the mailbox goes
// out over a transport that moves whole 32-bit words to word addresses, and the
// sequence number has to be the last of them so it can be sent after the fields
// it describes.
_Static_assert(offsetof(RemoteInput, seq) == sizeof(RemoteInput) - 4, "seq is the last word of the mailbox");
_Static_assert(sizeof(RemoteInput) % 4 == 0, "the mailbox is a whole number of words");
_Static_assert(offsetof(BmcvInstance, input.remote) % 4 == 0, "the mailbox starts on a word");
_Static_assert(offsetof(RemoteCommand, seq) == sizeof(RemoteCommand) - 4, "seq is the last word of the command");
_Static_assert(sizeof(RemoteCommand) % 4 == 0, "the command is a whole number of words");
_Static_assert(offsetof(BmcvInstance, command) % 4 == 0, "the command starts on a word");

void bmcv_sim_remote_button(BmcvSim* s, int32_t button, int32_t down)
{
  if (!s || button < 0 || button >= N_BUTTONS)
    return;
  s->remote_out.button_down[button] = down ? 1 : 0;
}

void bmcv_sim_remote_encoder(BmcvSim* s, int32_t encoder, int32_t detents)
{
  if (!s || encoder < 0 || encoder >= N_ENCODERS)
    return;
  s->remote_out.encoder_pos[encoder] = (int16_t) (s->remote_out.encoder_pos[encoder] + detents);
}

void bmcv_sim_remote_slider01(BmcvSim* s, float pos01)
{
  if (!s)
    return;
  s->remote_out.slider_raw = pos01 < 0.0f ? REMOTE_SLIDER_NONE : (int16_t) sim_slider_raw(pos01);
}

void bmcv_sim_remote_clear(BmcvSim* s)
{
  if (!s)
    return;

  // The sequence number survives, so that clearing is itself an update the far
  // end acts on rather than a silence it has to time out. Restarting the count
  // could repeat a value it has already seen and be ignored.
  const uint32_t seq = s->remote_out.seq;
  memset(&s->remote_out, 0, sizeof(s->remote_out));
  s->remote_out.slider_raw = REMOTE_SLIDER_NONE;
  s->remote_out.seq        = seq;
}

void bmcv_sim_remote_reset(BmcvSim* s, int32_t wipe_storage)
{
  if (!s)
    return;

  s->command_out.op = (uint8_t) (wipe_storage ? REMOTE_OP_RESET_WIPE : REMOTE_OP_RESET);

  // Never zero, which is what a module that has never been asked anything
  // holds - so the first command of a session cannot read as silence.
  if (++s->command_out.seq == 0)
    s->command_out.seq = 1;
}

int32_t bmcv_sim_command_offset(void) { return (int32_t) offsetof(BmcvInstance, command); }
int32_t bmcv_sim_command_size(void) { return (int32_t) sizeof(RemoteCommand); }
const void* bmcv_sim_command_blob(const BmcvSim* s) { return s ? &s->command_out : NULL; }

int32_t bmcv_sim_remote_offset(void) { return (int32_t) offsetof(BmcvInstance, input.remote); }
int32_t bmcv_sim_remote_size(void) { return (int32_t) sizeof(RemoteInput); }

const void* bmcv_sim_remote_blob(BmcvSim* s)
{
  if (!s)
    return NULL;

  // Zero means never written, so a count that wraps has to skip it rather than
  // hand the far end a mailbox that reads as untouched.
  if (++s->remote_out.seq == 0)
    s->remote_out.seq = 1;

  return &s->remote_out;
}

/* ---- running ------------------------------------------------------------ */

static void capture(BmcvSim* s)
{
  uint32_t head = s->scope_head;

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    // Not channels_output_level[]: mute is an output-stage gain, so what
    // leaves the module is the gated level engine_tick published. The same
    // array the firmware hands to the DAC.
    float v                                 = sim_dac_to_volts(s->m.engine_state.channels_gated_level[c]);
    s->outputs_v[c]                         = v;
    s->scope[c * BMCV_SIM_SCOPE_LEN + head] = v;
  }

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    // input_state[] is in DAC units (four times the ADC reading), so the same
    // conversion the outputs use applies.
    s->input_scope[i * BMCV_SIM_SCOPE_LEN + head] = sim_dac_to_volts(s->m.ux.hw_state->input_state[i]);
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    const ChannelEffective* e = &s->m.engine_state.channels_effective[c];
    float* out                = &s->effective[c * BMCV_EFF_COUNT];

    out[BMCV_EFF_FREQ_HZ]    = e->freq_hz;
    out[BMCV_EFF_FREQ_RATIO] = e->freq_ratio;
    out[BMCV_EFF_PHASE]      = e->phase;
    out[BMCV_EFF_SHAPE]      = e->shape;
    out[BMCV_EFF_MOD]        = e->mod;
    out[BMCV_EFF_AMP_V]      = sim_dac_to_volts((int16_t) e->amp);
    out[BMCV_EFF_OFFSET_V]   = sim_dac_to_volts((int16_t) e->offset);
    out[BMCV_EFF_GCD]        = (float) e->gcd;
    out[BMCV_EFF_PHASE_OFS]  = e->phase_offset;
  }

  s->scope_head = (head + 1) & (BMCV_SIM_SCOPE_LEN - 1);

  for (uint8_t i = 0; i < LED_COUNT; i++)
  {
    const LedRgb* led = &s->m.engine_state.leds[i];
    // Undithered and unrounded: a screen has no reason to quantise to what a
    // WS2812 takes, and sampling the dither at 60Hz would show its noise rather
    // than the colour it averages to.
    s->leds_rgb[i * 3]     = led->r;
    s->leds_rgb[i * 3 + 1] = led->g;
    s->leds_rgb[i * 3 + 2] = led->b;
  }
}

void bmcv_sim_run(BmcvSim* s, int32_t dt_us, int32_t n_ticks)
{
  if (!s || dt_us <= 0 || n_ticks <= 0)
    return;

  // A host cannot be allowed to hang itself here. See the header.
  if (n_ticks > BMCV_SIM_MAX_TICKS)
    n_ticks = BMCV_SIM_MAX_TICKS;

  for (int32_t i = 0; i < n_ticks; i++)
  {
    // Gate edges are consumed once per tick, whatever rate the host sampled
    // CV at, so a pulse between ticks still lands.
    sim_input_take_trigs(&s->sample, &s->trig);

    s->now_us += dt_us;
    bmcv_instance_tick(&s->m, &s->sample, s->now_us);
    capture(s);
  }
}

uint32_t bmcv_sim_now_us(const BmcvSim* s) { return s ? s->now_us : 0; }

/* ---- output ------------------------------------------------------------- */

const float* bmcv_sim_outputs_v(const BmcvSim* s) { return s->outputs_v; }
const uint16_t* bmcv_sim_leds_rgb(const BmcvSim* s) { return s->leds_rgb; }
const float* bmcv_sim_scope(const BmcvSim* s) { return s->scope; }
const float* bmcv_sim_input_scope(const BmcvSim* s) { return s->input_scope; }
const float* bmcv_sim_effective(const BmcvSim* s) { return s->effective; }
uint32_t bmcv_sim_scope_head(const BmcvSim* s) { return s->scope_head; }

/* ---- introspection ------------------------------------------------------ */

int32_t bmcv_sim_shift_state(const BmcvSim* s) { return s->m.ui_state.shift_state; }

const char* bmcv_sim_mode_name(int32_t shift_state)
{
  if (shift_state < 0 || shift_state >= SHIFT_STATE_COUNT)
    return NULL;
  return ui_mode((uint8_t) shift_state)->name;
}

int32_t bmcv_sim_mode_count(void) { return SHIFT_STATE_COUNT; }

float bmcv_sim_engine_fps(const BmcvSim* s) { return s->m.engine_state.engine_fps; }
float bmcv_sim_dac_fps(const BmcvSim* s) { return s->m.engine_state.dac_fps; }
float bmcv_sim_led_fps(const BmcvSim* s) { return s->m.engine_state.led_fps; }
int32_t bmcv_sim_selected_param(const BmcvSim* s) { return s->m.engine_config.selected_param; }
int32_t bmcv_sim_active_scene(const BmcvSim* s) { return s->m.engine_state.active_scene; }
float bmcv_sim_bpm(const BmcvSim* s) { return s->m.engine_state.clock.bpm; }
float bmcv_sim_active_bpm(const BmcvSim* s) { return s->m.engine_state.clock.beat_freq_smooth * 60.0f; }
int32_t bmcv_sim_have_beat(const BmcvSim* s) { return s->m.engine_state.clock.have_beat ? 1 : 0; }
int32_t bmcv_sim_error_flags(const BmcvSim* s) { return s->m.engine_state.error_flags; }

int32_t bmcv_sim_scene_contribution(const BmcvSim* s, int32_t scene)
{
  if (scene < 0 || scene >= N_SCENES)
    return 0;
  return s->m.engine_state.scenes_contribution[scene];
}

int32_t bmcv_sim_channel_muted(const BmcvSim* s, int32_t channel)
{
  if (channel < 0 || channel >= N_CHANNELS)
    return 0;
  return s->m.ui_state.muted[channel];
}

int32_t bmcv_sim_channel_param(const BmcvSim* s, int32_t channel, int32_t param)
{
  if (channel < 0 || channel >= N_CHANNELS || param < 0 || param >= CH_PARAM_COUNT)
    return 0;
  int8_t scene = s->m.engine_state.active_scene;
  if (scene < 0 || scene >= N_SCENES)
    return 0;
  return s->m.engine_config.channel_state[channel].params[scene][param];
}

// One name per shape mode, read with the same index. Appending a mode without
// naming it fails here rather than showing a blank cell in a host's table.
// Unsized on purpose: with the length written out, a mode added without a name
// here would leave a NULL in the table and the assert below would still pass.
static const char* const shape_mode_names[] = {"LFO", "STEPPED", "PWM"};
_Static_assert(sizeof shape_mode_names / sizeof shape_mode_names[0] == SHAPE_MODE_COUNT, "one name per shape mode");

int32_t bmcv_sim_encoder_pos(const BmcvSim* s, int32_t encoder)
{
  if (!s || encoder < 0 || encoder >= N_ENCODERS)
    return 0;
  return s->m.ux.hw_state->encoder_state[encoder];
}

float bmcv_sim_slider01(const BmcvSim* s)
{
  if (!s)
    return 0.0f;

  const float span = (float) (SLIDER_MAX_VALUE - SLIDER_MIN_VALUE);
  float pos        = ((float) s->m.ux.hw_state->slider_state - (float) SLIDER_MIN_VALUE) / span;

  // The frame is clamped to the raw range before it gets here, so this only
  // ever has to defend against a snapshot from a build whose range differs.
  if (pos < 0.0f)
    pos = 0.0f;
  if (pos > 1.0f)
    pos = 1.0f;
  return pos;
}

int32_t bmcv_sim_channel_shape_mode(const BmcvSim* s, int32_t channel)
{
  if (channel < 0 || channel >= N_CHANNELS)
    return 0;
  return s->m.engine_config.channel_state[channel].shape_mode;
}

const char* bmcv_sim_shape_mode_name(int32_t mode)
{
  if (mode < 0 || mode >= SHAPE_MODE_COUNT)
    return NULL;
  return shape_mode_names[mode];
}

/* ---- midi --------------------------------------------------------------- */

// MidiMsg is exactly the four bytes the flat API promises, so the drain writes
// straight into the caller's buffer rather than transcribing field by field.
_Static_assert(sizeof(MidiMsg) == BMCV_SIM_MIDI_MSG_BYTES, "midi message size");

int32_t bmcv_sim_midi_drain(BmcvSim* s, void* dst, int32_t max_msgs)
{
  if (!s || !dst || max_msgs <= 0)
    return 0;

  // The queue cannot hold more than this, and the count crosses the wasm
  // boundary as an int32 that a frontend computed - the same reason
  // bmcv_sim_run caps its tick count rather than trusting it.
  if (max_msgs > MIDI_OUT_QUEUE_LEN)
    max_msgs = MIDI_OUT_QUEUE_LEN;

  return midi_out_drain(&s->m.midi_out, (MidiMsg*) dst, (uint8_t) max_msgs);
}

/* ---- snapshots ----------------------------------------------------------- */

int32_t bmcv_sim_instance_size(void) { return (int32_t) sizeof(BmcvInstance); }

void bmcv_sim_export(const BmcvSim* s, void* dst)
{
  if (!s || !dst)
    return;
  memcpy(dst, &s->m, sizeof(s->m));
}

int32_t bmcv_sim_import(BmcvSim* s, const void* src, int32_t len)
{
  if (!s || !src || len != (int32_t) sizeof(BmcvInstance))
    return 0;

  memcpy(&s->m, src, sizeof(s->m));

  // The blob's pointers are wherever the module that produced it kept its
  // instance. Every one of them is re-pointed here; nothing else in it moves.
  bmcv_instance_wire(&s->m, &s->io);

  // The input layer stamps each frame with the time the module folded it, so
  // the snapshot carries its own clock and there is no need to guess one. It
  // also means a host that alternates importing and running does not jump the
  // engine's timers backwards.
  s->now_us = s->m.input.curr.time;

  // The sample is this host's, so it survived the copy and now disagrees with
  // the frame that arrived - which the next fold would read as input. See
  // sim_input_adopt(). A pending gate goes with the module that was replaced.
  sim_input_adopt(&s->sample, &s->m.input.curr);
  sim_trig_reset(&s->trig);

  // Straight away rather than on the next run(): the published readings are the
  // whole reason to import, and a snapshot is not something a host may advance
  // a tick to see.
  capture(s);
  return 1;
}

/* ---- persistence -------------------------------------------------------- */

int32_t bmcv_sim_storage_size(void) { return (int32_t) sizeof(SlotStore); }

void bmcv_sim_storage_get(const BmcvSim* s, void* dst) { memcpy(dst, &s->storage, sizeof(SlotStore)); }

int32_t bmcv_sim_storage_set(BmcvSim* s, const void* src, int32_t len)
{
  if (len != (int32_t) sizeof(SlotStore))
    return 0;
  memcpy(&s->storage, src, sizeof(SlotStore));
  return 1;
}
