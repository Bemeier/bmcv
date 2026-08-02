#include "bmcv_sim.h"
#include "channel.h"
#include "clock_sync.h"
#include "config.h"
#include "engine_state.h"
#include "error.h"
#include "hw_setup.h"
#include "input_fold.h"
#include "instance.h"
#include "presets.h"
#include "sim_rt.h"
#include "ui_mode.h"
#include "ui_state.h"
#include "ux_setup.h"
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

// Preset storage. On the module this is FRAM; here it is just memory the host
// can serialise. Records keep the same layout, so a blob is interchangeable.
typedef struct
{
  EngineConfig slots[FRAM_CONFIG_SLOTS];
  uint8_t occupied[FRAM_CONFIG_SLOTS];
} SimStorage;

struct BmcvSim
{
  BmcvInstance m;

  InputSample sample;
  SimTrigLatch trig;

  SimStorage storage;
  PresetIo io;

  uint32_t now_us;

  float outputs_v[N_CHANNELS];
  uint8_t leds_rgb[LED_COUNT * 3];

  float effective[N_CHANNELS * BMCV_EFF_COUNT];
  float scope[N_CHANNELS * BMCV_SIM_SCOPE_LEN];
  float input_scope[N_INPUTS * BMCV_SIM_SCOPE_LEN];
  uint32_t scope_head;
};

static int8_t sim_store(void* user, const EngineConfig* cfg, int8_t slot)
{
  SimStorage* st = (SimStorage*) user;
  if (slot < 0 || slot >= FRAM_CONFIG_SLOTS)
    return 0;
  st->slots[slot]    = *cfg;
  st->occupied[slot] = 1;
  return 1;
}

static int8_t sim_load(void* user, EngineConfig* cfg, int8_t slot)
{
  SimStorage* st = (SimStorage*) user;
  if (slot < 0 || slot >= FRAM_CONFIG_SLOTS || !st->occupied[slot])
    return 0;
  *cfg = st->slots[slot];
  return 1;
}

static void sim_boot(BmcvSim* s)
{
  s->io.store = sim_store;
  s->io.load  = sim_load;
  s->io.user  = &s->storage;

  s->now_us = 0;
  memset(&s->sample, 0, sizeof(s->sample));
  sim_trig_reset(&s->trig);

  bmcv_instance_init(&s->m, &s->io, 0);

  // Start at the bottom of slider travel rather than 0, which is below
  // SLIDER_MIN_VALUE and would clamp anyway.
  s->sample.slider_raw = SLIDER_MIN_VALUE;
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
    memset(&s->storage, 0, sizeof(s->storage));

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
  if (pos01 < 0.0f)
    pos01 = 0.0f;
  if (pos01 > 1.0f)
    pos01 = 1.0f;
  s->sample.slider_raw = (uint16_t) (SLIDER_MIN_VALUE + pos01 * (float) (SLIDER_MAX_VALUE - SLIDER_MIN_VALUE) + 0.5f);
}

void bmcv_sim_set_cv(BmcvSim* s, int32_t input, float volts)
{
  if (!s || input < 0 || input >= N_INPUTS)
    return;

  // The API speaks in jacks; InputSample speaks in converter channels.
  uint8_t ch  = s->m.hw_setup->input_adc_idx[input];
  int16_t raw = sim_volts_to_adc(volts);

  s->sample.cv_raw[ch] = raw;
  sim_trig_sample(&s->trig, ch, raw);
}

void bmcv_sim_fire_gate(BmcvSim* s, int32_t input)
{
  if (!s || input < 0 || input >= N_INPUTS)
    return;
  sim_trig_fire(&s->trig, s->m.hw_setup->input_adc_idx[input]);
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
    const LedRgb* led      = &s->m.engine_state.leds[i];
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
    for (uint8_t ch = 0; ch < N_INPUTS; ch++)
    {
      s->sample.cv_trig[ch] = sim_trig_take(&s->trig, ch);
    }

    s->now_us += dt_us;
    bmcv_instance_tick(&s->m, &s->sample, s->now_us);
    capture(s);
  }
}

uint32_t bmcv_sim_now_us(const BmcvSim* s) { return s ? s->now_us : 0; }

/* ---- output ------------------------------------------------------------- */

const float* bmcv_sim_outputs_v(const BmcvSim* s) { return s->outputs_v; }
const uint8_t* bmcv_sim_leds_rgb(const BmcvSim* s) { return s->leds_rgb; }
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
int32_t bmcv_sim_selected_param(const BmcvSim* s) { return s->m.ui_state.selected_param; }
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
static const char* const shape_mode_names[] = {"LFO", "SMOOTH", "SEMI", "HARD", "PWM"};
_Static_assert(sizeof shape_mode_names / sizeof shape_mode_names[0] == SHAPE_MODE_COUNT, "one name per shape mode");

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

/* ---- persistence -------------------------------------------------------- */

int32_t bmcv_sim_storage_size(void) { return (int32_t) sizeof(SimStorage); }

void bmcv_sim_storage_get(const BmcvSim* s, void* dst) { memcpy(dst, &s->storage, sizeof(SimStorage)); }

int32_t bmcv_sim_storage_set(BmcvSim* s, const void* src, int32_t len)
{
  if (len != (int32_t) sizeof(SimStorage))
    return 0;
  memcpy(&s->storage, src, sizeof(SimStorage));
  return 1;
}
