#ifndef INC_LIB_STATE_H_
#define INC_LIB_STATE_H_

#include "hw_setup.h"
#include <stdint.h>

#define STATE_RINGBUF_SIZE 2

#define CTRL_DEFAULT 0

#define SLIDER_MIN_VALUE 400
#define SLIDER_MAX_VALUE 7661

#define FAST_BLINK_PERIOD 300000
#define SLOW_BLINK_PERIOD 800000

typedef enum
{
  INPUT_DEFAULT, // Available for quantizing/adding
  INPUT_CLOCK,
  INPUT_RESET,
  INPUT_SLIDER,
  INPUT_MODE_COUNT
} InputMode;

// Persisted as a plain int8_t in ChannelConfig, so appending modes here does
// NOT change the FRAM layout and existing presets keep loading. Only ever
// append - inserting or reordering would silently remap saved channels.
typedef enum
{
  SHAPE_LFO,            // wavetable
  SHAPE_STEPPED_SMOOTH, // random steps, fully eased between values
  SHAPE_STEPPED_SEMI,   // random steps, half held then eased
  SHAPE_STEPPED_HARD,   // random steps, mostly held with a quick eased edge
  SHAPE_MODE_COUNT,
} ChannelShapeMode;

// In the stepped modes CH_PARAM_MOD selects a pattern length from a discrete
// set; in SHAPE_LFO it is a continuous phase warp. The edit behaviour and the
// latching below both differ accordingly.
static inline int shape_mode_is_stepped(int8_t mode) { return mode >= SHAPE_STEPPED_SMOOTH && mode <= SHAPE_STEPPED_HARD; }

typedef enum
{
  QUANTIZE_DISABLED,
  QUANTIZE_CONTINUOUS,
  QUANTIZE_TRIG_SRC,
  QUANTIZE_MODE_COUNT,
} ChannelQuantizeMode;

typedef enum
{
  INPUT_AMP_DISABLED,
  INPUT_AMP_ADD,
  INPUT_AMP_MULT,
  INPUT_AMP_MODE_COUNT,
} ChannelInputAmpMode;

typedef enum
{
  CH_PARAM_FRQ,
  CH_PARAM_SHP,
  CH_PARAM_MOD,
  CH_PARAM_PHS,
  CH_PARAM_AMP,
  CH_PARAM_OFS,
  CH_PARAM_COUNT
} ChannelParameters;

typedef enum
{
  SHIFT_STATE_STA,
  SHIFT_STATE_SYS,
  SHIFT_STATE_QNT,
  SHIFT_STATE_MON,
  SHIFT_STATE_SAV,
  SHIFT_STATE_STB,
  SHIFT_STATE_MUT,
  SHIFT_STATE_CPY,
  SHIFT_STATE_CLR,
  SHIFT_STATE_NONE,
  SHIFT_STATE_COUNT
} ShiftStates;

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint16_t version;
  uint16_t length;
  uint32_t crc;
} FramRecordHeader;

typedef struct __attribute__((packed))
{
  int8_t src_input;
  int8_t src_trig;
  int8_t shape_mode;
  ChannelInputAmpMode input_amp_mode;
  ChannelQuantizeMode quantize_mode;
  int16_t params[N_SCENES][CH_PARAM_COUNT];
} ChannelConfig;

typedef struct __attribute__((packed))
{
  uint8_t clock_div;
  uint8_t scene_a;
  uint8_t scene_b;
  uint8_t current_preset;
  uint16_t quantize_mask;
  InputMode input_mode[N_INPUTS];
  ChannelConfig channel_state[N_ENCODERS];
} EngineConfig;

// TODO: Configure quantization pre/post LFO?
//   - When we add offset from cv, pre LFO could be nice for vibrato
//   - Otherwise, post LFO makes more sense, as we can use it for sequences
// ...

typedef struct
{
  uint32_t time; // timestamp of state
  uint32_t dt;   // time since last state

  uint16_t slider_state;

  uint8_t trigger_src[N_INPUTS + N_CHANNELS];

  // Raw level only. Press durations and gestures are derived once, in
  // ui_input.c - HwState deliberately no longer carries them, so there is no
  // second source of truth for "how long has this been held".
  uint8_t button_state[N_BUTTONS];

  int16_t encoder_state[N_ENCODERS];
  int16_t encoder_delta[N_ENCODERS]; // change of encoder since last state

  int16_t input_state[N_INPUTS];
} HwState;

typedef struct
{
  uint8_t r, g, b;
} LedRgb;

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
  // ui_channel_note_edit() - compute_channel reads it to decide whether a
  // stepped-pattern length change applies immediately or waits for the cycle
  // wrap, which is the one place the DSP needs to know the user is fiddling.
  uint32_t channels_last_delta[N_CHANNELS];

  // Ramped output gate, 0..1, slewed toward UiState.muted[] at DAC rate so
  // muting does not click.
  float channels_mute_gain[N_CHANNELS];

  int16_t cgcd[N_CHANNELS];
  float cphsc[N_CHANNELS];
  float csphs[N_CHANNELS];
  float cfrm[N_CHANNELS];
  float cshp[N_CHANNELS];
  float cmod[N_CHANNELS];

  // Which scene currently dominates the crossfade. Derived from the slider
  // (and the momentary scene the UI passes in), so it is an engine output the
  // UI reads, not UI state.
  int8_t active_scene;

  // LED framebuffer. The UX layer renders into this; a flush step pushes it
  // to the driver. Keeps LED behaviour assertable without any hardware.
  LedRgb leds[LED_COUNT];

  float engine_fps;
  float dac_fps;

} EngineState;

#endif /* INC_LIB_STATE_H_ */
