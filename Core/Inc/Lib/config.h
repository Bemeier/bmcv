#ifndef INC_LIB_CONFIG_H_
#define INC_LIB_CONFIG_H_

#include "hw_setup.h"
#include <stdint.h>

// The persisted state of a module: everything that survives a power cycle, and
// the enums whose numeric values are therefore part of the FRAM record format.
//
// Separated from the engine's running state (engine_state.h) and from the
// hardware frame (hw_state.h) because the rules are different here: these
// values are written to a chip and read back by a possibly different build, so
// they are validated on load (config_validate.h) and may only ever be
// appended to.

// How many stored configs there are, and which slot the periodic autosave
// writes to. Here rather than in presets.h because the core needs the count
// and presets.c is a driver source - a core file must not depend on a driver
// header for it.
#define FRAM_CONFIG_SLOTS 8
#define CONFIG_AUTOSAVE_SLOT (FRAM_CONFIG_SLOTS - 1)

// What this build writes. Bumping it is how a record whose layout or meaning
// has changed stops being read as if it had not - and config_migrate.c is
// where you then say how to read the old one. Here rather than in presets.h
// for the same reason as the slot count: the migration is core code and must
// not include a driver header to learn the version it is migrating to.
#define CONFIG_STATE_VERSION 5

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
  SHAPE_LFO,     // wavetable
  SHAPE_STEPPED, // random values, eased between; MOD is the density
  SHAPE_PWM,     // square; SHP is the pulse width, MOD the envelope
  SHAPE_MODE_COUNT,
} ChannelShapeMode;

// The stepped mode reads its pattern length from ChannelConfig.sr_length_idx
// rather than from a scene parameter, so this is what says whether that setting
// means anything for a given channel.
static inline int shape_mode_is_stepped(int8_t mode) { return mode == SHAPE_STEPPED; }

// What the output stage clamps a channel to. A range, not a scaling: the
// parameters still mean what they say and the ends of the swing are cut off.
//
// Bipolar first, and the widest first, so a zeroed config is the full range -
// which is what every channel did before this existed.
typedef enum
{
  CLAMP_BI_10,  // +/-10V, the converter's whole range
  CLAMP_BI_5,   // +/-5V
  CLAMP_UNI_10, // 0..10V
  CLAMP_UNI_5,  // 0..5V
  CLAMP_MODE_COUNT,
} ChannelClampMode;

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

  // Per channel rather than per scene, both of them deliberately.
  //
  // Pattern length is an index into a curated set of whole steps per cycle -
  // there is nothing between 8 steps and 12, so a value that gets crossfaded
  // between scenes is meaningless. It was CH_PARAM_MOD, which cost that
  // parameter its continuity in three of the five shape modes.
  //
  // The clamp is what the module is patched into, not part of the patch: an
  // output feeding a 0..5V input should stay in that range as the scenes move.
  int8_t sr_length_idx;
  int8_t clamp_mode; // ChannelClampMode
} ChannelConfig;

typedef struct __attribute__((packed))
{
  uint8_t clock_div;
  uint8_t scene_a;
  uint8_t scene_b;
  uint8_t current_preset;
  uint16_t quantize_mask;
  InputMode input_mode[N_INPUTS];
  ChannelConfig channel_state[N_CHANNELS];

  // Which parameter the encoders edit when no shift mode is running, and the
  // one field here that is about the panel rather than about the sound. It
  // lives with the patch rather than in UiState because UiState is the half
  // that is deliberately not saved - a module coming back on the page it was
  // left on is the whole point, and scene_a/scene_b/current_preset are already
  // stored the same way for the same reason.
  //
  // Appended rather than filed with the scalars at the top, so a v4 record is
  // exactly this struct without its last byte and the migration is a copy
  // rather than a field-by-field transcription.
  uint8_t selected_param; // ChannelParameters
} EngineConfig;

// TODO: Configure quantization pre/post LFO?
//   - When we add offset from cv, pre LFO could be nice for vibrato
//   - Otherwise, post LFO makes more sense, as we can use it for sequences
// ...

#endif /* INC_LIB_CONFIG_H_ */
