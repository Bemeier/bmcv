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
  ChannelConfig channel_state[N_CHANNELS];
} EngineConfig;

// TODO: Configure quantization pre/post LFO?
//   - When we add offset from cv, pre LFO could be nice for vibrato
//   - Otherwise, post LFO makes more sense, as we can use it for sequences
// ...

#endif /* INC_LIB_CONFIG_H_ */
