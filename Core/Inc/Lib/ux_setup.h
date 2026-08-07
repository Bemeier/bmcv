#ifndef INC_LIB_UX_SETUP_H_
#define INC_LIB_UX_SETUP_H_

#include "hw_setup.h"
#include <stdint.h>

typedef struct
{
  // Config
  uint8_t id;
  int8_t button;
  int8_t led;
} SceneSetup;

typedef struct
{
  // Config
  uint8_t id;
  int8_t button;
  int8_t led;
  uint8_t color;
} CtrlButtonSetup;

typedef struct
{
  uint8_t id;
  int8_t button;
  int8_t led;
  int8_t encoder;
  int8_t dac_channel;
} ChannelSetup;

typedef struct
{
  uint8_t id; // semitone index
  int8_t button;
  int8_t led;
} QuantizerSetup;

typedef struct
{
  SceneSetup scenes[N_SCENES];
  ChannelSetup channels[N_ENCODERS];
  CtrlButtonSetup ctrl_buttons[N_CTRL_BUTTONS];
  QuantizerSetup quantizer_semitones[N_SEMITONES];
} UxSetup;

const UxSetup* UxSetup_InitFromHw(const HwSetup* hw);

#endif /* INC_LIB_UX_SETUP_H_ */
