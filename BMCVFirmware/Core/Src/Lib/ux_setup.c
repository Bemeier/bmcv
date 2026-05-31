#include "ux_setup.h"
#include "hw_setup.h"

static UxSetup ux_setup;

const UxSetup* UxSetup_InitFromHw(const HwSetup* hw)
{
  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    ux_setup.scenes[s] = (SceneSetup){
        .id     = s,
        .led    = hw->scene_button_led_idx[s],
        .button = hw->scene_button_idx[s],
    };
  }

  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    ux_setup.channels[c] = (ChannelSetup){
        .id          = c,
        .button      = hw->channel_button_idx[c],
        .led         = hw->channel_led_idx[c],
        .encoder     = hw->channel_encoder_idx[c],
        .dac_channel = hw->channel_dac_idx[c],
    };
  }

  for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
  {
    ux_setup.ctrl_buttons[b] = (CtrlButtonSetup){
        .id     = b,
        .button = hw->ctrl_button_idx[b],
        .led    = hw->ctrl_button_led_idx[b],
        .color  = hw->ctrl_button_color[b],
    };
  }

  for (uint8_t st = 0; st < N_SEMITONES; st++)
  {
    ux_setup.quantizer_semitones[st] = (QuantizerSetup){
        .id     = st,
        .button = hw->quantizer_button_idx[st],
        .led    = hw->quantizer_button_led_idx[st],
    };
  }

  return &ux_setup;
}
