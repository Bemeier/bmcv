// Dumps HwSetup and the UxSetup derived from it as JSON, for gen_panel_spec.py
// to merge with the KiCad placement data.
//
// It links the real structs rather than parsing hw_setup.c, so the panel spec
// can never disagree with the firmware about which button carries which role
// or which LED belongs to which control.

#include "hw_setup.h"
#include "ui_mode.h"
#include "ux_setup.h"
#include <stdio.h>

static void emit_i8(const char* name, const int8_t* v, int n, const char* tail)
{
  printf("    \"%s\": [", name);
  for (int i = 0; i < n; i++)
  {
    printf("%s%d", i ? ", " : "", v[i]);
  }
  printf("]%s\n", tail);
}

static void emit_u8(const char* name, const uint8_t* v, int n, const char* tail)
{
  printf("    \"%s\": [", name);
  for (int i = 0; i < n; i++)
  {
    printf("%s%u", i ? ", " : "", v[i]);
  }
  printf("]%s\n", tail);
}

int main(void)
{
  const HwSetup* hw = HwSetup_Get();
  const UxSetup* ux = UxSetup_InitFromHw(hw);

  printf("{\n");

  printf("  \"counts\": {\n");
  printf("    \"inputs\": %d,\n", N_INPUTS);
  printf("    \"encoders\": %d,\n", N_ENCODERS);
  printf("    \"channels\": %d,\n", N_CHANNELS);
  printf("    \"buttons\": %d,\n", N_BUTTONS);
  printf("    \"scenes\": %d,\n", N_SCENES);
  printf("    \"ctrl_buttons\": %d,\n", N_CTRL_BUTTONS);
  printf("    \"semitones\": %d,\n", N_SEMITONES);
  printf("    \"leds\": %d\n", LED_COUNT);
  printf("  },\n");

  // A ctrl button's id is a ShiftStates, so its name is the mode table's own -
  // emitted here rather than retyped in gen_panel_spec.py.
  printf("  \"ctrl_names\": [");
  for (int i = 0; i < N_CTRL_BUTTONS; i++)
  {
    printf("%s\"%s\"", i ? ", " : "", ui_mode((uint8_t) i)->name);
  }
  printf("],\n");

  printf("  \"ranges\": {\n");
  printf("    \"adc_10v\": %d,\n", ADC_10V);
  printf("    \"dac_10v\": %d\n", DAC_10V);
  printf("  },\n");

  printf("  \"hw_setup\": {\n");
  emit_u8("input_adc_idx", hw->input_adc_idx, N_INPUTS, ",");
  emit_i8("channel_dac_idx", hw->channel_dac_idx, N_ENCODERS, ",");
  emit_i8("channel_encoder_idx", hw->channel_encoder_idx, N_ENCODERS, ",");
  emit_i8("channel_button_idx", hw->channel_button_idx, N_ENCODERS, ",");
  emit_u8("quantizer_button_idx", hw->quantizer_button_idx, N_SEMITONES, ",");
  emit_i8("ctrl_button_idx", hw->ctrl_button_idx, N_CTRL_BUTTONS, ",");
  emit_i8("scene_button_idx", hw->scene_button_idx, N_SCENES, ",");
  emit_i8("channel_led_idx", hw->channel_led_idx, N_ENCODERS, ",");
  emit_i8("scene_button_led_idx", hw->scene_button_led_idx, N_SCENES, ",");
  emit_u8("quantizer_button_led_idx", hw->quantizer_button_led_idx, N_SEMITONES, ",");
  emit_i8("ctrl_button_led_idx", hw->ctrl_button_led_idx, N_CTRL_BUTTONS, ",");
  emit_u8("ctrl_button_color", hw->ctrl_button_color, N_CTRL_BUTTONS, "");
  printf("  },\n");

  // The transposed, role-oriented view. Redundant with hw_setup above, but it
  // is what the generator actually consumes, and dumping it proves the
  // transpose in ux_setup.c agrees with the tables it came from.
  printf("  \"ux_setup\": {\n");

  printf("    \"channels\": [\n");
  for (int i = 0; i < N_ENCODERS; i++)
  {
    const ChannelSetup* c = &ux->channels[i];
    printf("      {\"id\": %u, \"button\": %d, \"led\": %d, \"encoder\": %d, \"dac_channel\": %d}%s\n", c->id, c->button, c->led,
           c->encoder, c->dac_channel, i + 1 < N_ENCODERS ? "," : "");
  }
  printf("    ],\n");

  printf("    \"scenes\": [\n");
  for (int i = 0; i < N_SCENES; i++)
  {
    const SceneSetup* s = &ux->scenes[i];
    printf("      {\"id\": %u, \"button\": %d, \"led\": %d}%s\n", s->id, s->button, s->led, i + 1 < N_SCENES ? "," : "");
  }
  printf("    ],\n");

  printf("    \"ctrl_buttons\": [\n");
  for (int i = 0; i < N_CTRL_BUTTONS; i++)
  {
    const CtrlButtonSetup* c = &ux->ctrl_buttons[i];
    printf("      {\"id\": %u, \"button\": %d, \"led\": %d, \"color\": %u}%s\n", c->id, c->button, c->led, c->color,
           i + 1 < N_CTRL_BUTTONS ? "," : "");
  }
  printf("    ],\n");

  printf("    \"quantizer_semitones\": [\n");
  for (int i = 0; i < N_SEMITONES; i++)
  {
    const QuantizerSetup* q = &ux->quantizer_semitones[i];
    printf("      {\"id\": %u, \"button\": %d, \"led\": %d}%s\n", q->id, q->button, q->led, i + 1 < N_SEMITONES ? "," : "");
  }
  printf("    ]\n");

  printf("  }\n");
  printf("}\n");

  return 0;
}
