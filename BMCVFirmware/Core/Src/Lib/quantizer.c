#include "quantizer.h"
#include "assign.h"
#include "color_presets.h"
#include "led_fb.h"
#include "state.h"
#include "ui_input.h"

void update_quantizer_buttons(UxState* state)
{
  if (state->ui->shift_state != SHIFT_STATE_QNT)
    return;
  // TODO: The one non-semitone button?
  for (uint16_t st = 0; st < N_SEMITONES; st++)
  {
    if (btn_ev(&state->ui->in, state->ux_setup->quantizer_semitones[st].button, BTN_EV_UP))
    {
      state->engine_config->quantize_mask ^= (1u << st);
    }
  }
}

void write_quantizer_button_leds(UxState* state)
{
  if (state->ui->shift_state != SHIFT_STATE_QNT)
    return;
  if (assign_state(state) == ASSIGN_TRIG_SRC)
    return;
  // TODO: The one non-semitone button?
  for (uint16_t st = 0; st < N_SEMITONES; st++)
  {
    uint8_t sat = SAT_OFF;
    uint8_t val = (state->engine_config->quantize_mask & (1u << st)) ? VAL_LOW : VAL_OFF;
    led_set_hsv(state, state->ux_setup->quantizer_semitones[st].led, 0, sat, val);
  }
}
