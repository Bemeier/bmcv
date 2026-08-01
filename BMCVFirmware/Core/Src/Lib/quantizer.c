#include "quantizer.h"
#include "color_presets.h"
#include "led_fb.h"
#include "state.h"
#include "ui_input.h"
#include "ui_select.h"

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
