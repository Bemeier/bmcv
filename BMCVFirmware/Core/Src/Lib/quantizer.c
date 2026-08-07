#include "quantizer.h"
#include "color_presets.h"
#include "led_fb.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"

void ui_quantizer_update(UxState* state)
{
  // Which modes overlay a keyboard is the mode table's business, not this
  // file's - it used to name SHIFT_STATE_QNT itself, as did the renderer.
  if (!ui_mode(state->ui->shift_state)->keyboard_overlay)
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
