#include "ui_input.h"
#include "hw_setup.h"
#include "hw_state.h"
#include <stdint.h>

void ui_input_update(UiInput* in, const HwState* curr)
{
  in->dt += curr->dt;

  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    uint8_t now_down = curr->button_state[b] != 0;
    uint8_t was_down = in->down[b];

    if (now_down && !was_down)
    {
      in->ev[b] |= BTN_EV_DOWN;
      in->held_us[b] = 0;
      in->crossed[b] = 0;
    }
    else if (now_down)
    {
      in->held_us[b] += curr->dt;

      // Threshold crossings are not level changes, so they have to be caught
      // here - the UX layer may not run on the tick they happen.
      if (!(in->crossed[b] & BTN_EV_HOLD) && in->held_us[b] >= UI_T_HOLD)
      {
        in->ev[b] |= BTN_EV_HOLD;
        in->crossed[b] |= BTN_EV_HOLD;
      }
      if (!(in->crossed[b] & BTN_EV_LONG) && in->held_us[b] >= UI_T_LONG)
      {
        in->ev[b] |= BTN_EV_LONG;
        in->crossed[b] |= BTN_EV_LONG;
      }
      if (!(in->crossed[b] & BTN_EV_VLONG) && in->held_us[b] >= UI_T_VLONG)
      {
        in->ev[b] |= BTN_EV_VLONG;
        in->crossed[b] |= BTN_EV_VLONG;
      }
    }
    else if (was_down)
    {
      // The release tick counts towards the press: the button was down when
      // the previous tick sampled it and up when this one did, so the whole
      // interval belongs to the press the same way it does for hw_state.time.
      //
      // Not cosmetic. It is what makes `time - held_us` the instant of the
      // press on every tick including this one, which is the comparison the
      // reset guard in ui_channel.c makes against the last edit. Stopping one
      // tick short made an edit on the press tick look older than the press
      // containing it, and the release wiped it.
      in->held_us[b] += curr->dt;

      if (in->held_us[b] >= UI_T_DEBOUNCE)
      {
        in->ev[b] |= BTN_EV_UP;
        if (in->held_us[b] < UI_T_HOLD)
        {
          in->ev[b] |= BTN_EV_TAP;
        }
      }
    }

    in->down[b] = now_down;
  }

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    in->enc_delta[e] += curr->encoder_delta[e];
  }
}

void ui_input_drain(UiInput* in)
{
  in->dt = 0;

  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    in->ev[b] = BTN_EV_NONE;
  }

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    in->enc_delta[e] = 0;
  }
}
