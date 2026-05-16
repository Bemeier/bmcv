#include "ux_state.h"
#include "assign.h"
#include "channel.h"
#include "ctrl_button.h"
#include "hw_setup.h"
#include "quantizer.h"
#include "scene.h"
#include "state.h"
#include <stdint.h>

void update_ux_state(UxState* state)
{
    //state->engine_state->shift_state      = SHIFT_STATE_NONE;

    for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
    {
        // Check if any shift mode is active
        update_shift_mode(&state->ux_setup->ctrl_buttons[b], state);
    }

    // could be more specific, assign only happens in MON CPY or CLR
    if (state->engine_state->shift_state == SHIFT_STATE_NONE)
    {
        assign_reset();
    }

    for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
    {
        // Check for config param switch
        update_selected_param(&state->ux_setup->ctrl_buttons[b], state);
    }

    update_quantizer_buttons(state);

    state->engine_state->momentary_active_for = 0;
    state->engine_state->momentary_scene      = -1;

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        update_scene_button(&state->ux_setup->scenes[s], state);
    }

    compute_scenes_contribution(state);

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        update_channel(&state->ux_setup->channels[c], state);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        compute_channel(&state->ux_setup->channels[c], state);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        write_channel_dac(&state->ux_setup->channels[c], state);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        write_channel_led(&state->ux_setup->channels[c], state);
    }

    for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
    {
        write_ctrl_button_led(&state->ux_setup->ctrl_buttons[b], state);
    }

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        write_scene_button_led(&state->ux_setup->scenes[s], state);
    }

    write_quantizer_button_leds(state);
}
