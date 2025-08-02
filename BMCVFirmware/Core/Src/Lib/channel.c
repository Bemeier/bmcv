#include "channel.h"
#include "dac_adc.h"
#include "ws2811.h"

void init_channel(Channel* ch)
{
    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        ch->level[s] = 0;
        //-8000 + (16000/N_SCENES)*s;
    }
}

void update_channel(Channel* ch, State* state)
{
    int16_t delta = state->encoder_delta[ch->encoder];
    // CLAMP
    ch->level[state->active_scene_id] += (delta * 50);

    // TODO: allow blinky if change
    if (state->button_released_t[ch->button] > 1000)
    {
        init_channel(ch);
    }
    else if (state->button_released_t[ch->button] > 50)
    {
        ch->level[state->active_scene_id] = 0;
    }

    ws2811_setled_dac(ch->led, ch->level[state->active_scene_id]);
    dacadc_write(ch->dac_channel, ch->level[state->active_scene_id]);
}
