#ifndef INC_LIB_ASSIGN_H_
#define INC_LIB_ASSIGN_H_

#include "ux_state.h"
#include <stdint.h>

// The mutations behind the selection model. Deliberately plain functions with
// no notion of what is currently selected or how it got picked - ui_select.c
// owns that, and calls these once a source and a destination are both known.
//
//   scene -> scene       duplicate every channel's params
//   channel -> scene     put this channel's params into another scene
//   channel -> channel   within the active scene
//   input -> channel     which input modulates it (MON)
//   trig src -> channel  what samples it (QNT)

void assign_input_to_channel(int8_t i, int8_t c, UxState* state);

void assign_channel_to_channel(int8_t c_src, int8_t c_dst, UxState* state);

void assign_channel_to_scene(int8_t c_src, int8_t s_dst, UxState* state);

void assign_scene_to_scene(int8_t s_src, int8_t s_dst, UxState* state);

void assign_trig_src_use_channel(int8_t c_src, int8_t c_dst, UxState* state);

void assign_trig_src_use_input(int8_t c_src, int8_t i_dst, UxState* state);

void clear_channel(int8_t c, int8_t all_scenes, UxState* state);

void clear_scene(int8_t s, UxState* state);

#endif /* INC_LIB_ASSIGN_H_ */
