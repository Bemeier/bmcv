#ifndef INC_LIB_ENGINE_H_
#define INC_LIB_ENGINE_H_

#include "ux_state.h"
#include <stdint.h>

// One tick of the module, with no hardware involved: scene blending, per
// channel signal generation, output trigger detection, and (rate-limited) the
// UX/LED update.
//
// Everything it reads comes from UxState (hw_state is filled in by the caller
// from real hardware, or by a test/tool from a scripted timeline) and
// everything it produces lands back in EngineState - channel levels in
// channels_output_level[] and LED colours in leds[]. Pushing those to the DAC
// and LED driver is the caller's job.
//
// input_dirty: non-zero when a button/encoder changed this tick, which forces
// the UX update instead of waiting for the periodic interval.
void engine_tick(UxState* state, uint32_t now_us, uint8_t input_dirty);

#endif /* INC_LIB_ENGINE_H_ */
