#ifndef INC_LIB_UXSTATE_H_
#define INC_LIB_UXSTATE_H_

#include "state.h"
#include "ux_setup.h"
#include <stdint.h>

typedef struct
{
  const UxSetup* ux_setup;
  const HwSetup* hw_setup;
  uint32_t last_ux_update;
  uint32_t dt;
  HwState* hw_state;
  EngineConfig* engine_config;
  EngineState* engine_state;
} UxState;

void update_ux_state(UxState* state);

#endif /* INC_LIB_UXSTATE_H_ */
