#ifndef INC_LIB_UXSTATE_H_
#define INC_LIB_UXSTATE_H_

#include "state.h"
#include "ux_setup.h"

typedef struct
{
    const UxSetup* ux_setup;
    const HwSetup* hw_setup;
    HwState* hw_state;
    EngineConfig* engine_config;
    EngineState* engine_state;
} UxState;

void update_ux_state(UxState* state);

#endif /* INC_LIB_UXSTATE_H_ */
