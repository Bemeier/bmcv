#ifndef INC_LIB_UXSTATE_H_
#define INC_LIB_UXSTATE_H_

typedef enum
{
    CH_PARAM_FRQ,
    CH_PARAM_SHP,
    CH_PARAM_PHS,
    CH_PARAM_INP,
    CH_PARAM_AMP,
    CH_PARAM_OFS,
    CH_PARAM_COUNT
} ChannelParameters;

typedef enum
{
    SHIFT_STATE_STL,
    SHIFT_STATE_SAV,
    SHIFT_STATE_SYS,
    SHIFT_STATE_MON,
    SHIFT_STATE_SEQ,
    SHIFT_STATE_STR,
    SHIFT_STATE_QNT,
    SHIFT_STATE_CPY,
    SHIFT_STATE_CLR,
    SHIFT_STATE_NONE,
    SHIFT_STATE_COUNT
} ShiftStates;

// SYSTEM STATE = ChannelParameter + ShfitState

// ASSIGN STATE = ASSIGN_TYPE (src) + srcIdx
//   => param

//

#endif /* INC_LIB_UXSTATE_H_ */
