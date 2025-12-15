#include "assign.h"
#include "bmcv.h"
#include <stdint.h>

static AssignType _assign_state = ASSIGN_NONE;
static int8_t _source_id;

void assign_reset()
{
    _assign_state = ASSIGN_NONE;
    _source_id    = -1;
}

int8_t assign_src() { return _source_id; }
AssignType assign_state() { return _assign_state; }

void assign_event(AssignType targetType, int8_t targetId)
{
    if (_assign_state == ASSIGN_NONE)
    {
        _assign_state = targetType;
        _source_id    = targetId;
    }
    else
    {
        if (_assign_state == ASSIGN_INPUT && targetType == ASSIGN_CHANNEL)
        {
            bmcv_assign_input_to_channel(_source_id, targetId);
        }
    }
}
