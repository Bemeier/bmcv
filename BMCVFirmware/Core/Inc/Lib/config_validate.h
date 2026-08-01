#ifndef INC_LIB_CONFIG_VALIDATE_H_
#define INC_LIB_CONFIG_VALIDATE_H_

#include "state.h"

// Clamp every persisted field that is later used as an array index or enum
// selector into a range the rest of the firmware can safely handle.
//
// preset_load() checks the record header - magic, version, length, CRC - but
// a valid header says nothing about whether the *values* inside make sense to
// this build. The case that matters in practice is a firmware downgrade: a
// preset saved by a newer build carries a shape_mode this build has never
// heard of, yet still passes every header check. Without this it would index
// shape_mode_color[] past its end.
//
// A no-op for any config this build would itself produce, so running it over a
// healthy preset changes nothing.
void config_validate(EngineConfig* cfg);

#endif /* INC_LIB_CONFIG_VALIDATE_H_ */
