#ifndef INC_LIB_CONFIG_MIGRATE_H_
#define INC_LIB_CONFIG_MIGRATE_H_

#include "config.h"
#include <stdint.h>

// Reading a stored config written by an older build.
//
// The record format has changed three times in as many months, and every time
// it did, `version != CONFIG_STATE_VERSION` threw away all eight slots. That is
// the right default while the only module in existence is on the bench and the
// person flashing it is the person who changed the format. It stops being the
// right default the moment somebody else has seven scenes dialled in.
//
// So: rejecting is still what happens to a version this build has never heard
// of, but a version it does know is converted. Each conversion is a function of
// the old layout and the old *meaning* - two fields appended is not the same
// problem as a value that used to mean half of what it now means, and both have
// happened here.
//
// The old layouts live in config_migrate.c, spelled out with the types their
// headers used. A stored record is always read back by the target that wrote
// it, so what an old struct has to match is the old header on that target, not
// the current struct on any other.

// The oldest record this build can still read. Anything below it is rejected,
// which is the first-boot path: config_defaults() and an empty module.
#define CONFIG_STATE_VERSION_MIN 2

// Convert `data` - `length` bytes of payload as written by `version` - into a
// current EngineConfig. Returns 1 if the version is one this build knows.
//
// The caller has already checked the magic and the CRC; this is only about
// layout and meaning. config_validate() runs on the result, so a record that
// converts is always safe to index with, whatever it held.
int8_t config_migrate(uint16_t version, uint16_t length, const void* data, EngineConfig* out);

#endif /* INC_LIB_CONFIG_MIGRATE_H_ */
