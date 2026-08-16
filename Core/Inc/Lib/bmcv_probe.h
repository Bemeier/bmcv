#ifndef INC_LIB_BMCV_PROBE_H_
#define INC_LIB_BMCV_PROBE_H_

#include <stdint.h>

// Where a debug probe should look, published so that it does not need the ELF.
//
// The whole module is one struct at one address - `bmcv`, see bmcv.h - so a
// host holding an SWD probe can read the entire running state out of RAM in a
// single transfer and hand it to bmcv_sim_import(), which decodes it with the
// firmware's own code. That is how the web frontend shows a physical module;
// docs/live-module.md is the long version.
//
// The problem this solves is finding it. `bmcv` sits wherever the linker put
// it, and it moves whenever anything before it in .bss changes size, so its
// address cannot be written down anywhere. A browser holding a WebUSB handle to
// an ST-Link has no ELF and no nm to ask. So the firmware says where it is, at
// an address that does not move.
//
// This is a const in flash and nothing else - no code, on no path, costing 28
// bytes. It is the same bargain bmcv.h already makes when it keeps the instance
// external "purely so a live debugger has something to attach to".

// Immediately past the vector table, which ends at 0x080001D8, and rounded up
// to somewhere memorable. The linker script places the section here and asserts
// that it did, so this constant and the image cannot disagree.
#define BMCV_PROBE_INFO_ADDR 0x08000200u

// "BMCV" as it reads in a little-endian memory dump.
#define BMCV_PROBE_MAGIC 0x56434D42u

// Bumped when a field below changes meaning or the struct grows. A reader that
// does not know a version must refuse rather than guess: everything after this
// field is what it describes.
#define BMCV_PROBE_INFO_VERSION 1

typedef struct
{
  uint32_t magic;         // BMCV_PROBE_MAGIC
  uint16_t info_version;  // BMCV_PROBE_INFO_VERSION
  uint16_t instance_size; // BMCV_SNAPSHOT_BYTES - what to read, and the first
                          // check that the reader agrees about the layout
  uint32_t instance_addr; // &bmcv - what to read it from
  char version[16];       // the firmware version, "0.10.0", NUL-terminated
} BmcvProbeInfo;

// A reader indexes these bytes by hand, so the shape is fixed here rather than
// left to a compiler. Every field is naturally aligned and there is no padding.
_Static_assert(sizeof(BmcvProbeInfo) == 28, "probe info layout is part of the ABI");

#endif /* INC_LIB_BMCV_PROBE_H_ */
