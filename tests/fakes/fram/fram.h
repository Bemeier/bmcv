#ifndef INC_DRIVERS_FRAM_H_
#define INC_DRIVERS_FRAM_H_

// The FRAM part, as far as presets.c can see it: two calls against a flat
// address space, and no SPI, no HAL and no chip select.
//
// Shadows Core/Inc/Lib/fram.h on the include path so presets.c can be compiled
// and driven on the host. It deliberately does *not* declare fram_init - the
// real one takes an SPI handle and GPIO port, presets.c never calls it, and
// leaving it out means a test cannot accidentally depend on the driver's
// startup path.
//
// The backing store below is what makes the interesting cases reachable: the
// corrupt-record checks in preset_load can only be exercised by putting bytes
// in FRAM that preset_store would never write.

#include <stdint.h>

void fram_Write(uint16_t addr, const uint8_t* data, uint16_t len);
void fram_Read(uint16_t addr, uint8_t* data, uint16_t len);

/* ---- what only a test may do -------------------------------------------- */

#define FRAM_FAKE_SIZE 8192u

// Blank, as a part that has never been written reads. 0xFF rather than 0x00,
// which is what an erased part actually gives and is a different failure from
// a zeroed one - a zeroed header fails the magic check the tidy way, and this
// is the other way.
void fram_fake_reset(uint8_t fill);

// Reach past the driver to scribble on a stored record.
uint8_t* fram_fake_bytes(void);

#endif /* INC_DRIVERS_FRAM_H_ */
