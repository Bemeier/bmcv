// A FRAM part as a byte array. Reads and writes are clamped rather than
// wrapped: the real part wraps at its top, but a test that runs off the end is
// a test that has found a bug in the caller, and silently wrapping would hide
// exactly the out-of-range slot arithmetic preset_store guards against.

#include "fram.h"
#include <string.h>

static uint8_t mem[FRAM_FAKE_SIZE];

void fram_fake_reset(uint8_t fill) { memset(mem, fill, sizeof mem); }

uint8_t* fram_fake_bytes(void) { return mem; }

void fram_Write(uint16_t addr, const uint8_t* data, uint16_t len)
{
  if (addr >= FRAM_FAKE_SIZE)
    return;
  if ((uint32_t) addr + len > FRAM_FAKE_SIZE)
    len = (uint16_t) (FRAM_FAKE_SIZE - addr);
  memcpy(mem + addr, data, len);
}

void fram_Read(uint16_t addr, uint8_t* data, uint16_t len)
{
  if (addr >= FRAM_FAKE_SIZE)
  {
    memset(data, 0, len);
    return;
  }
  if ((uint32_t) addr + len > FRAM_FAKE_SIZE)
  {
    memset(data + (FRAM_FAKE_SIZE - addr), 0, len - (FRAM_FAKE_SIZE - addr));
    len = (uint16_t) (FRAM_FAKE_SIZE - addr);
  }
  memcpy(data, mem + addr, len);
}
