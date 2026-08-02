// Stand-in for the one driver function the logic layer still calls directly.
// Linked only into the native (host) build.
//
// LED output is deliberately absent: the UX layer renders into
// EngineState.leds[] via led_fb.c, so tests read the framebuffer straight
// from the state struct and no LED fake is needed.
//
// CV input needs no fake either - it arrives as an InputSample. Nor do
// presets: they go through UxState.presets, which a test either leaves NULL
// (store is a no-op, load reports "nothing stored") or points at its own
// counting backend.
#include "dac_adc.h"

// Reached only by write_channel_dac, which the host side never calls - it
// reads EngineState.channels_output_level[] instead.
void dacadc_write(uint8_t idx, int16_t data)
{
  (void) idx;
  (void) data;
}
