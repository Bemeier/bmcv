// Stand-ins for the few driver functions the logic layer still calls
// directly. Linked only into the native (host) build.
//
// LED output is deliberately absent: the UX layer renders into
// EngineState.leds[] via led_fb.c, so tests read the framebuffer straight
// from the state struct and no LED fake is needed.
#include "dac_adc.h"
#include "presets.h"

// Tests drive CV inputs through HwState.input_state[], which the engine reads
// directly; get_adc is only reached by the MON-mode LED path. Give it a
// backing store when a test actually needs one.
int16_t get_adc(uint8_t channel)
{
  (void) channel;
  return 0;
}

uint8_t adc_read_trig_state(uint8_t channel)
{
  (void) channel;
  return 0;
}

void dacadc_write(uint8_t idx, int16_t data)
{
  (void) idx;
  (void) data;
}

int8_t preset_store(EngineConfig* cfg, int8_t dst)
{
  (void) cfg;
  (void) dst;
  return 1;
}

int8_t preset_load(EngineConfig* cfg, int8_t src)
{
  (void) cfg;
  (void) src;
  return 0; // "no stored preset", the path bmcv_init treats as first boot
}
