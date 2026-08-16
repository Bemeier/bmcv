#ifndef INC_LIB_BMCV_H_
#define INC_LIB_BMCV_H_

#include "instance.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

// The module. Everything the firmware runs on lives inside this one struct -
// see instance.h - and it is named here rather than kept file-static purely so
// a live debugger has something to attach to: every measurement worth watching
// on hardware (engine_state.dac_fps, engine_state.clock.bpm, the output levels,
// the folded input frame) is a member path from here, at a fixed address.
//
// Nothing in the firmware should reach for it. The calls below take what they
// need, and other hosts allocate their own instance - which is the whole
// reason the clock and the error flags stopped being globals.
extern BmcvInstance bmcv;

// One measured stretch of the tick, in CPU cycles and in microseconds. Filled
// from the DWT cycle counter - see the profiling block in bmcv.c.
//
// min and max are sticky, so a worst case that happened once is still there
// when you get around to looking at it. Both treat 0 as "unset", so writing 0
// to either from a live debugger re-arms it for the next tick.
typedef struct
{
  uint32_t last_cycles;
  uint32_t min_cycles;
  uint32_t max_cycles;
  uint32_t max_at_tick; // which tick set max - a sticky max reached once during
                        // startup reads the same as one reached every 8ms, and
                        // this is what tells the two apart
  float last_us;
  float avg_us; // exponential average, smoothed like the fps readouts
  float min_us;
  float max_us;
} BmcvSpan;

// Where the time inside one 4kHz tick goes. A global at a fixed address for the
// same reason `bmcv` is one: so a debugger can watch it while the module runs.
typedef struct
{
  BmcvSpan engine; // engine_tick() on its own
  BmcvSpan tick;   // the whole gated block: input read, engine, DAC writes
  float load;      // tick.avg_us against ENGINE_TICK_US - 1.0 is the whole budget
  uint32_t ticks;  // ticks measured since boot

  // Ticks whose gated block ran past ENGINE_TICK_US, so the next tick's gate was
  // already open when they finished. Against `ticks` this is a rate: a UX pass
  // every 8ms would put it near 1 in 31, a startup transient leaves it at 1.
  uint32_t overruns;

  // Ticks dropped outright: the schedule had fallen a whole period behind and
  // was resynchronised to now rather than repaid as a burst. An overrun is the
  // loop running late and catching up; a resync is a tick that never happened,
  // and any number here that keeps climbing means the tick period is too short
  // for the work in it.
  uint32_t resyncs;
} BmcvProfile;

extern BmcvProfile bmcv_profile;

void bmcv_init(uint16_t mpc_interrupt_pin, ADC_TypeDef* slider_adc);

void bmcv_main(uint32_t now_us);

uint8_t bmcv_state_update(uint32_t now);

void bmcv_flush_leds(void);

void bmcv_handle_adc_conversion_complete(ADC_HandleTypeDef* hadc);

void bmcv_poll_tasks();

void bmcv_handle_gpio_exti(uint16_t GPIO_Pin);

// Called from the SPI completion interrupt. `now_us` comes from the caller for
// the same reason bmcv_main()'s does: the timebase is the host's, and this file
// does not reach for it.
void bmcv_handle_txrx_complete(SPI_HandleTypeDef* hspi, uint32_t now_us);

#endif /* INC_LIB_BMCV_H_ */
