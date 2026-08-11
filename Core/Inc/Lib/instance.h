#ifndef INC_LIB_INSTANCE_H_
#define INC_LIB_INSTANCE_H_

#include "config.h"
#include "engine_state.h"
#include "hw_setup.h"
#include "hw_state.h"
#include "input_fold.h"
#include "midi_out.h"
#include "ui_state.h"
#include "ux_setup.h"
#include "ux_state.h"
#include <stdint.h>

// One module, in one struct.
//
// Everything a BMCV needs to run and nothing that belongs to a particular
// host. The firmware keeps a single static one; a simulator or a VCV Rack
// patch keeps one per instance, which is why the clock and the error flags
// moved out of file scope and into EngineState.
//
// Allocate it, call bmcv_instance_init(), then feed it InputSamples. What
// comes out is EngineState: channels_output_level[] for the DAC and leds[]
// for the LED driver. Pushing those anywhere is the host's job.
typedef struct
{
  const HwSetup* hw_setup;
  const UxSetup* ux_setup;

  EngineConfig engine_config;
  EngineState engine_state;
  UiState ui_state;

  UxState ux;
  InputFrames input;

  // What the module publishes on the MIDI bus. Here rather than in EngineState
  // because that struct is the signal path and this is not - a host that never
  // drains the queue simply never sends anything.
  MidiOut midi_out;
} BmcvInstance;

// Bring the module up to its power-on state: wire the UxState, initialise the
// clock and the channels, load the stored config through `io` (or apply the
// first-boot defaults if there is none), validate it, and baseline the input
// layer.
//
// `io` may be NULL, in which case there is never a stored config and the
// first-boot defaults always apply.
void bmcv_instance_init(BmcvInstance* m, const PresetIo* io, uint32_t now_us);

// One tick: fold the sample into HwState, then run the engine. Returns the
// `dirty` flag, non-zero when a button level changed or an encoder moved.
//
// The firmware does not use this - it needs to interleave its FPS accounting
// between the two halves - but every host that has no such requirement should.
uint8_t bmcv_instance_tick(BmcvInstance* m, const InputSample* sample, uint32_t now_us);

#endif /* INC_LIB_INSTANCE_H_ */
