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
#include <stddef.h>
#include <stdint.h>

// What a host outside this module can ask it to do.
//
// The remote input mailbox in input_fold.h covers everything a *panel* can say.
// This covers the two things a panel cannot: start again, and forget what was
// saved. Both are destructive and neither is a level, so this is an edge -
// acted on once per change of `seq`, however often the mailbox is rewritten.
//
// Separate from RemoteInput, and written only when someone asks for it, because
// the two have opposite shapes. Input is a level re-sent continuously and read
// every tick; a command is a one-off whose whole risk is being performed twice.
typedef enum
{
  REMOTE_OP_NONE = 0,

  // Back to power-on, keeping whatever is in storage. What the panel has no
  // gesture for and a debugger would otherwise do by halting the core.
  REMOTE_OP_RESET = 1,

  // The same, having first forgotten every stored preset. The module comes back
  // on its first-boot defaults.
  REMOTE_OP_RESET_WIPE = 2,
} RemoteOp;

typedef struct
{
  uint8_t op; // a RemoteOp
  uint8_t _pad[3];

  // Bumped by the writer after `op` is set. Zero means nothing has ever been
  // asked, which is what a freshly booted module holds - so a host that starts
  // its own count at one cannot have its first command mistaken for silence.
  uint32_t seq;
} RemoteCommand;

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

  // Written by whoever is driving this module from outside it, and by nobody
  // here. See RemoteCommand.
  RemoteCommand command;

  // The last command acted on. Outside the mailbox for the reason the input
  // layer keeps its own bookkeeping outside RemoteInput: a writer that saw this
  // change under it could not tell a stale mailbox from a fresh one.
  uint32_t command_seq;
  // Derived state, and the last member on purpose.
  //
  // What each channel remembers of the pattern it is playing: the step pair and
  // the slot values. Rebuilt from the setting whenever the setting moves, and
  // meaningful to nobody but that channel's own evaluation - so a host copying
  // the module out stops before it, which is what BMCV_SNAPSHOT_BYTES is.
  //
  // It is 2.7KB of the 5.9KB this struct would otherwise be. Snapshotting it
  // doubled what the USB link ships per frame to say nothing a decoder can use.
  ChannelScratch channel_scratch[N_CHANNELS];

} BmcvInstance;

// How much of the instance a host copies out to show the module.
//
// Everything up to `stepped_scratch`, which is derived and rebuilds itself from
// the setting. It is what usblink ships, what the probe descriptor advertises,
// and what bmcv_sim_import() accepts - the three have to agree, so they all
// read this.
#define BMCV_SNAPSHOT_BYTES (offsetof(BmcvInstance, channel_scratch))

// Point every pointer in the instance at the right part of the instance, and at
// this host's setup tables and preset store. Everything else here is plain data.
//
// Split out of bmcv_instance_init because it is also what makes an instance
// that arrived as bytes usable: a snapshot lifted out of another module's RAM
// carries that module's addresses, which mean nothing here. See
// bmcv_sim_import(). `io` may be NULL, as it may be for init.
//
// Wiring only - it initialises nothing and overwrites no state, so it is safe
// to call on a full instance.
void bmcv_instance_wire(BmcvInstance* m, const PresetIo* io);

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

// Perform a command left in the mailbox, if one is pending, and return non-zero
// if anything was done. Call from the host loop between ticks - it may rebuild
// the whole instance, which is not something to do underneath an engine tick.
//
// Idempotent per command: acting on it records the sequence number, so a
// mailbox that keeps arriving - which is what both transports do - resets the
// module once rather than continuously.
uint8_t bmcv_instance_take_command(BmcvInstance* m, const PresetIo* io, uint32_t now_us);

#endif /* INC_LIB_INSTANCE_H_ */
