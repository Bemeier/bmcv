#include "midi_out.h"

#include "helpers.h"

#define QUEUE_MASK (MIDI_OUT_QUEUE_LEN - 1)
static_assert((MIDI_OUT_QUEUE_LEN & QUEUE_MASK) == 0, "queue length must be a power of two");

#define CC_NEVER 0xFF

void midi_out_init(MidiOut* o)
{
  o->head            = 0;
  o->tail            = 0;
  o->last_publish_us = 0;
  o->have_published  = 0;
  o->clk_slot        = 0;
  o->clk_running     = 0;

  for (uint8_t i = 0; i < MIDI_OUT_CC_COUNT; i++)
  {
    o->last_cc[i] = CC_NEVER;
  }
}

// Returns 0 when the queue is full. The caller decides what a drop means: a
// control change puts its CC back to CC_NEVER so the next slot re-sends it, and
// a clock message is simply lost, which is the better of the two failures - a
// late clock is a tempo glitch, a missing one is a beat of drift that the next
// message corrects.
static uint8_t push(MidiOut* o, uint8_t status, uint8_t d1, uint8_t d2, uint8_t len)
{
  const uint8_t next = (uint8_t) ((o->head + 1u) & QUEUE_MASK);
  if (next == o->tail)
    return 0;

  o->q[o->head].status = status;
  o->q[o->head].d1     = d1;
  o->q[o->head].d2     = d2;
  o->q[o->head].len    = len;
  o->head              = next;
  return 1;
}

uint8_t midi_out_scale7(int16_t dac_level)
{
  // +/-DAC_10V onto 0..127. In int32 because the shifted numerator reaches
  // 65535 * 127, which is well past what an int16 holds.
  //
  // The span is 2 * DAC_10V - 1 rather than 2 * DAC_10V because the converter
  // is signed and its positive side stops one count short - the same off-by-one
  // sim_volts_to_adc clamps to. Dividing by the round number instead puts full
  // scale at 126 and leaves 127 unreachable, since +DAC_10V does not fit in the
  // int16 this takes.
  const int32_t shifted = (int32_t) dac_level + (int32_t) DAC_10V;
  const int32_t scaled  = (shifted * 127) / (2 * (int32_t) DAC_10V - 1);
  return (uint8_t) iclamp((int) scaled, 0, 127);
}

static void publish_cc(MidiOut* o, uint8_t idx, uint8_t value)
{
  if (o->last_cc[idx] == value)
    return;

  if (push(o, MIDI_OUT_STATUS_CC, (uint8_t) (MIDI_OUT_CC_BASE + idx), value, 3))
  {
    o->last_cc[idx] = value;
  }
  else
  {
    o->last_cc[idx] = CC_NEVER; // dropped; say it again next slot
  }
}

// Where in the beat the 24-PPQN grid is. beat_phase is 0..1 - Clock_Poll wraps
// it into that range at both ends - so the multiply cannot leave the grid, but
// it is clamped anyway because a NaN reaching a cast is not worth the risk.
static uint8_t phase_slot(float beat_phase)
{
  const int slot = (int) (beat_phase * (float) MIDI_OUT_CLOCK_PPQN);
  return (uint8_t) iclamp(slot, 0, (int) MIDI_OUT_CLOCK_PPQN - 1);
}

// The module forwards its own clock, and only when a jack is what it is
// following. clock_source_is_midi is the flag input_fold already computes for
// the other direction: a patched cable wins and MIDI is the fallback, so this
// stays quiet exactly when the host is already the master. Sending then would
// regenerate the host's own clock back at it.
static void publish_clock(MidiOut* o, const EngineState* es, const HwState* hw)
{
  const uint8_t live = !hw->clock_source_is_midi && es->clock.have_beat;

  if (!live)
  {
    if (o->clk_running)
    {
      push(o, MIDI_RT_STOP, 0, 0, 1);
      o->clk_running = 0;
    }
    return;
  }

  const uint8_t slot = phase_slot(es->clock.beat_phase);

  // A reset restarts the far end rather than nudging it. Clock_Reset has
  // already zeroed beat_phase by the time this runs, so the slot taken here is
  // the downbeat the Start refers to.
  if (!o->clk_running || hw->clock_reset)
  {
    push(o, MIDI_RT_START, 0, 0, 1);
    push(o, MIDI_RT_CLOCK, 0, 0, 1);
    o->clk_running = 1;
    o->clk_slot    = slot;
    return;
  }

  uint8_t advance = (uint8_t) ((slot + MIDI_OUT_CLOCK_PPQN - o->clk_slot) % MIDI_OUT_CLOCK_PPQN);
  if (advance == 0)
    return;

  // Beyond a couple of slots the phase jumped rather than elapsed. Re-align
  // without replaying the gap.
  if (advance > MIDI_OUT_CLOCK_MAX_BURST)
    advance = 1;

  for (uint8_t i = 0; i < advance; i++)
  {
    push(o, MIDI_RT_CLOCK, 0, 0, 1);
  }
  o->clk_slot = slot;
}

void midi_out_publish(MidiOut* o, const EngineState* es, const HwState* hw, uint32_t now_us)
{
  publish_clock(o, es, hw);

  // Unsigned difference, so the 71-minute wrap needs no handling. The first
  // call publishes rather than waiting out a slot against a zero timestamp.
  if (o->have_published && (now_us - o->last_publish_us) < MIDI_OUT_PUBLISH_US)
    return;

  o->last_publish_us = now_us;
  o->have_published  = 1;

  // channels_gated_level, not channels_output_level: mute is an output-stage
  // gain and what leaves the module is the gated one, so a muted channel must
  // read as silent here too.
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    publish_cc(o, c, midi_out_scale7(es->channels_gated_level[c]));
  }

  // input_state is already in DAC units - input_fold scales the ADC reading by
  // four on the way in - so both halves of the block share one conversion.
  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    publish_cc(o, (uint8_t) (N_CHANNELS + i), midi_out_scale7(hw->input_state[i]));
  }
}

uint8_t midi_out_drain(MidiOut* o, MidiMsg* dst, uint8_t max)
{
  uint8_t n = 0;

  while (n < max && o->tail != o->head)
  {
    dst[n++] = o->q[o->tail];
    o->tail  = (uint8_t) ((o->tail + 1u) & QUEUE_MASK);
  }

  return n;
}
