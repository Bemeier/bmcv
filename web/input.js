// Where a gesture on the panel goes.
//
// There are two modules it could mean: the simulation running in this tab, or
// the physical one a probe is attached to. Both take the same three gestures,
// so the panel does not need to know which is listening - it presses a button
// and this decides whose button it was.
//
// Small on purpose. It exists because the alternative was the panel asking
// `mode.live` at every call site, which it used to do to answer the opposite
// question - whether to swallow the gesture entirely, since with nowhere to
// send it the honest thing was to do nothing.
//
// CV is missing here deliberately: the input jacks on a physical module are
// physical, and a fader on this page cannot drive a voltage into one. See
// inputs.js, which stays pointed at the simulation.

import { sim } from './sim.js';
import { mode } from './mode.js';

export const input = {
  setButton: (index, down) =>
    mode.live ? sim.remoteButton(index, down) : sim.setButton(index, down),

  addEncoder: (index, detents) =>
    mode.live ? sim.remoteEncoder(index, detents) : sim.addEncoder(index, detents),

  // 0..1 over the crossfader's travel.
  //
  // On hardware this is a claim rather than a position: it holds the module's
  // fader until this page hands it back, or until someone moves the physical
  // one far enough to take it back. Nothing here has to track which of those
  // happened - the module decides, and the handle is drawn from what it says.
  setSlider01: pos => (mode.live ? sim.remoteSlider01(pos) : sim.setSlider01(pos)),
};
