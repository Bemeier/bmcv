// The probe box: one button, and enough status to tell the failures apart.
//
// Kept away from probe.js so that the connection logic has no idea a DOM
// exists - which is what lets it be reasoned about, and what stops "is the
// module powered" from turning into a string comparison against a label.

import { sim } from '../sim.js';
import { probe, webusbAvailable } from './probe.js';

const el = id => document.getElementById(id);

// Each state says what is true and what to do about it. The error text comes
// from probe.js, which knows what actually went wrong.
function render(p) {
  const button = el('probe-connect');
  const status = el('probe-status');
  const detail = el('probe-detail');

  switch (p.state) {
  case 'idle':
    button.textContent = 'Connect a module';
    button.disabled = false;
    // The line is labelled "source", so it says what is driving the page rather
    // than what is absent from it.
    status.textContent = 'simulated';
    status.className = 'value muted';
    detail.textContent = '';
    break;

  case 'connecting':
    button.textContent = 'Connecting…';
    button.disabled = true;
    status.textContent = 'connecting';
    status.className = 'value';
    // Which step, as it happens. A connection that stalls then says where,
    // which is the difference between a bug report and a diagnosis.
    detail.textContent = p.stage;
    break;

  case 'live':
    button.textContent = 'Disconnect';
    button.disabled = false;
    // Paused is still connected: the probe is open and the module is untouched,
    // the page has just stopped asking. Saying "live" through it would be a lie
    // about numbers that have stopped moving.
    status.textContent = p.paused ? 'paused' : 'live';
    status.className = p.paused ? 'value warn' : 'value ok';
    detail.textContent = p.paused
      ? 'The tab is in the background, where browsers throttle timers to about '
        + 'one a second and stop animation frames entirely. Polling is stopped '
        + 'rather than reduced to a trickle; it resumes when the tab is shown. '
        + 'To watch a module while working elsewhere, put this page in its own '
        + 'window and leave it visible - a window that is not focused is not '
        + 'throttled, only one that is hidden.'
      :
      // The read time is the diagnostic that matters when the rate is
      // disappointing: against the interval between snapshots it says whether
      // the USB is the limit or something else on the page is.
      `${p.link.version.name}, target ${p.voltage.toFixed(2)}V, firmware ${p.info.version}, `
      + `${p.link.readMs.toFixed(1)}ms per read in ${Math.ceil(p.info.instanceSize / p.link.block)}`;
    break;

  case 'error':
    button.textContent = 'Try again';
    button.disabled = false;
    status.textContent = 'simulated';
    status.className = 'value warn';
    detail.textContent = p.error;
    break;
  }
}

// The rates, redrawn on the readout cadence rather than per frame - they are
// smoothed averages and nobody reads three significant figures at 60Hz.
//
// Three of the four are the module's own measurements of itself, lifted out of
// whatever snapshot last landed, so they say what the hardware is managing and
// not what this page is. The fourth is the only one this page owns.
const rateEls = {};

// All four are rates in Hz and are read side by side, so they carry the unit
// and switch to kHz at the same threshold rather than one of them silently
// meaning something else.
function rateText(hz) {
  if (!isFinite(hz) || hz <= 0) return '—';
  return hz >= 1000 ? `${(hz / 1000).toFixed(1)} kHz` : `${hz.toFixed(0)} Hz`;
}

export function drawProbeRates() {
  if (!rateEls.probe) return;

  // The detail line carries the measured read time, which moves - so it is
  // redrawn with the rates rather than written once when the state changed.
  if (probe.state === 'live' && !probe.paused) render(probe);

  // Blank rather than zero when nothing is connected: this is the one number
  // that is a property of the connection rather than of the module.
  rateEls.probe.textContent = probe.state === 'live' ? rateText(probe.hz) : '—';

  rateEls.engine.textContent = rateText(sim.engineFps());
  rateEls.dac.textContent = rateText(sim.dacFps());

  // Zero everywhere but on hardware - the simulator has no LED driver to flush
  // to - and rateText already renders that as a dash.
  rateEls.leds.textContent = rateText(sim.ledFps());
}

export function initProbe() {
  const button = el('probe-connect');
  if (!button) return;

  rateEls.probe = el('r-probe-hz');
  rateEls.engine = el('r-engine-fps');
  rateEls.dac = el('r-dac-fps');
  rateEls.leds = el('r-led-fps');

  if (!webusbAvailable) {
    button.disabled = true;
    button.textContent = 'Needs Chromium';
    el('probe-status').textContent = 'no WebUSB in this browser';
    el('probe-status').className = 'value warn';
    el('probe-detail').textContent =
      'Reading a module needs WebUSB, which only Chromium-based browsers have. ' +
      'The simulator itself works everywhere.';
    return;
  }

  probe.onchange = render;
  button.addEventListener('click', () => {
    // Both directions from one button: the label says which it is doing.
    if (probe.state === 'live') probe.disconnect();
    else probe.connect();
  });

  render(probe);
}
