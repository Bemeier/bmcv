// The source box: which module the page is showing, over what, and how well.
//
// Kept away from the transports so that neither has any idea a DOM exists -
// which is what lets them be reasoned about, and what stops "is the module
// powered" from turning into a string comparison against a label.
//
// One renderer for three sources rather than one per transport. The previous
// arrangement had a render function per link writing into the same two
// elements, and they could disagree: connecting over one while the other sat in
// an error state left the error text underneath a "live" status.

import { sim } from '../sim.js';
import { mode, SIM, MIDI, PROBE, SOURCE_NAME } from '../mode.js';
import { probe, webusbAvailable } from './probe.js';
import { midilink, webmidiAvailable } from './midilink.js';

const el = id => document.getElementById(id);

const links = { [MIDI]: midilink, [PROBE]: probe };

// Whichever link is doing something, or null when the simulation is running and
// nothing has gone wrong. An error outlives the attempt that caused it, so it
// stays on screen long enough to read.
function speaking() {
  for (const link of [midilink, probe]) {
    if (link.state === 'connecting' || link.state === 'live') return link;
  }
  for (const link of [midilink, probe]) {
    if (link.state === 'error') return link;
  }
  return null;
}

/* ---- what each source has to say ----------------------------------------- */

// A module over USB: which port it turned out to be, since the name is exactly
// the thing that cannot be relied on to find it.
const midiDetail = () =>
  `${midilink.input?.name || 'unnamed port'}, firmware ${midilink.version}, no probe attached`;

// A module over a probe. The read time is the diagnostic that matters when the
// rate disappoints: against the interval between snapshots it says whether the
// USB is the limit or something else on the page is.
const probeDetail = () =>
  `${probe.link.version.name}, target ${probe.voltage.toFixed(2)}V, firmware ${probe.info.version}, `
  + `${probe.link.readMs.toFixed(1)}ms per read in ${Math.ceil(probe.info.instanceSize / probe.link.block)}`;

// Why the simulation is running rather than a module, when that is a fact about
// the browser rather than a choice.
function simDetail() {
  if (!webmidiAvailable && !webusbAvailable) {
    return 'This browser has neither Web MIDI nor WebUSB, so it cannot reach a module at all. '
      + 'The simulation itself works everywhere.';
  }
  if (!webmidiAvailable) return 'This browser has no Web MIDI, so a module can only be reached through a debug probe.';
  if (!webusbAvailable) return 'This browser has no WebUSB, so the debug probe route is unavailable. Chromium has it.';
  return '';
}

function render() {
  const link = speaking();
  const name = el('source-name');
  const detail = el('source-detail');

  el('connect-midi').textContent = midilink.state === 'live' ? 'Disconnect'
    : midilink.state === 'connecting' ? 'Finding…' : 'Connect over USB';
  el('connect-probe').textContent = probe.state === 'live' ? 'Disconnect'
    : probe.state === 'connecting' ? 'Connecting…' : 'Use a debug probe';

  // Only the busy link can be clicked, so the other cannot be started
  // underneath it - the page has one wasm instance and both would import into
  // it.
  const busy = link && link.state !== 'error';
  el('connect-midi').disabled = !webmidiAvailable || (busy && link !== midilink);
  el('connect-probe').disabled = !webusbAvailable || (busy && link !== probe);

  if (!link || link.state === 'error') {
    name.textContent = SOURCE_NAME[SIM];
    name.className = link ? 'value warn' : 'value muted';
    detail.textContent = link ? link.error : simDetail();
    return;
  }

  if (link.state === 'connecting') {
    name.textContent = 'connecting';
    name.className = 'value';
    // Which step, as it happens. A connection that stalls then says where,
    // which is the difference between a bug report and a diagnosis.
    detail.textContent = link.stage;
    return;
  }

  // Paused is still connected: the probe is open and the module untouched, the
  // page has just stopped asking. Saying "live" through it would be a lie about
  // numbers that have stopped moving.
  const paused = link === probe && probe.paused;
  name.textContent = paused ? 'paused' : SOURCE_NAME[link === probe ? PROBE : MIDI];
  name.className = paused ? 'value warn' : 'value ok';

  detail.textContent = paused
    ? 'The tab is in the background, where browsers throttle timers to about one '
      + 'a second and stop animation frames entirely. Polling is stopped rather '
      + 'than reduced to a trickle; it resumes when the tab is shown. To watch a '
      + 'module while working elsewhere, put this page in its own window and leave '
      + 'it visible - a window that is not focused is not throttled, only one that '
      + 'is hidden.'
    : link === probe ? probeDetail() : midiDetail();
}

/* ---- the rates ----------------------------------------------------------- */

// Redrawn on the readout cadence rather than per frame - they are smoothed
// averages and nobody reads three significant figures at 60Hz.
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
  if (!rateEls.snapshots) return;

  const link = links[mode.current];

  // The detail line carries measurements that move, so it is redrawn with the
  // rates rather than written once when the state changed.
  if (link && link.state === 'live') render();

  // Blank rather than zero when nothing is connected: these are properties of
  // the link rather than of the module.
  rateEls.snapshots.textContent = link ? rateText(link.hz) : '—';
  el('source-link').textContent = link ? `${rateText(link.hz)}, ${link.snapshots} snapshots` : '—';

  rateEls.engine.textContent = rateText(sim.engineFps());
  rateEls.dac.textContent = rateText(sim.dacFps());

  // Zero everywhere but on hardware - the simulator has no LED driver to flush
  // to - and rateText already renders that as a dash.
  rateEls.leds.textContent = rateText(sim.ledFps());
}

export function initProbe() {
  if (!el('connect-midi')) return;

  rateEls.snapshots = el('r-probe-hz');
  rateEls.engine = el('r-engine-fps');
  rateEls.dac = el('r-dac-fps');
  rateEls.leds = el('r-led-fps');

  midilink.onchange = render;
  probe.onchange = render;

  // Both buttons toggle, and the label says which way.
  el('connect-midi').addEventListener('click', () => {
    if (midilink.state === 'live') midilink.disconnect();
    else midilink.connect();
  });
  el('connect-probe').addEventListener('click', () => {
    if (probe.state === 'live') probe.disconnect();
    else probe.connect();
  });

  mode.onChange(render);
}
