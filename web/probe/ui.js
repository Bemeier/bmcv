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
import { mode, SIM, USB, PROBE, SOURCE_NAME } from '../mode.js';
import { probe, webusbAvailable } from './probe.js';
import { usblink } from './usblink.js';

const el = id => document.getElementById(id);

const links = { [USB]: usblink, [PROBE]: probe };

// Whichever link is doing something, or null when the simulation is running and
// nothing has gone wrong. An error outlives the attempt that caused it, so it
// stays on screen long enough to read.
function speaking() {
  for (const link of [usblink, probe]) {
    if (link.state === 'connecting' || link.state === 'live') return link;
  }
  for (const link of [usblink, probe]) {
    if (link.state === 'error') return link;
  }
  return null;
}

/* ---- what each source has to say ----------------------------------------- */

// A module over its own cable.
const usbDetail = () => usblink.description;

// A module over a probe. The read time is the diagnostic that matters when the
// rate disappoints: against the interval between snapshots it says whether the
// USB is the limit or something else on the page is.
const probeDetail = () =>
  `${probe.link.version.name}, target ${probe.voltage.toFixed(2)}V, firmware ${probe.info.version}, `
  + `${probe.link.readMs.toFixed(1)}ms per read in ${Math.ceil(probe.info.instanceSize / probe.link.block)}`;

// Why the simulation is running rather than a module, when that is a fact about
// the browser rather than a choice.
function simDetail() {
  if (!webusbAvailable) {
    return 'This browser has no WebUSB, so it cannot reach a module at all. Chromium has it; '
      + 'the simulation itself works everywhere.';
  }
  return '';
}

function render() {
  const link = speaking();
  const name = el('source-name');
  const detail = el('source-detail');

  // Three buttons, one per source: a switch that also says which way it is
  // thrown. The current source is the disabled one - there is nothing to
  // switch to when you are already there - and the only other reason a button
  // is disabled is a browser that cannot reach that source at all.
  //
  // Everything else stays clickable, including while a connection is being
  // made, because a connection that is going nowhere is exactly when you want
  // to pick something else.
  for (const [id, source] of [['src-sim', SIM], ['src-usb', USB], ['src-probe', PROBE]]) {
    const button = el(id);
    const isCurrent = mode.current === source;
    button.classList.toggle('active', isCurrent);
    button.disabled = isCurrent || (source !== SIM && !webusbAvailable);
  }

  if (!link || link.state === 'error') {
    name.textContent = SOURCE_NAME[SIM];
    name.className = link ? 'value warn' : 'value on-sim';
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
  name.textContent = paused ? 'paused' : SOURCE_NAME[link === probe ? PROBE : USB];
  name.className = paused ? 'value warn' : `value on-${link === probe ? 'probe' : 'usb'}`;

  detail.textContent = paused
    ? 'The tab is in the background, where browsers throttle timers to about one '
      + 'a second and stop animation frames entirely. Polling is stopped rather '
      + 'than reduced to a trickle; it resumes when the tab is shown. To watch a '
      + 'module while working elsewhere, put this page in its own window and leave '
      + 'it visible - a window that is not focused is not throttled, only one that '
      + 'is hidden.'
    : link === probe ? probeDetail() : usbDetail();
}

/* ---- switching ----------------------------------------------------------- */

// Serialised, because these are buttons and buttons get clicked twice.
let switching = null;

// Put the page on a source, whatever it was on before.
//
// Whatever is live gives the page up first, and is waited for. Two links at
// once is not a thing that half-works: they import into the same wasm instance
// and each publishes its own source every frame, so the page ends up alternating
// between them several times a second - which is what it did, visibly, once the
// buttons stopped being disabled while another link was busy.
//
// Connecting is left to fail on its own terms. If it does, the page is already
// back on the simulation and the error says why.
function switchTo(source) {
  if (switching) return switching;

  switching = (async () => {
    for (const other of [usblink, probe]) {
      if (other !== links[source] && (other.state === 'live' || other.state === 'connecting')) {
        await other.disconnect();
      }
    }
    if (source !== SIM) await links[source].connect();
  })().finally(() => { switching = null; });

  return switching;
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

  // Named after whatever is carrying it. "snapshots" said what the number
  // counted; this says what it is a property of, which is the thing that
  // changes between one connection and the next.
  rateEls.label.textContent = mode.current === PROBE ? 'probe link'
    : mode.current === USB ? 'usb link' : 'link';

  rateEls.engine.textContent = rateText(sim.engineFps());
  rateEls.dac.textContent = rateText(sim.dacFps());

  // Zero everywhere but on hardware - the simulator has no LED driver to flush
  // to - and rateText already renders that as a dash.
  rateEls.leds.textContent = rateText(sim.ledFps());
}

export function initProbe() {
  if (!el('src-usb')) return;

  rateEls.snapshots = el('r-probe-hz');
  rateEls.label = el('r-link-label');
  rateEls.engine = el('r-engine-fps');
  rateEls.dac = el('r-dac-fps');
  rateEls.leds = el('r-led-fps');

  usblink.onchange = render;
  probe.onchange = render;

  el('src-sim').addEventListener('click', () => switchTo(SIM));
  el('src-usb').addEventListener('click', () => switchTo(USB));
  el('src-probe').addEventListener('click', () => switchTo(PROBE));

  mode.onChange(render);
}
