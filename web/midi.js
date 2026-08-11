// The simulator on the MIDI bus.
//
// What to send is decided in the firmware's own midi_out.c and reaches here
// through sim.drainMidi() already framed - this file picks a port, drains once
// a frame, and does nothing else. Nothing about the mapping lives on this side,
// so the browser and the module cannot disagree about what a CC means.
//
// Two things are worth knowing before this is useful:
//
//   - Web MIDI is Chromium and Firefox. Safari does not ship it, and the page
//     says so rather than failing silently.
//   - a browser can only send to a port that already exists, and an operating
//     system does not provide one by default. macOS has the IAC driver (Audio
//     MIDI Setup > Window > MIDI Studio), Windows needs loopMIDI or similar,
//     Linux has ALSA's virmidi. With none of those the port list is empty and
//     there is nothing this can do about it.

import { sim, N_CH, N_IN } from './sim.js';

// The CC block, mirroring midi_out.h: the eight channels first, then the four
// inputs, all on MIDI channel 1.
const CC_BASE = 16;
const ROWS = [
  ...Array.from({ length: N_CH }, (_, c) => [`ch ${c}`, CC_BASE + c]),
  ...Array.from({ length: N_IN }, (_, i) => [`in ${i}`, CC_BASE + N_CH + i]),
];

const STATUS_CC = 0xB0;
const RT_CLOCK = 0xF8;
const RT_START = 0xFA;
const RT_STOP = 0xFC;

const el = id => document.getElementById(id);

const ui = {
  select: el('midi-port'),
  status: el('midi-status'),
  clock: el('midi-clock'),
  table: el('midi-cc'),
};

let access = null;
let port = null;

function setStatus(text, cls) {
  ui.status.textContent = text;
  ui.status.className = cls ? `value ${cls}` : 'value';
}

// Rebuilt rather than diffed: the list is a handful of entries and it changes
// only when hardware is plugged in, so keeping the selection is the only state
// worth carrying across.
function refreshPorts() {
  const keep = port?.id;
  ui.select.replaceChildren();

  const none = document.createElement('option');
  none.value = '';
  none.textContent = 'off';
  ui.select.append(none);

  for (const output of access.outputs.values()) {
    const opt = document.createElement('option');
    opt.value = output.id;
    opt.textContent = output.name;
    ui.select.append(opt);
  }

  if (keep && access.outputs.get(keep)) {
    ui.select.value = keep;
  } else if (keep) {
    // The port went away underneath us - stop sending to it.
    port = null;
    setStatus('port disconnected', 'warn');
  }
}

function selectPort(id) {
  port = id ? access.outputs.get(id) ?? null : null;
  setStatus(port ? `sending to ${port.name}` : 'off');
}

export async function initMidi() {
  if (!navigator.requestMIDIAccess) {
    ui.select.disabled = true;
    setStatus('no WebMIDI in this browser', 'warn');
    return;
  }

  try {
    // No sysex: this only sends control changes and clock. Asking for it anyway
    // would put a scarier permission prompt in front of a page that does not
    // need one - the update page asks, because it genuinely does.
    access = await navigator.requestMIDIAccess();
  } catch (err) {
    ui.select.disabled = true;
    setStatus(`access refused: ${err.message}`, 'bad');
    return;
  }

  refreshPorts();
  access.onstatechange = refreshPorts;
  ui.select.addEventListener('change', () => selectPort(ui.select.value));

  setStatus(access.outputs.size ? 'off' : 'no MIDI outputs on this machine', 'warn');
}

/* ---- what is going out -------------------------------------------------- */
//
// The last value seen for each CC, and whether the clock is being forwarded.
// Tracked here rather than recomputed from the engine, so the readout shows
// what was actually sent - including the 7-bit quantisation and the clipping
// the channel's clamp mode imposes, which is the point of showing it.

const sent = new Array(ROWS.length).fill(null);
let clockRunning = null; // null until the module says one way or the other

function observe(msg) {
  if (msg[0] === STATUS_CC) {
    const idx = msg[1] - CC_BASE;
    if (idx >= 0 && idx < sent.length) sent[idx] = msg[2];
    return;
  }
  if (msg[0] === RT_START) clockRunning = true;
  if (msg[0] === RT_STOP) clockRunning = false;
}

// Drain every frame regardless of whether a port is selected, so the queue does
// not sit full and hand a freshly chosen port a burst of stale values - and so
// the readout below works with no MIDI port on the machine at all, which is the
// common case.
//
// The module re-states a dropped control change on its next publish slot, so
// throwing these away costs at most one slot of staleness.
export function pumpMidi() {
  for (const msg of sim.drainMidi()) {
    observe(msg);

    if (!port) continue;
    try {
      port.send(msg);
    } catch {
      // A port that has gone away throws on send. Drop it and let the next
      // statechange repopulate the list.
      port = null;
      setStatus('send failed, port dropped', 'bad');
    }
  }
}

/* ---- the readout -------------------------------------------------------- */

ui.table.innerHTML =
  '<tr><th>src</th><th>cc</th><th>val</th><th>level</th></tr>' +
  ROWS.map(([name, cc], i) =>
    `<tr data-cc="${i}"><td>${name}</td><td>${cc}</td><td>—</td><td class="bar-cell"></td></tr>`).join('');
const ccRows = [...ui.table.querySelectorAll('tr[data-cc]')];

const BAR_TRACK = '#2b2f36';
const BAR_FILL = '#5b8ff9';

// Pinned to 0 or 127: the channel's clamp is cutting the swing off. Worth
// marking rather than hiding - clipping against the clamp is a thing you might
// be doing on purpose.
const BAR_RAILED = '#f2b134';

const setText = (node, value) => { if (node.textContent !== value) node.textContent = value; };

// null is "nothing said yet", which covers both reasons the module stays quiet:
// MIDI is the clock source, or no beat has been detected on the jack. Naming
// either one here would be a guess, so the note in index.html says when it
// sends and this only says whether it is.
const CLOCK_TEXT = {
  true: 'forwarding at 24 PPQN',
  false: 'stopped',
  null: 'not sending',
};

export function drawMidi() {
  setText(ui.clock, CLOCK_TEXT[clockRunning]);

  for (let i = 0; i < ccRows.length; i++) {
    const cells = ccRows[i].children;
    const v = sent[i];
    setText(cells[2], v === null ? '—' : String(v));

    const pct = ((v ?? 0) / 127) * 100;
    const fill = v === 0 || v === 127 ? BAR_RAILED : BAR_FILL;
    cells[3].style.background =
      `linear-gradient(to right, ${fill} ${pct}%, ${BAR_TRACK} ${pct}%)`;
  }
}
