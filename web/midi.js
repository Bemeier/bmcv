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
  box: el('midi'),
  select: el('midi-port'),
  status: el('midi-status'),
  clock: el('midi-clock'),
  table: el('midi-cc'),
};

// The CC table is a picture of what is going out of a port. With no port there
// is nothing going out, so it is a picture of nothing - twelve rows of 64 that
// look like data. The port and the status stay; everything that only means
// something once a port is chosen is hidden with this.
function setSending(on) {
  ui.box.classList.toggle('sending', on);
}

let access = null;
let port = null;

// Never a dash. Every other readout on the page uses one for "no number yet",
// which is a different thing from a port that is deliberately not sending -
// this one always knows its own answer, and "off" is an answer.
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
    setStatus('port gone', 'warn');
    setSending(false);
  }
}

function selectPort(id) {
  port = id ? access.outputs.get(id) ?? null : null;
  // Just the verb. The port's name is already in the picker directly above,
  // and repeating it made the longest string on the page out of the least
  // interesting fact on it.
  //
  // Coloured the way the probe's source line is - muted for idle, green for
  // carrying something, amber for a problem - so the two status readouts in
  // this box mean the same thing by the same colour.
  setStatus(port ? 'sending' : 'off', port ? 'ok' : 'muted');
  setSending(!!port);
}

export async function initMidi() {
  if (!navigator.requestMIDIAccess) {
    ui.select.disabled = true;
    setStatus('unsupported', 'warn');
    return;
  }

  try {
    // No sysex: this only sends control changes and clock. Asking for it anyway
    // would put a scarier permission prompt in front of a page that does not
    // need one - the update page asks, because it genuinely does.
    access = await navigator.requestMIDIAccess();
  } catch (err) {
    ui.select.disabled = true;
    setStatus('refused', 'warn');
    return;
  }

  refreshPorts();
  access.onstatechange = refreshPorts;
  ui.select.addEventListener('change', () => selectPort(ui.select.value));

  setStatus(access.outputs.size ? 'off' : 'no outputs', access.outputs.size ? 'muted' : 'warn');
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
      setStatus('send failed', 'warn');
    }
  }
}

/* ---- the readout -------------------------------------------------------- */

// Twelve narrow meters rather than twelve table rows.
//
// A control change is one number between 0 and 127, and what is worth seeing is
// where twelve of them are relative to each other and which are moving. A row
// of vertical tracks with a mark riding up and down each shows that at a
// glance; a table of filled horizontal bars was twelve reading tasks, and the
// fill was carrying colour and weight that the value did not deserve.
//
// The mark is a line, not a bar, for the same reason: it says where the value
// is without also drawing everything below it.
ui.table.innerHTML = ROWS.map(([name, cc], i) =>
  `<div class="meter" data-cc="${i}">
     <div class="meter-track"><i></i></div>
     <span class="meter-name">${name.replace(' ', '')}</span>
     <span class="meter-cc">${cc}</span>
   </div>`).join('');

const meters = [...ui.table.querySelectorAll('.meter')].map(m => ({
  mark: m.querySelector('i'),
  root: m,
}));

const setText = (node, value) => { if (node.textContent !== value) node.textContent = value; };

// One word each. This sits in a row of readouts whose values are numbers, and a
// sentence where they have a figure makes the row look broken - the rate it
// forwards at is a constant of the protocol and belongs in the note below,
// not in a value that changes.
//
// null is "nothing said yet", which covers both reasons the module stays quiet:
// MIDI is the clock source, or no beat has been detected on the jack. Naming
// either one here would be a guess.
const CLOCK_TEXT = {
  true: 'sending',
  false: 'stopped',
  null: 'silent',
};

export function drawMidi() {
  setText(ui.clock, CLOCK_TEXT[clockRunning]);

  for (let i = 0; i < meters.length; i++) {
    const { mark, root } = meters[i];
    const v = sent[i];

    // 0 at the bottom, 127 at the top, which is the way the value reads on a
    // fader and the way the input scopes below it are drawn.
    mark.style.bottom = `${((v ?? 64) / 127) * 100}%`;

    // Pinned to either end: the channel's clamp is cutting the swing off.
    // Worth marking rather than hiding - clipping against a clamp is a thing
    // you might be doing on purpose.
    root.classList.toggle('railed', v === 0 || v === 127);
    root.classList.toggle('silent', v === null);
  }
}
