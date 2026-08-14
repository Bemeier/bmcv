// How fast can the module talk over its own USB port?
//
// A spike, not a feature. The probe bridge reads module state over SWD at about
// seventy snapshots a second; the open question is whether the same thing can be
// done over USB MIDI, which needs no probe, no WinUSB and no Chromium. Every
// other part of that design is arithmetic. This is the one number it hangs on,
// and there was no way to guess it: it depends on the browser's MIDI stack, the
// OS driver and the host controller, none of which are in this repo.
//
// See docs/plans/midi-transport.md, and SYSEX_CMD_BENCH_REQ in
// Core/Inc/Lib/sysex.h for the firmware end.

import { identify, SYSEX_ID, CMD_STREAM_REQ, CMD_SNAPSHOT } from '../probe/midiports.js';

const CMD_BENCH_REQ = 0x10;
const CMD_BENCH_DATA = 0x11;

// How long the snapshot measurement runs.
const SNAPSHOT_SECONDS = 8;

// Mirrors of Core/Inc/Lib/sysex.h.
const BENCH_MESSAGES = 2000;
const BENCH_MSG_BYTES = 48;
const BENCH_PACKET_BYTES = 64;

// What a snapshot would cost, so the raw rate can be read as something useful.
// The instance size is the ARM/wasm figure; the delta is measured - see the
// plan doc for the harness that produced it.
const INSTANCE_BYTES = 2368;
const DELTA_CHUNKS = 20;
const CHUNK_BYTES = 32 + 2; // payload plus its index

// SysEx is seven bits to the byte, so a payload costs 8/7 of itself on the way
// out and is worth 7/8 of itself on the way in.
const SEVEN_BIT = 8 / 7;

const el = id => document.getElementById(id);
const ui = {
  run: el('run'), status: el('status'), log: el('log'),
  results: el('results'), verdict: el('verdict'),
};

el('n-msgs').textContent = BENCH_MESSAGES;

function log(line) {
  ui.log.textContent += `${line}\n`;
}

function setStatus(text) {
  ui.status.textContent = text;
}

const fmt = (n, digits = 0) => n.toLocaleString(undefined, {
  minimumFractionDigits: digits, maximumFractionDigits: digits,
});

const kb = n => `${fmt(n / 1024, 1)} kB/s`;

/* ---- finding the module -------------------------------------------------- */

// By handshake, not by name - see midiports.js. A MIDI port is named by the
// host driver and this device gives it nothing to work with.
const findPorts = access => identify(access, stage => setStatus(stage));

/* ---- the snapshot rate, with nothing else in the way --------------------- */

// The bench above measures what the endpoint can carry. This measures what the
// real transport achieves, on a page that does no decoding and draws nothing.
//
// That gap is the whole point of running it here rather than on the simulator
// page: if snapshots arrive quickly here and slowly there, the transport is fine
// and the cost is in decoding and drawing them. If they are slow here too, it is
// the transport or the firmware. Nothing else distinguishes those.
function measureSnapshots(input, output) {
  return new Promise(resolve => {
    let count = 0, bytes = 0, first = 0, last = 0, short = 0;
    const gaps = [];

    const request = () => output.send([0xf0, ...SYSEX_ID, CMD_STREAM_REQ, 0xf7]);

    input.onmidimessage = ev => {
      const d = ev.data;
      if (d.length < 6 || d[0] !== 0xf0) return;
      if (d[1] !== SYSEX_ID[0] || d[2] !== SYSEX_ID[1] || d[3] !== SYSEX_ID[2]) return;
      if (d[4] !== CMD_SNAPSHOT) return;

      const now = performance.now();
      if (!count) first = now;
      else gaps.push(now - last);
      last = now;
      count++;
      bytes += d.length;
      if (d.length < 2000) short++;

      // One credit back, immediately. Nothing is decoded, so this is as fast as
      // a consumer can possibly ask - which makes it the ceiling for the paced
      // arrangement the simulator page uses.
      request();
    };

    // The same two the link opens with.
    request();
    request();

    setTimeout(() => {
      input.onmidimessage = null;
      resolve({ count, bytes, first, last, short, gaps });
    }, SNAPSHOT_SECONDS * 1000);
  });
}

function reportSnapshots(r) {
  const seconds = (r.last - r.first) / 1000;
  const rate = seconds > 0 ? (r.count - 1) / seconds : 0;

  ui.results.hidden = true;
  ui.verdict.hidden = false;

  if (r.count < 2) {
    ui.verdict.textContent = 'The module sent no snapshots. It is running firmware without the stream command, or the request is not reaching it.';
    return;
  }

  // The spread matters more than the average here. A steady stream and one that
  // arrives in bursts can average the same and look completely different.
  const sorted = [...r.gaps].sort((a, b) => a - b);
  const median = sorted[Math.floor(sorted.length / 2)];
  const worst = sorted[sorted.length - 1];

  ui.verdict.textContent =
    `${fmt(rate, 1)} snapshots/s over ${fmt(seconds, 1)}s, decoding nothing and drawing nothing. `
    + `Gap between them: ${fmt(median, 1)}ms typical, ${fmt(worst, 0)}ms worst. `
    + (worst > median * 5
      ? 'That spread means they are arriving in bursts, so something is stalling the stream rather than slowing it.'
      : 'Evenly spaced, so the transport itself is steady.');

  log(`${r.count} snapshots, ${fmt(r.bytes / 1024, 0)} kB, ${kb(r.bytes / seconds)}`);
  if (r.short) log(`${r.short} were shorter than a whole instance`);
}

/* ---- the run ------------------------------------------------------------- */

// Resolves once the burst has arrived, or once it has clearly stopped arriving.
//
// Timed from the *first* message rather than from the request, so what is
// measured is the transport moving data and not the round trip to start it -
// which on a first run includes the browser handing out a permission.
function collect(input, output) {
  return new Promise((resolve, reject) => {
    let first = 0, last = 0, count = 0, bad = 0;
    const seen = new Set();
    let idleTimer = null;

    const finish = () => {
      input.onmidimessage = null;
      clearTimeout(idleTimer);
      if (count === 0) return reject(new Error('the module answered nothing - is it running firmware with the bench command?'));
      resolve({ first, last, count, bad, unique: seen.size });
    };

    // A burst that stops early still gets measured, rather than hanging: the
    // module bounds its own run, so silence means it finished or something
    // dropped it.
    const armIdle = () => {
      clearTimeout(idleTimer);
      idleTimer = setTimeout(finish, 400);
    };

    input.onmidimessage = ev => {
      const d = ev.data;
      if (d.length !== BENCH_MSG_BYTES || d[0] !== 0xf0) return;
      if (d[1] !== SYSEX_ID[0] || d[2] !== SYSEX_ID[1] || d[3] !== SYSEX_ID[2]) return;
      if (d[4] !== CMD_BENCH_DATA) { bad++; return; }

      // performance.now(), not ev.timeStamp: the two agree in Chrome but the
      // spec leaves the origin to the implementation, and a wrong origin here
      // would show up as an absurd rate rather than as an error.
      const now = performance.now();
      if (count === 0) first = now;
      last = now;
      count++;
      seen.add(d[5] | (d[6] << 7));

      if (count >= BENCH_MESSAGES) return finish();
      armIdle();
    };

    armIdle();
    output.send([0xf0, ...SYSEX_ID, CMD_BENCH_REQ, 0xf7]);
  });
}

function report(r) {
  // One message is one transfer, which is what makes the rest of this honest -
  // tests/test_sysex.c holds the firmware to that.
  const seconds = (r.last - r.first) / 1000;

  // count - 1 intervals between count arrivals. At these counts it barely
  // matters, but dividing by count would quietly overstate a short run.
  const rate = seconds > 0 ? (r.count - 1) / seconds : 0;
  const payload = rate * BENCH_MSG_BYTES;
  const wire = rate * BENCH_PACKET_BYTES;
  const useful = payload / SEVEN_BIT;

  el('r-msgs').textContent = fmt(r.count);
  el('r-lost').textContent = r.unique === r.count
    ? 'none lost'
    : `${fmt(r.count - r.unique)} duplicated or reordered`;
  el('r-time').textContent = `${fmt(seconds, 2)} s`;
  el('r-rate').textContent = fmt(rate);
  el('r-payload').textContent = kb(payload);
  el('r-wire').textContent = kb(wire);
  el('r-useful').textContent = kb(useful);

  const full = useful / INSTANCE_BYTES;
  const delta = useful / (DELTA_CHUNKS * CHUNK_BYTES);
  el('r-full').textContent = fmt(full, 1);
  el('r-delta').textContent = fmt(delta, 1);

  ui.results.hidden = false;

  // The probe manages about seventy. That is the bar, and saying so here beats
  // reading a rate and having to remember what it should be compared with.
  const verdict = delta >= 50
    ? `Comfortable. Chunked deltas would run at ${fmt(delta, 0)}/s against the probe's ~70, and even whole snapshots at ${fmt(full, 1)}/s are usable.`
    : delta >= 20
      ? `Workable. Chunked deltas at ${fmt(delta, 0)}/s is below the probe's ~70 but well above the rate a panel needs to look alive. Whole snapshots at ${fmt(full, 1)}/s would not be.`
      : delta >= 5
        ? `Marginal. ${fmt(delta, 0)} chunked updates a second would show a module moving, but not smoothly, and only a delta scheme gets there at all.`
        : `Too slow to be worth building. ${fmt(delta, 1)} updates a second is a readout, not a display.`;

  ui.verdict.textContent = verdict;
  ui.verdict.hidden = false;

  log(`${r.count} messages in ${fmt(seconds, 3)}s`);
  if (r.bad) log(`${r.bad} messages had the wrong command byte`);
  if (r.count < BENCH_MESSAGES) log(`asked for ${BENCH_MESSAGES}, so ${BENCH_MESSAGES - r.count} did not arrive`);
}

/* ---- wiring -------------------------------------------------------------- */

if (!navigator.requestMIDIAccess) {
  setStatus('this browser has no Web MIDI');
} else {
  ui.run.disabled = false;
  setStatus('ready');
}

// Both buttons share the connect-and-find preamble.
async function withModule(fn) {
  setStatus('asking for MIDI access');
  const access = await navigator.requestMIDIAccess({ sysex: true });
  const { input, output, version } = await findPorts(access);
  log(`in:  ${input.name}`);
  log(`out: ${output.name}`);
  log(`firmware ${version}`);
  return fn(input, output);
}

el('run-snapshots')?.addEventListener('click', async () => {
  const button = el('run-snapshots');
  button.disabled = true;
  ui.verdict.hidden = true;
  ui.results.hidden = true;
  ui.log.textContent = '';

  try {
    await withModule(async (input, output) => {
      setStatus(`streaming for ${SNAPSHOT_SECONDS}s`);
      reportSnapshots(await measureSnapshots(input, output));
      setStatus('done');
    });
  } catch (e) {
    setStatus(e.message);
  } finally {
    button.disabled = false;
  }
});

ui.run.addEventListener('click', async () => {
  ui.run.disabled = true;
  ui.verdict.hidden = true;
  ui.results.hidden = true;
  ui.log.textContent = '';

  try {
    setStatus('asking for MIDI access');
    const access = await navigator.requestMIDIAccess({ sysex: true });

    const { input, output } = await findPorts(access);
    log(`in:  ${input.name}`);
    log(`out: ${output.name}`);

    setStatus('running');
    const result = await collect(input, output);
    setStatus('done');
    report(result);
  } catch (e) {
    setStatus(e.message);
  } finally {
    ui.run.disabled = false;
  }
});
