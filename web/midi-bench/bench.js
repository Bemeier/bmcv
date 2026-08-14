// Two instruments for the MIDI link, kept because they earned it.
//
// Every time a fault here was reasoned about it was diagnosed wrongly, and every
// time one of these was pointed at it the answer arrived in a single run. The
// snapshot rate separated a slow transport from a stalling one; the bus monitor
// separated a dead endpoint from a lost reply. Both distinctions were invisible
// from the simulator page, and both had been guessed at for several rounds.
//
// See docs/midi-link.md.

import { identify, SYSEX_ID, CMD_IDENTITY_REQ, CMD_STREAM_REQ, CMD_SNAPSHOT } from '../probe/midiports.js';

// How long the snapshot measurement runs.
const SNAPSHOT_SECONDS = 8;

const el = id => document.getElementById(id);
const ui = {
  status: el('status'), log: el('log'), verdict: el('verdict'),
};

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

/* ---- what is actually on the bus ----------------------------------------- */

// No theory, just bytes.
//
// Four attempts to find why a module that is plugged in, named, and running
// answers nothing have each been a plausible guess, and each has been wrong in
// a different way. This asks the bus instead: open every port, say what state
// it is in, send the identity request to each output in turn, and print every
// byte that arrives on any input.
//
// If the module answers, this shows what it said. If it does not, this shows
// whether the request could even be sent - which is the fork the error message
// alone cannot resolve.
async function diagnose(access) {
  const inputs = [...access.inputs.values()];
  const outputs = [...access.outputs.values()];

  const hex = d => [...d].map(b => b.toString(16).padStart(2, '0')).join(' ');

  log(`inputs:  ${inputs.length}`);
  for (const p of inputs) log(`  "${p.name}" [${p.manufacturer || 'no manufacturer'}] state=${p.state} connection=${p.connection}`);
  log(`outputs: ${outputs.length}`);
  for (const p of outputs) log(`  "${p.name}" [${p.manufacturer || 'no manufacturer'}] state=${p.state} connection=${p.connection}`);

  log('');
  log('opening every port...');
  for (const p of [...inputs, ...outputs]) {
    try {
      await p.open();
      log(`  ${p.type} "${p.name}" -> ${p.connection}`);
    } catch (e) {
      log(`  ${p.type} "${p.name}" -> FAILED: ${e.name} ${e.message}`);
    }
  }

  // Listen before asking anything.
  //
  // The module publishes its channel outputs as control changes continuously,
  // whether or not anybody has spoken to it. So this separates the two failures
  // that look identical from outside: control changes arriving means the IN
  // endpoint works and only the reply is broken; total silence means the
  // endpoint is producing nothing at all, and no amount of SysEx will help.
  //
  // The previous run of this tool already said which it was - one message
  // heard, and that one was loopMIDI's own echo - and it was misread.
  const passive = new Map(inputs.map(p => [p.name, 0]));
  const sample = new Map();

  for (const input of inputs) {
    input.onmidimessage = ev => {
      passive.set(input.name, passive.get(input.name) + 1);
      if (!sample.has(input.name)) sample.set(input.name, hex(ev.data.subarray(0, 8)));
    };
  }

  log('');
  log('listening for 3s, saying nothing...');
  await new Promise(r => setTimeout(r, 3000));

  for (const [name, n] of passive) {
    log(`  "${name}": ${n} messages${n ? ` (first: ${sample.get(name)})` : ''}`);
  }
  if ((passive.get('BMCV') ?? 0) === 0) {
    log('  BMCV sent nothing unprompted. It publishes its channel outputs as');
    log('  control changes continuously, so silence here means the IN endpoint');
    log('  is dead rather than the SysEx reply being lost.');
  }

  let heard = 0;
  for (const input of inputs) {
    input.onmidimessage = ev => {
      heard++;
      const d = ev.data;
      const tag = d.length > 24 ? `${hex(d.subarray(0, 12))} ... (${d.length} bytes)` : hex(d);
      log(`  <- "${input.name}": ${tag}`);
    };
  }

  for (const output of outputs) {
    log('');
    log(`asking "${output.name}" who it is...`);
    const before = heard;
    try {
      output.send([0xf0, ...SYSEX_ID, CMD_IDENTITY_REQ, 0xf7]);
      log('  sent F0 7D 42 4D 02 F7');
    } catch (e) {
      log(`  send FAILED: ${e.name} ${e.message}`);
      continue;
    }
    await new Promise(r => setTimeout(r, 700));
    if (heard === before) log('  (nothing came back)');
  }

  log('');
  log(`total messages heard: ${heard}`);
  if (!heard) {
    log('Nothing at all arrived. Either the module is not running this firmware,');
    log('or its USB IN endpoint is holding a transfer nobody collected - which a');
    log('power cycle clears, and which the current firmware recovers from itself.');
  }

  for (const input of inputs) input.onmidimessage = null;
}

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

/* ---- wiring -------------------------------------------------------------- */

if (!navigator.requestMIDIAccess) {
  setStatus('this browser has no Web MIDI');
  for (const id of ['run-snapshots', 'run-diagnose']) el(id).disabled = true;
} else {
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

el('run-diagnose')?.addEventListener('click', async () => {
  const button = el('run-diagnose');
  button.disabled = true;
  ui.verdict.hidden = true;
  ui.log.textContent = '';

  try {
    setStatus('asking for MIDI access');
    const access = await navigator.requestMIDIAccess({ sysex: true });
    setStatus('probing');
    await diagnose(access);
    setStatus('done - copy the log');
  } catch (e) {
    setStatus(`${e.name}: ${e.message}`);
  } finally {
    button.disabled = false;
  }
});

el('run-snapshots')?.addEventListener('click', async () => {
  const button = el('run-snapshots');
  button.disabled = true;
  ui.verdict.hidden = true;
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

