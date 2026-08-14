// Finding a BMCV on the MIDI bus.
//
// Shared by the link and by the throughput bench, which measure different
// things and must not disagree about which device they are measuring.

// Mirrors of Core/Inc/Lib/sysex.h.
export const SYSEX_ID = [0x7d, 0x42, 0x4d];
export const CMD_IDENTITY_REQ = 0x02;
export const CMD_REMOTE_INPUT = 0x20;
export const CMD_STREAM_REQ = 0x21;
export const CMD_SNAPSHOT = 0x22;

// How long one output gets to produce an answer before moving on. Two USB
// frames would do; this is loose enough for a busy machine and still leaves a
// bus with a dozen ports identified inside a second.
const IDENTIFY_MS = 150;

// Which pair of ports on the bus is a BMCV?
//
// Not by name. The module's USB *product* string is "BMCV", but a MIDI port's
// name is whatever the host driver decides to call it, and this device
// publishes no iJack strings at all - every iJack in the class descriptor is 0
// - so there is nothing for a driver to name the port after. Windows is
// particularly fond of calling such a device "USB Audio Device". Matching on
// the name is how this failed the first time it met real hardware.
//
// So ask instead. Each output gets an identity request and every input is
// listened to; the first output whose request produces our reply is the
// module's, and the input that answered is its other half. That is the same
// command the update page uses to read the firmware version, and it identifies
// the module positively rather than guessing from a label.
//
// Sequential rather than all at once, because the point is to learn *which*
// output the answer belongs to - firing at all of them together would identify
// the input and leave the output ambiguous on a bus with more than one device.
export async function identify(access, onStage) {
  const inputs = [...access.inputs.values()];
  const outputs = [...access.outputs.values()];

  if (!inputs.length || !outputs.length) {
    throw new Error('this browser sees no MIDI ports at all - is the module plugged in and powered?');
  }

  // Anything that does look like a BMCV is tried first, so the usual case
  // costs one round trip rather than a walk of the whole bus.
  const likely = p => /bmcv|bemeier/i.test(p.name ?? '');
  outputs.sort((a, b) => (likely(b) ? 1 : 0) - (likely(a) ? 1 : 0));

  for (const output of outputs) {
    onStage?.(`asking ${output.name}`);

    const answer = await new Promise(resolve => {
      const timer = setTimeout(() => resolve(null), IDENTIFY_MS);

      for (const input of inputs) {
        input.onmidimessage = ev => {
          const d = ev.data;
          if (d.length < 9 || d[0] !== 0xf0) return;
          if (d[1] !== SYSEX_ID[0] || d[2] !== SYSEX_ID[1] || d[3] !== SYSEX_ID[2]) return;
          if (d[4] !== CMD_IDENTITY_REQ) return;

          clearTimeout(timer);
          resolve({ input, version: `${d[5]}.${d[6]}.${d[7]}` });
        };
      }

      try {
        output.send([0xf0, ...SYSEX_ID, CMD_IDENTITY_REQ, 0xf7]);
      } catch {
        clearTimeout(timer);
        resolve(null); // a port that will not take a message is not the one
      }
    });

    for (const input of inputs) input.onmidimessage = null;
    if (answer) return { input: answer.input, output, version: answer.version };
  }

  const names = ps => ps.map(p => p.name || '(unnamed)').join(', ');
  throw new Error(
    `no BMCV answered. Outputs tried: ${names(outputs)}. Inputs listened to: ${names(inputs)}. `
    + 'If the module is plugged in, it is probably running firmware without the identity command.',
  );
}

