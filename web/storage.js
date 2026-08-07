// Mirror the module's preset store into localStorage.
//
// The module autosaves its config to its last FRAM slot every couple of
// seconds. Copying that blob out on the same cadence gives the simulator the
// property the hardware has: what you set up is still there next time.
//
// The blob is the simulator's own slot array, not a FRAM chip dump, so it is
// not interchangeable with one. A blob of the wrong length is rejected here;
// anything that gets past that is still run through config_validate() when the
// module boots.

import { sim } from './sim.js';

const KEY = 'bmcv.fram.v1';

// 6KB is past the point where String.fromCharCode(...bytes) is safe, so chunk.
function toB64(bytes) {
  let out = '';
  for (let i = 0; i < bytes.length; i += 0x8000) {
    out += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000));
  }
  return btoa(out);
}

function fromB64(str) {
  const bin = atob(str);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

let lastPersisted = null;

// Load whatever is saved and reboot the module so it picks it up. Returns true
// if a saved patch was actually restored.
export function restore() {
  try {
    const saved = localStorage.getItem(KEY);
    if (!saved) return false;

    const bytes = fromB64(saved);
    // A blob from a build with a different EngineConfig layout will not be
    // this length.
    if (bytes.length !== sim.storageSize || !sim.writeStorage(bytes)) return false;

    sim.reset(false);        // reboot with the storage in place, so it loads
    lastPersisted = saved;
    return true;
  } catch (e) {
    console.warn('BMCV: could not restore saved state', e);
    return false;
  }
}

// A no-op when nothing has changed, so this is cheap to call on a timer.
export function persist() {
  try {
    const b64 = toB64(sim.readStorage());
    if (b64 === lastPersisted) return;
    localStorage.setItem(KEY, b64);
    lastPersisted = b64;
  } catch (e) {
    // Private mode or a full quota: keep running, just without persistence.
  }
}

export function forget() {
  try { localStorage.removeItem(KEY); } catch (e) { /* ignore */ }
  lastPersisted = null;
}
