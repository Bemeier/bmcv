// Headless check of the DfuSe client against a fake WebUSB device.
//
// This is the part of the updater that cannot be tried out casually: getting it
// wrong means a half-written module, and the only way to find out is to flash
// real hardware. So the fake device below records every control transfer and
// the checks assert the exact sequence ST's bootloader expects - block 0
// commands with the right opcodes and addresses, data blocks numbered from 2,
// a status poll after each one.
//
// Run with `just dfu-check`.

import { readFileSync, existsSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

import { DfuDevice, describeImageProblem, transferSizeFromDescriptor, FLASH_PAGE_SIZE, FLASH_START } from './dfuse.js';

const repoRoot = join(dirname(fileURLToPath(import.meta.url)), '..', '..');

let failures = 0;
const check = (ok, what) => {
  if (!ok) {
    failures++;
    console.error(`FAIL ${what}`);
  } else {
    console.log(`ok   ${what}`);
  }
};

// A device that answers every status request with "idle, no error" and writes
// down what it was asked to do.
function fakeDevice() {
  const calls = [];
  return {
    calls,
    async controlTransferOut(setup, data) {
      calls.push({ dir: 'out', request: setup.request, value: setup.value, data: data ? [...new Uint8Array(data)] : [] });
      return { status: 'ok' };
    },
    async controlTransferIn(setup, length) {
      calls.push({ dir: 'in', request: setup.request, value: setup.value, length });
      // GETSTATUS: bStatus=OK, poll timeout 0, state dfuDNLOAD_IDLE.
      const buf = new Uint8Array([0, 0, 0, 0, 5, 0]);
      return { status: 'ok', data: new DataView(buf.buffer) };
    },
    async close() {},
  };
}

const DNLOAD = 1;
const GETSTATUS = 3;

const downloads = (calls) => calls.filter((c) => c.dir === 'out' && c.request === DNLOAD);

/* ---- erase --------------------------------------------------------------- */

{
  const dev = fakeDevice();
  const dfu = new DfuDevice(dev, 0, 2048);

  // Just over two pages, so the page count has to round up rather than down.
  await dfu.erase(FLASH_START, FLASH_PAGE_SIZE * 2 + 1);

  const cmds = downloads(dev.calls);
  check(cmds.length === 3, `erase of 2 pages + 1 byte issues 3 page erases (got ${cmds.length})`);
  check(
    cmds.every((c) => c.value === 0 && c.data[0] === 0x41),
    'every erase is a block 0 command with opcode 0x41',
  );

  const addresses = cmds.map((c) => c.data[1] | (c.data[2] << 8) | (c.data[3] << 16) | (c.data[4] << 24));
  check(
    addresses[0] === FLASH_START &&
      addresses[1] === FLASH_START + FLASH_PAGE_SIZE &&
      addresses[2] === FLASH_START + 2 * FLASH_PAGE_SIZE,
    'erase addresses step one page at a time from the start of flash',
  );

  const statusPolls = dev.calls.filter((c) => c.dir === 'in' && c.request === GETSTATUS);
  check(statusPolls.length >= cmds.length, 'each erase is followed by a status poll');
}

/* ---- write --------------------------------------------------------------- */

{
  const dev = fakeDevice();
  const transferSize = 2048;
  const dfu = new DfuDevice(dev, 0, transferSize);

  // Deliberately not a multiple of the transfer size: the last chunk is short,
  // and truncating or padding it would both be wrong.
  const image = new Uint8Array(transferSize * 2 + 100);
  image.fill(0xab);

  await dfu.write(FLASH_START, image);

  const cmds = downloads(dev.calls);
  const setAddress = cmds[0];
  check(
    setAddress.value === 0 && setAddress.data[0] === 0x21,
    'write begins by setting the address pointer (block 0, opcode 0x21)',
  );

  const dataBlocks = cmds.slice(1);
  check(dataBlocks.length === 3, `a 2.05-transfer image writes 3 blocks (got ${dataBlocks.length})`);
  check(
    dataBlocks.map((c) => c.value).join(',') === '2,3,4',
    'data blocks are numbered from 2 upwards',
  );
  check(
    dataBlocks[0].data.length === transferSize && dataBlocks[1].data.length === transferSize,
    'full blocks carry a whole transfer each',
  );
  check(dataBlocks[2].data.length === 100, 'the final short block is sent at its real length, not padded');

  const total = dataBlocks.reduce((n, c) => n + c.data.length, 0);
  check(total === image.length, 'every byte of the image is sent exactly once');
}

/* ---- finish -------------------------------------------------------------- */

{
  const dev = fakeDevice();
  const dfu = new DfuDevice(dev, 0, 2048);

  await dfu.finish(FLASH_START);

  const cmds = downloads(dev.calls);
  check(cmds[0].data[0] === 0x21, 'finish sets the address pointer to the image start');
  check(cmds[1].data.length === 0, 'finish then sends a zero-length download');

  // Block 0 means "this is a command", and a command with no payload is
  // discarded - so the download never closes and the bootloader never reaches
  // its manifest phase at all.
  check(cmds[1].value !== 0, 'the zero-length download is NOT block 0, which reads as an empty command');
  check(cmds[1].value === 2, 'it uses block 2, the value dfu-util sends');
}

/* ---- a device that stops answering mid-finish ---------------------------- */

{
  // The bootloader drops off the bus partway through the final status request.
  // That is normal on the way out, and it must not surface as a failed flash.
  const dev = fakeDevice();
  let sawFinalStatus = false;
  const inner = dev.controlTransferIn.bind(dev);
  dev.controlTransferIn = async (setup, length) => {
    if (sawFinalStatus) throw new Error('device disconnected');
    sawFinalStatus = true;
    return inner(setup, length);
  };

  const dfu = new DfuDevice(dev, 0, 2048);
  let threw = false;
  try {
    await dfu.finish(FLASH_START);
  } catch {
    threw = true;
  }
  check(!threw, 'a device that stops answering while finishing is not an error');
}

/* ---- what counts as a firmware image ------------------------------------- */

// The .elf is the mistake worth guarding hardest against: it sits next to the
// .bin, it is what the ST-Link workflow copies to the desktop, and writing one
// erases the module before failing partway through on an address past the end
// of the flash.
{
  const vectorTable = (sp, pc) => {
    const b = new Uint8Array(256);
    new DataView(b.buffer).setUint32(0, sp, true);
    new DataView(b.buffer).setUint32(4, pc, true);
    return b;
  };

  check(describeImageProblem(vectorTable(0x20020000, 0x08000201)) === null, 'a plausible raw image is accepted');

  const elf = new Uint8Array(1024);
  elf.set([0x7f, 0x45, 0x4c, 0x46]);
  const elfProblem = describeImageProblem(elf);
  check(elfProblem !== null && /\.elf/.test(elfProblem), 'an .elf is rejected, and says so by name');

  const hex = new TextEncoder().encode(':020000040800F2\n');
  check(describeImageProblem(hex) !== null, 'Intel .hex is rejected');

  check(describeImageProblem(new Uint8Array(4)) !== null, 'a file too short to hold a vector table is rejected');

  const huge = vectorTable(0x20020000, 0x08000201);
  const oversize = new Uint8Array(600 * 1024);
  oversize.set(huge);
  check(describeImageProblem(oversize) !== null, 'an image larger than the flash is rejected');

  check(
    describeImageProblem(vectorTable(0x08001234, 0x08000201)) !== null,
    'a first word that is not a RAM stack pointer is rejected',
  );
  check(
    describeImageProblem(vectorTable(0x20020000, 0x20001234)) !== null,
    'a reset handler that does not point into flash is rejected',
  );
}

// And against the real artefacts, when a build has been done. This is the check
// that would have caught the actual mistake, on the actual files.
{
  const bin = join(repoRoot, 'build-rel', 'BMCVFirmware.bin');
  const elf = join(repoRoot, 'build-rel', 'BMCVFirmware.elf');

  if (existsSync(bin)) {
    check(describeImageProblem(new Uint8Array(readFileSync(bin))) === null, 'the real built .bin is accepted');
  } else {
    console.log('skip the real .bin (no build-rel yet)');
  }

  if (existsSync(elf)) {
    check(describeImageProblem(new Uint8Array(readFileSync(elf))) !== null, 'the real built .elf is rejected');
  } else {
    console.log('skip the real .elf (no build-rel yet)');
  }
}

/* ---- the transfer size, which decides where every byte lands -------------- */

// DfuSe has the device compute each block's address as
// pointer + (block - 2) * wTransferSize, from its own declared value. Reading
// this wrong does not fail the flash - it scatters the image.
{
  // A configuration descriptor shaped like the STM32 ROM bootloader's: one
  // interface carrying three alternate settings (internal flash, option bytes,
  // OTP) that all share a bInterfaceNumber, with the DFU functional descriptor
  // after the last of them.
  const iface = (num, alt) => [9, 0x04, num, alt, 0, 0xfe, 0x01, 0x02, 0];
  const dfuFunctional = (size) => [9, 0x21, 0x0b, 0xff, 0x00, size & 0xff, (size >> 8) & 0xff, 0x1a, 0x01];
  const config = (size) =>
    new Uint8Array([9, 0x02, 0, 0, 1, 1, 0, 0xc0, 50, ...iface(0, 0), ...iface(0, 1), ...iface(0, 2), ...dfuFunctional(size)]);

  check(transferSizeFromDescriptor(config(2048), 0) === 2048, 'the declared transfer size is read from the DFU functional descriptor');
  check(transferSizeFromDescriptor(config(1024), 0) === 1024, 'a device declaring something other than 2048 is believed');
  check(transferSizeFromDescriptor(config(2048), 3) === null, 'a functional descriptor on another interface is not used');
  check(transferSizeFromDescriptor(config(0), 0) === null, 'a declared size of zero is rejected rather than returned');
  check(transferSizeFromDescriptor(new Uint8Array([9, 0x02, 0, 0, 1, 1, 0, 0xc0, 50]), 0) === null, 'a descriptor with no DFU functional block yields nothing');

  // A zero length would step the walk nowhere; it must terminate anyway.
  check(transferSizeFromDescriptor(new Uint8Array([0, 0x02, 0, 0]), 0) === null, 'a zero-length descriptor does not hang the walk');
}

// The same number, one layer up: write() must refuse rather than loop. With a
// transfer size of zero every chunk is empty, so the offset never advances and
// the loop issues downloads with rising block numbers indefinitely.
{
  let downloads = 0;
  const dev = {
    async controlTransferOut() {
      if (++downloads > 1000) throw new Error('runaway write loop');
      return { status: 'ok' };
    },
    async controlTransferIn() {
      return { status: 'ok', data: new DataView(new Uint8Array([0, 0, 0, 0, 5, 0]).buffer) };
    },
  };

  let message = null;
  try {
    await new DfuDevice(dev, 0, 0).write(FLASH_START, new Uint8Array(4096));
  } catch (e) {
    message = e.message;
  }
  check(message !== null && !message.includes('runaway'), 'a transfer size of zero is refused instead of looping');
  check(downloads === 0, 'and nothing is sent to the device before it is refused');
}

/* ---- returning to idle --------------------------------------------------- */

// The recovery path after an interrupted flash: the bootloader is left in
// dfuDNLOAD_IDLE and rejects the next set-address until it is cleared. This is
// what makes "the module is still in update mode and can be flashed again"
// true rather than hopeful.
{
  const states = [5, 2]; // dfuDNLOAD_IDLE, then dfuIDLE after the abort
  const requests = [];
  const dev = {
    async controlTransferOut(setup) {
      requests.push(setup.request);
      return { status: 'ok' };
    },
    async controlTransferIn() {
      const state = states.shift() ?? 2;
      return { status: 'ok', data: new DataView(new Uint8Array([0, 0, 0, 0, state, 0]).buffer) };
    },
  };

  await new DfuDevice(dev, 0, 2048).toIdle();
  check(requests.includes(6), 'an interrupted download is cleared with DFU_ABORT');
}

{
  // A device stuck in dfuERROR needs CLRSTATUS, not an abort.
  const states = [10, 2];
  const requests = [];
  const dev = {
    async controlTransferOut(setup) {
      requests.push(setup.request);
      return { status: 'ok' };
    },
    async controlTransferIn() {
      const state = states.shift() ?? 2;
      return { status: 'ok', data: new DataView(new Uint8Array([0, 0, 0, 0, state, 0]).buffer) };
    },
  };

  await new DfuDevice(dev, 0, 2048).toIdle();
  check(requests.includes(4), 'a device in dfuERROR is cleared with DFU_CLRSTATUS');
}

{
  // A device that will not come back is an error rather than a flash attempt.
  const dev = {
    async controlTransferOut() {
      return { status: 'ok' };
    },
    async controlTransferIn() {
      return { status: 'ok', data: new DataView(new Uint8Array([0, 0, 0, 0, 9, 0]).buffer) };
    },
  };

  let threw = false;
  try {
    await new DfuDevice(dev, 0, 2048).toIdle();
  } catch {
    threw = true;
  }
  check(threw, 'a device that will not return to idle is refused, not flashed anyway');
}

/* ---- a device that reports a failure mid-write ---------------------------- */

{
  const dev = {
    async controlTransferOut() {
      return { status: 'ok' };
    },
    async controlTransferIn() {
      // bStatus = 3 (write failed), state dfuIDLE.
      return { status: 'ok', data: new DataView(new Uint8Array([3, 0, 0, 0, 2, 0]).buffer) };
    },
  };

  let message = null;
  try {
    await new DfuDevice(dev, 0, 2048).write(FLASH_START, new Uint8Array(64));
  } catch (e) {
    message = e.message;
  }
  check(message === 'write failed', `a device-reported write failure stops the flash (got ${message})`);
}

console.log(failures ? `\n${failures} check(s) failed` : '\nall checks passed');
process.exit(failures ? 1 : 0);
