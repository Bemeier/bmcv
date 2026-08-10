// A DfuSe client over WebUSB, enough of it to write one image to internal
// flash and leave.
//
// DfuSe is ST's extension of DFU 1.1: plain DFU has no idea what an address is,
// so ST tunnels "set address pointer" and "erase page" as download blocks 0
// with a leading command byte, and starts real data at block 2. Everything
// below that is stock DFU.
//
// Written here rather than pulled in as a dependency because the whole protocol
// we need is about two hundred lines, and the alternative is vendoring a
// library to use a fifth of it.

const DFU_DNLOAD = 1;
const DFU_GETSTATUS = 3;
const DFU_CLRSTATUS = 4;
const DFU_ABORT = 6;

const STATE_DFU_IDLE = 2;
const STATE_DFU_DNBUSY = 4;
const STATE_DFU_ERROR = 10;

// DfuSe block-0 commands.
const CMD_SET_ADDRESS = 0x21;
const CMD_ERASE_PAGE = 0x41;

// Block number on the zero-length download that ends a session. Anything but 0
// would do - 0 is the one value that means "command" and gets the transfer
// discarded - and 2 is what dfu-util sends, so it is the best-tested choice.
const DFU_END_BLOCK = 2;

// ST's DFU interface: application-specific class, DFU subclass, DFU-mode
// protocol. Alt setting 0 is internal flash; 1 is option bytes and 2 is OTP,
// neither of which this touches.
const USB_CLASS_APP_SPECIFIC = 0xfe;
const USB_SUBCLASS_DFU = 0x01;
const ALT_INTERNAL_FLASH = 0;

// The erase granularity we ask for. A G474CE in its default dual-bank layout
// has 2 KB pages; if a part ever turns up configured for 4 KB, the bootloader
// erases the page containing each address we name, so the only cost of being
// wrong this way is erasing some pages twice.
export const FLASH_PAGE_SIZE = 2048;

export const FLASH_START = 0x08000000;

// The target's memory map, used to tell a firmware image from any other file.
const FLASH_SIZE = 512 * 1024;
const FLASH_END = FLASH_START + FLASH_SIZE;
const SRAM_START = 0x20000000;
const SRAM_END = 0x20020000; // 128 KB, and where the linker puts _estack

// Decide whether `bytes` is a raw image for this target, and say why not if it
// is not. Returns null when it looks fine.
//
// This exists because the obvious mistake is easy and expensive: an .elf sits
// right next to the .bin, is what the ST-Link workflow copies around, and is
// not a raw image at all - it is headers and symbols, so byte 0 is not the
// vector table and a debug build is four times the size of the flash. Writing
// one gets some way in and then fails on an address past the end of the chip,
// leaving the module erased and half-written.
//
// The vector table check is what makes this reliable rather than a guess: every
// raw Cortex-M image starts with an initial stack pointer and a reset handler,
// so an .elf, a .hex, a .dfu or a holiday photo all fail it.
export function describeImageProblem(bytes) {
  if (bytes.length < 8) {
    return 'That file is too small to be a firmware image.';
  }

  if (bytes[0] === 0x7f && bytes[1] === 0x45 && bytes[2] === 0x4c && bytes[3] === 0x46) {
    return 'That is an .elf, which is not a raw image. Flash the .bin the build writes beside it.';
  }

  if (bytes[0] === 0x3a) {
    return 'That looks like Intel .hex. Flash the raw .bin instead.';
  }

  if (bytes.length > FLASH_SIZE) {
    const kb = Math.round(bytes.length / 1024);
    return `That image is ${kb} KB and the flash is only ${FLASH_SIZE / 1024} KB.`;
  }

  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const stackPointer = view.getUint32(0, true);
  const resetHandler = view.getUint32(4, true);

  if (stackPointer < SRAM_START || stackPointer > SRAM_END) {
    return 'That file does not start with a vector table - its first word is not a stack pointer in RAM.';
  }

  if (resetHandler < FLASH_START || resetHandler >= FLASH_END) {
    return 'That file does not start with a vector table - its reset handler does not point into flash.';
  }

  return null;
}

// Where the ROM bootloader shows up. 0x0483 is ST's vendor ID and 0xdf11 its
// DFU product ID - the same pair every STM32 in DFU mode presents, which is
// also why Windows needs the WinUSB driver bound to it before Chrome can claim
// it.
const DFU_VENDOR_ID = 0x0483;
const DFU_PRODUCT_ID = 0xdf11;

export class DfuError extends Error {}

function statusName(status) {
  const names = {
    0: 'OK',
    1: 'target address not writable',
    2: 'file failed verification',
    3: 'write failed',
    4: 'erase failed',
    5: 'erase check failed',
    6: 'program failed',
    7: 'verify failed',
    8: 'address out of range',
    9: 'download ended early',
    10: 'firmware corrupt',
    11: 'vendor specific error',
    12: 'unexpected USB reset',
    13: 'power-on reset detected',
    14: 'unknown error',
    15: 'stalled',
  };
  return names[status] ?? `status ${status}`;
}

export class DfuDevice {
  constructor(device, interfaceNumber, transferSize) {
    this.device = device;
    this.interfaceNumber = interfaceNumber;
    this.transferSize = transferSize;
  }

  async controlOut(request, value, data) {
    const result = await this.device.controlTransferOut(
      {
        requestType: 'class',
        recipient: 'interface',
        request,
        value,
        index: this.interfaceNumber,
      },
      data,
    );
    if (result.status !== 'ok') {
      throw new DfuError(`control transfer failed: ${result.status}`);
    }
    return result;
  }

  async controlIn(request, value, length) {
    const result = await this.device.controlTransferIn(
      {
        requestType: 'class',
        recipient: 'interface',
        request,
        value,
        index: this.interfaceNumber,
      },
      length,
    );
    if (result.status !== 'ok') {
      throw new DfuError(`control transfer failed: ${result.status}`);
    }
    return new Uint8Array(result.data.buffer);
  }

  async getStatus() {
    const d = await this.controlIn(DFU_GETSTATUS, 0, 6);
    return {
      status: d[0],
      pollTimeout: d[1] | (d[2] << 8) | (d[3] << 16),
      state: d[4],
    };
  }

  async clearStatus() {
    await this.controlOut(DFU_CLRSTATUS, 0, new Uint8Array(0));
  }

  async abort() {
    await this.controlOut(DFU_ABORT, 0, new Uint8Array(0));
  }

  // Get back to dfuIDLE from wherever the last session left the device. A
  // bootloader that was interrupted mid-download stays in dfuDNLOAD_IDLE and
  // will reject the next set-address until it is cleared.
  //
  // Named for the DFU state machine, not the bus - it sends no USB reset.
  async toIdle() {
    let s = await this.getStatus();
    if (s.state === STATE_DFU_ERROR) {
      await this.clearStatus();
      s = await this.getStatus();
    }
    if (s.state !== STATE_DFU_IDLE) {
      await this.abort();
      s = await this.getStatus();
    }
    if (s.state !== STATE_DFU_IDLE) {
      throw new DfuError(`device will not return to idle (state ${s.state})`);
    }
  }

  // Poll until the device stops reporting itself busy. It tells us how long to
  // wait between polls, and page erases are the slow case.
  async waitWhileBusy() {
    let s = await this.getStatus();
    while (s.state === STATE_DFU_DNBUSY) {
      await new Promise((r) => setTimeout(r, s.pollTimeout + 1));
      s = await this.getStatus();
    }
    if (s.status !== 0) {
      throw new DfuError(statusName(s.status));
    }
    return s;
  }

  // DfuSe block-0 command: a download whose payload is a command byte and,
  // usually, a little-endian address. The GETSTATUS afterwards is what makes
  // the device actually run it.
  async command(bytes) {
    await this.controlOut(DFU_DNLOAD, 0, new Uint8Array(bytes));
    await this.waitWhileBusy();
  }

  async setAddress(addr) {
    await this.command([
      CMD_SET_ADDRESS,
      addr & 0xff,
      (addr >> 8) & 0xff,
      (addr >> 16) & 0xff,
      (addr >>> 24) & 0xff,
    ]);
  }

  async erasePage(addr) {
    await this.command([
      CMD_ERASE_PAGE,
      addr & 0xff,
      (addr >> 8) & 0xff,
      (addr >> 16) & 0xff,
      (addr >>> 24) & 0xff,
    ]);
  }

  // Erase exactly the pages the image will occupy. A mass erase would be one
  // command instead of ninety, but it gives no progress to report and wipes
  // flash this update has no business touching.
  async erase(startAddr, length, onProgress) {
    const pages = Math.ceil(length / FLASH_PAGE_SIZE);
    for (let i = 0; i < pages; i++) {
      await this.erasePage(startAddr + i * FLASH_PAGE_SIZE);
      onProgress?.((i + 1) / pages);
    }
  }

  async write(startAddr, image, onProgress) {
    // Before anything is sent, because this is the number every byte's address
    // is derived from. A zero would also make the loop below never advance:
    // slice(offset, offset + 0) is empty, offset never moves, and it would sit
    // there issuing downloads with rising block numbers for ever.
    if (!Number.isInteger(this.transferSize) || this.transferSize <= 0) {
      throw new DfuError(`the device declared a transfer size of ${this.transferSize}, which cannot be used`);
    }

    await this.setAddress(startAddr);

    // Data blocks are numbered from 2; the device derives each block's address
    // from the pointer we just set plus (block - 2) * transferSize.
    let offset = 0;
    let block = 2;
    while (offset < image.length) {
      const chunk = image.slice(offset, offset + this.transferSize);
      await this.controlOut(DFU_DNLOAD, block, chunk);
      await this.waitWhileBusy();
      offset += chunk.length;
      block++;
      onProgress?.(offset / image.length);
    }
  }

  // Close the download session. Does NOT restart the module.
  //
  // A DFU download is closed by a zero-length download, and it has to carry
  // block 2 rather than 0 - block 0 means "this is a command", and a command
  // with no payload is discarded. This is what dfu-util sends.
  //
  // On this part that puts the bootloader into its manifest phase, where it
  // drops its own USB pull-up and then stops, in the place ST's reference
  // implementation would have called NVIC_SystemReset(). It waits for a bus
  // reset that can no longer reach it. Nothing the host can send fixes that, so
  // the module needs a power cycle and the page says so - see the note in
  // index.html. The image is fully written by this point either way.
  async finish(startAddr) {
    await this.setAddress(startAddr);
    await this.controlOut(DFU_DNLOAD, DFU_END_BLOCK, new Uint8Array(0));
    try {
      await this.getStatus();
    } catch {
      // The bootloader stops answering on its way out. Expected.
    }
  }

  async close() {
    try {
      await this.device.close();
    } catch {
      // Already gone.
    }
  }
}

// Walk a raw configuration descriptor and return the wTransferSize the DFU
// functional descriptor declares for `interfaceNumber`, or null if there is
// none to find. Exported so it can be tested against a descriptor without a
// device - see dfu-check.mjs.
//
// This number is not a buffer size we are free to pick. DfuSe has the device
// derive each block's address as pointer + (block - 2) * wTransferSize, using
// its own value, so a wrong one here does not fail - it writes the image to
// the wrong addresses.
export function transferSizeFromDescriptor(bytes, interfaceNumber) {
  let seenOurInterface = false;
  for (let i = 0; i + 1 < bytes.length; ) {
    const len = bytes[i];
    const type = bytes[i + 1];
    if (len === 0) break;

    if (type === 0x04) {
      // Interface descriptor: bInterfaceNumber is at offset 2. The STM32 ROM
      // bootloader has three alternate settings on one interface, all sharing
      // a number, so this stays true across them.
      seenOurInterface = bytes[i + 2] === interfaceNumber;
    } else if (type === 0x21 && seenOurInterface && i + 6 < bytes.length) {
      // DFU functional descriptor: wTransferSize at offset 5.
      const size = bytes[i + 5] | (bytes[i + 6] << 8);
      return size > 0 ? size : null;
    }
    i += len;
  }
  return null;
}

// Pull the raw configuration descriptor and read wTransferSize out of the DFU
// functional descriptor. WebUSB exposes interfaces and endpoints but not
// class-specific descriptors, so this is the only way to ask.
async function readTransferSize(device, interfaceNumber) {
  const header = await device.controlTransferIn(
    { requestType: 'standard', recipient: 'device', request: 0x06, value: 0x0200, index: 0 },
    4,
  );
  const totalLength = header.data.getUint16(2, true);

  const full = await device.controlTransferIn(
    { requestType: 'standard', recipient: 'device', request: 0x06, value: 0x0200, index: 0 },
    totalLength,
  );
  const bytes = new Uint8Array(full.data.buffer);

  const size = transferSizeFromDescriptor(bytes, interfaceNumber);
  if (size === null) {
    // This used to fall back to 2048, on the grounds that every STM32 ROM
    // bootloader reports it. That is a guess at the number the device uses to
    // place the bytes, and guessing low is silent: the device spaces the blocks
    // further apart than we chunked them and the image lands full of holes.
    // Refusing costs a device that would otherwise have worked by luck, and
    // saves one that would have been quietly corrupted.
    throw new DfuError('That device did not say what transfer size it wants, so it cannot be flashed safely.');
  }
  return size;
}

// Ask the user to pick the device, then open and claim its DFU interface.
// Must be called from a user gesture - WebUSB will not show the picker
// otherwise.
export async function requestDfuDevice() {
  if (!navigator.usb) {
    throw new DfuError('This browser has no WebUSB. Use Chrome, Edge or another Chromium browser.');
  }

  const device = await navigator.usb.requestDevice({
    filters: [{ vendorId: DFU_VENDOR_ID, productId: DFU_PRODUCT_ID }],
  });

  await device.open();
  if (device.configuration === null) {
    await device.selectConfiguration(1);
  }

  let interfaceNumber = null;
  for (const iface of device.configuration.interfaces) {
    for (const alt of iface.alternates) {
      if (alt.interfaceClass === USB_CLASS_APP_SPECIFIC && alt.interfaceSubclass === USB_SUBCLASS_DFU) {
        interfaceNumber = iface.interfaceNumber;
        break;
      }
    }
    if (interfaceNumber !== null) break;
  }

  if (interfaceNumber === null) {
    await device.close();
    throw new DfuError('That device has no DFU interface.');
  }

  const transferSize = await readTransferSize(device, interfaceNumber);

  await device.claimInterface(interfaceNumber);
  await device.selectAlternateInterface(interfaceNumber, ALT_INTERNAL_FLASH);

  return new DfuDevice(device, interfaceNumber, transferSize);
}
