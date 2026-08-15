// What the module's vendor interface is, on the wire.
//
// Numbers and two one-line exchanges, mirroring Core/Inc/Lib/usblink.h and
// USB_Device/App/usbd_webusb.h. Nothing here holds a session, decodes a
// snapshot or knows what a BmcvInstance is.
//
// Split out of usblink.js, and the split is not tidiness. Two pages talk to a
// module without simulating one: the firmware updater, and the diagnostics
// page. Both used to reach into usblink.js for a version string and a reboot,
// which imports sim.js, which instantiates bmcv.wasm at module scope - so both
// pages loaded a wasm they had no use for, and, since bmcv.js and bmcv.wasm are
// build output rather than checked-in files, neither page would run at all on a
// clean tree. The updater is the page that recovers a module whose firmware is
// broken; it must not depend on a simulator build to load.
//
// So: anything a page can want without the wasm lives here, and anything that
// needs the wasm lives in usblink.js. Keep this file free of imports.

// WebUSB is Chromium's alone, and a page that simply has no button on Firefox
// is a bug report waiting to happen. The UI asks this so it can say why.
//
// Both links used to export their own copy of this, which is one line each and
// two places for it to drift.
export const webusbAvailable = typeof navigator !== 'undefined' && !!navigator.usb;

export const BMCV_VID = 0x0483;
export const BMCV_PID = 0x572b;

// The interface a browser claims, and the pair of bulk endpoints on it. MIDI is
// interface 0 and keeps the class driver it already has.
export const VENDOR_INTERFACE = 1;
export const EP_IN = 2;
export const EP_OUT = 2;

// One transfer, first byte first. Mirrors UsbLinkOp.
export const OP_SNAPSHOT_REQ = 0x01;
export const OP_REMOTE_INPUT = 0x02;
export const OP_REMOTE_COMMAND = 0x03;
export const OP_ENTER_DFU = 0x04;

// Mirrors BMCV_REQ_VERSION in USB_Device/App/usbd_webusb.h.
const REQ_VERSION = 0x43;

// Ask a module what firmware it is running.
//
// A control request, so it works on a device that has only been opened and
// needs nothing claimed or streaming - which is what the update page wants, and
// what a diagnostic wants before deciding anything else is worth trying.
export async function readVersion(device) {
  const r = await device.controlTransferIn({
    requestType: 'vendor',
    recipient: 'interface',
    request: REQ_VERSION,
    value: 0,
    index: VENDOR_INTERFACE,
  }, 32);

  if (r.status !== 'ok' || !r.data?.byteLength) return null;
  return new TextDecoder().decode(r.data).replace(/\0.*$/, '');
}

// Hand a module to its ROM DFU bootloader.
//
// Takes a device rather than belonging to a link, because the page that wants
// it owns no link: the updater opens the module itself, tells it to reboot, and
// then talks to something else entirely. Keeping the opcode here is what stops
// it being written out again over there.
export function enterDfuOn(device) {
  return device.transferOut(EP_OUT, new Uint8Array([OP_ENTER_DFU]));
}
