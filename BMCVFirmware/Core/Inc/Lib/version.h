#ifndef INC_LIB_VERSION_H_
#define INC_LIB_VERSION_H_

// The running firmware's version, reported over SysEx so the update page can
// show what is on the module next to the image it is about to write.
//
// Nothing enforces it. Any build flashes over any other - the ROM DFU
// bootloader has no idea what a version is, and adding a policy here would only
// be a policy the recovery path ignores.
//
// Each field is sent as a single SysEx data byte, so keep them under 128.
#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 0

#endif /* INC_LIB_VERSION_H_ */
