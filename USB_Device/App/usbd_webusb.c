#include "usbd_webusb.h"

// One UTF-16LE code unit of a plain ASCII character. The registry property
// below carries two strings in UTF-16, and writing forty of these out as
// `'D', 0x00,` pairs is where a transcription error would hide.
#define U16(c) (uint8_t)(c), 0x00

/* ---- BOS ----------------------------------------------------------------- */

// Both capability UUIDs are written the way the specifications print them,
// which is little-endian for the first three fields and big-endian for the
// last two. Getting that backwards produces a device the host enumerates and
// then ignores, with nothing said about why.

// clang-format off
static const uint8_t bos[BMCV_BOS_TOTAL_LEN] = {
    /* ---- BOS descriptor header, 5 bytes ---- */
    0x05,                                              /* bLength */
    0x0F,                                              /* bDescriptorType: BOS */
    (uint8_t) (BMCV_BOS_TOTAL_LEN & 0xFF),             /* wTotalLength */
    (uint8_t) (BMCV_BOS_TOTAL_LEN >> 8),
    0x02,                                              /* bNumDeviceCaps */

    /* ---- WebUSB platform capability, 24 bytes ---- */
    0x18,                                              /* bLength */
    0x10,                                              /* bDescriptorType: DEVICE_CAPABILITY */
    0x05,                                              /* bDevCapabilityType: PLATFORM */
    0x00,                                              /* bReserved */
    /* UUID 3408b638-09a9-47a0-8bfd-a0768815b665 */
    0x38, 0xB6, 0x08, 0x34,
    0xA9, 0x09,
    0xA0, 0x47,
    0x8B, 0xFD,
    0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
    0x00, 0x01,                                        /* bcdVersion 1.00 */
    BMCV_WEBUSB_VENDOR_CODE,                           /* bVendorCode */
    0x00,                                              /* iLandingPage: none. A URL here
                                                          makes browsers offer to open a
                                                          site when the module is plugged
                                                          in, which this does not want. */

    /* ---- Microsoft OS 2.0 platform capability, 28 bytes ---- */
    0x1C,                                              /* bLength */
    0x10,                                              /* bDescriptorType: DEVICE_CAPABILITY */
    0x05,                                              /* bDevCapabilityType: PLATFORM */
    0x00,                                              /* bReserved */
    /* UUID d8dd60df-4589-4cc7-9cd2-659d9e648a9f */
    0xDF, 0x60, 0xDD, 0xD8,
    0x89, 0x45,
    0xC7, 0x4C,
    0x9C, 0xD2,
    0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06,                            /* dwWindowsVersion: 8.1, the first
                                                          release that reads these */
    (uint8_t) (BMCV_MSOS20_TOTAL_LEN & 0xFF),          /* wMSOSDescriptorSetTotalLength */
    (uint8_t) (BMCV_MSOS20_TOTAL_LEN >> 8),
    BMCV_MSOS20_VENDOR_CODE,                           /* bMS_VendorCode */
    0x00,                                              /* bAltEnumCode: no alternate
                                                          enumeration */
};

/* ---- Microsoft OS 2.0 descriptor set -------------------------------------- */

// Four nested lengths that all have to agree: the set's total, the
// configuration subset's, the function subset's, and each feature's own. Three
// of them are sums of what follows, so a single added byte moves all of them -
// which is what the test exists to catch.

#define MSOS20_PROPERTY_LEN 132
#define MSOS20_FUNCTION_LEN (8 + 20 + MSOS20_PROPERTY_LEN)
#define MSOS20_CONFIG_LEN (8 + MSOS20_FUNCTION_LEN)

static const uint8_t msos20[BMCV_MSOS20_TOTAL_LEN] = {
    /* ---- set header, 10 bytes ---- */
    0x0A, 0x00,                                        /* wLength */
    0x00, 0x00,                                        /* wDescriptorType: SET_HEADER */
    0x00, 0x00, 0x03, 0x06,                            /* dwWindowsVersion: 8.1 */
    (uint8_t) (BMCV_MSOS20_TOTAL_LEN & 0xFF),          /* wTotalLength */
    (uint8_t) (BMCV_MSOS20_TOTAL_LEN >> 8),

    /* ---- configuration subset, 8 bytes of header ---- */
    0x08, 0x00,                                        /* wLength */
    0x01, 0x00,                                        /* wDescriptorType: SUBSET_CONFIGURATION */
    0x00,                                              /* bConfigurationValue: the first
                                                          configuration, counted from 0 -
                                                          not the bConfigurationValue of 1
                                                          that the config descriptor
                                                          carries */
    0x00,                                              /* bReserved */
    (uint8_t) (MSOS20_CONFIG_LEN & 0xFF),              /* wTotalLength */
    (uint8_t) (MSOS20_CONFIG_LEN >> 8),

    /* ---- function subset, 8 bytes of header ---- */
    0x08, 0x00,                                        /* wLength */
    0x02, 0x00,                                        /* wDescriptorType: SUBSET_FUNCTION */
    BMCV_WEBUSB_INTERFACE,                             /* bFirstInterface: the vendor one.
                                                          Interface 0 is MIDI and keeps the
                                                          driver it already has. */
    0x00,                                              /* bReserved */
    (uint8_t) (MSOS20_FUNCTION_LEN & 0xFF),            /* wSubsetLength */
    (uint8_t) (MSOS20_FUNCTION_LEN >> 8),

    /* ---- compatible ID: bind WinUSB, 20 bytes ---- */
    0x14, 0x00,                                        /* wLength */
    0x03, 0x00,                                        /* wDescriptorType: FEATURE_COMPATIBLE_ID */
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,          /* CompatibleID */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    /* SubCompatibleID: none */

    /* ---- registry property: the device interface GUID, 132 bytes ---- */
    //
    // WinUSB binds without this, but nothing can then open the device: the GUID
    // is what a device interface is registered under, and what a host library
    // enumerates by. Omitting it produces a device that is bound to the right
    // driver and still unreachable.
    (uint8_t) (MSOS20_PROPERTY_LEN & 0xFF),            /* wLength */
    (uint8_t) (MSOS20_PROPERTY_LEN >> 8),
    0x04, 0x00,                                        /* wDescriptorType: FEATURE_REG_PROPERTY */
    0x07, 0x00,                                        /* wPropertyDataType: REG_MULTI_SZ */
    0x2A, 0x00,                                        /* wPropertyNameLength: 42 */
    U16('D'), U16('e'), U16('v'), U16('i'), U16('c'), U16('e'),
    U16('I'), U16('n'), U16('t'), U16('e'), U16('r'), U16('f'), U16('a'), U16('c'), U16('e'),
    U16('G'), U16('U'), U16('I'), U16('D'), U16('s'), U16('\0'),
    0x50, 0x00,                                        /* wPropertyDataLength: 80 */
    /* One GUID, then the empty string that terminates a REG_MULTI_SZ. Generated
       once for this module and never to be changed: it is what a host remembers
       the device by. */
    U16('{'),
    U16('9'), U16('7'), U16('5'), U16('f'), U16('4'), U16('4'), U16('d'), U16('9'), U16('-'),
    U16('0'), U16('d'), U16('0'), U16('8'), U16('-'),
    U16('4'), U16('3'), U16('f'), U16('d'), U16('-'),
    U16('8'), U16('b'), U16('3'), U16('e'), U16('-'),
    U16('1'), U16('2'), U16('7'), U16('c'), U16('a'), U16('8'), U16('a'), U16('f'), U16('f'), U16('f'), U16('9'), U16('d'),
    U16('}'),
    U16('\0'),
    U16('\0'),
};
// clang-format on

const uint8_t* bmcv_bos_descriptor(uint16_t* length)
{
  *length = (uint16_t) sizeof(bos);
  return bos;
}

const uint8_t* bmcv_msos20_descriptor(uint16_t* length)
{
  *length = (uint16_t) sizeof(msos20);
  return msos20;
}
