// The descriptors that get a browser talking to the module with no driver
// installed.
//
// Worth testing far more than their size suggests, because every mistake
// available here fails the same silent way: the device enumerates, Windows
// binds nothing useful to it, and the browser simply never offers the device.
// No error, no log, nothing to read. And they are four nested lengths that all
// have to agree, three of them sums of what follows - so adding one byte
// anywhere moves numbers in three other places.
//
// What is checked is internal consistency, which is the whole class of error a
// compiler cannot see. Whether Windows likes the result is a question only
// Windows can answer.

#include "testkit.h"
#include "usbd_webusb.h"
#include <string.h>

static uint16_t u16le(const uint8_t* p) { return (uint16_t) (p[0] | (p[1] << 8)); }

TEST_CASE(the_bos_descriptor_is_internally_consistent)
{
  uint16_t len;
  const uint8_t* d = bmcv_bos_descriptor(&len);

  CHECK(len == BMCV_BOS_TOTAL_LEN);
  CHECK(d[0] == 0x05);        // bLength of the header itself
  CHECK(d[1] == 0x0F);        // bDescriptorType: BOS
  CHECK(u16le(d + 2) == len); // wTotalLength covers everything

  const uint8_t caps = d[4];
  CHECK(caps == 2);

  // Walk the capabilities and make sure they tile the descriptor exactly. A
  // capability whose bLength is wrong leaves the host reading the next one from
  // the middle of this one.
  uint16_t at = 5;
  for (uint8_t i = 0; i < caps; i++)
  {
    const uint8_t cap_len = d[at];
    CHECK(cap_len >= 4);
    CHECK(d[at + 1] == 0x10); // DEVICE_CAPABILITY
    CHECK(d[at + 2] == 0x05); // PLATFORM
    at = (uint16_t) (at + cap_len);
    CHECK(at <= len);
  }
  CHECK(at == len);
}

TEST_CASE(the_bos_descriptor_publishes_both_vendor_codes)
{
  uint16_t len;
  const uint8_t* d = bmcv_bos_descriptor(&len);

  // The two capabilities put their vendor code in different places, because
  // they carry different things before it: the WebUSB one has a two-byte
  // bcdVersion after its UUID, the Microsoft one has a four-byte Windows
  // version and then a two-byte length. Writing this test from the first
  // layout and applying it to the second is exactly the mistake it caught.
  const uint8_t* webusb = d + 5;      /* 24 bytes: 4 header, 16 UUID, 2 bcd, code, page */
  const uint8_t* msos   = d + 5 + 24; /* 28 bytes: 4 header, 16 UUID, 4 winver, 2 len, code, alt */

  // The host only learns these from here, so a mismatch with what the request
  // handler answers to is a device that is asked nothing.
  CHECK(webusb[22] == BMCV_WEBUSB_VENDOR_CODE);
  CHECK(msos[26] == BMCV_MSOS20_VENDOR_CODE);

  // And the length it advertises for the descriptor set has to be the length
  // that set actually is, or Windows asks for the wrong number of bytes.
  uint16_t set_len;
  bmcv_msos20_descriptor(&set_len);
  CHECK(u16le(msos + 24) == set_len);
}

TEST_CASE(the_msos20_set_nests_correctly)
{
  uint16_t len;
  const uint8_t* d = bmcv_msos20_descriptor(&len);

  CHECK(len == BMCV_MSOS20_TOTAL_LEN);

  // Set header.
  CHECK(u16le(d) == 10);         // wLength of this header
  CHECK(u16le(d + 2) == 0x0000); // SET_HEADER
  CHECK(u16le(d + 8) == len);    // wTotalLength covers the whole set

  // Configuration subset: its total must reach exactly the end of the set.
  const uint8_t* cfg = d + 10;
  CHECK(u16le(cfg) == 8);          // wLength of the subset header
  CHECK(u16le(cfg + 2) == 0x0001); // SUBSET_CONFIGURATION
  const uint16_t cfg_total = u16le(cfg + 6);
  CHECK(10 + cfg_total == len);

  // Function subset: its length must reach exactly the end of the configuration
  // subset, and it must name the vendor interface rather than the MIDI one.
  const uint8_t* fn = cfg + 8;
  CHECK(u16le(fn) == 8);
  CHECK(u16le(fn + 2) == 0x0002); // SUBSET_FUNCTION
  CHECK(fn[4] == BMCV_WEBUSB_INTERFACE);
  CHECK(fn[4] != 0); // interface 0 is MIDI and keeps its own driver
  const uint16_t fn_total = u16le(fn + 6);
  CHECK(8 + fn_total == cfg_total);

  // The features inside it must tile that length exactly.
  uint16_t at        = 0;
  uint8_t saw_winusb = 0, saw_guid = 0;
  while (at + 4 <= fn_total - 8)
  {
    const uint8_t* feat     = fn + 8 + at;
    const uint16_t feat_len = u16le(feat);
    CHECK(feat_len >= 4);

    if (u16le(feat + 2) == 0x0003)
    {
      saw_winusb = memcmp(feat + 4, "WINUSB\0", 7) == 0;
      CHECK(feat_len == 20);
    }
    if (u16le(feat + 2) == 0x0004)
    {
      saw_guid = 1;
    }

    at = (uint16_t) (at + feat_len);
  }
  CHECK(at + 8 == fn_total);

  // Without the compatible ID Windows binds no driver; without the GUID it
  // binds one and nothing can open the device.
  CHECK(saw_winusb);
  CHECK(saw_guid);
}

// The registry property carries two UTF-16 strings whose declared lengths are
// separate from the bytes that follow them, which is the other place a byte
// count can quietly disagree.
TEST_CASE(the_registry_property_declares_its_own_strings)
{
  uint16_t len;
  const uint8_t* d = bmcv_msos20_descriptor(&len);

  // Find it rather than counting to it, so this test does not have to move
  // whenever something is inserted before it.
  const uint8_t* prop = NULL;
  for (uint16_t at = 10 + 8 + 8; at + 4 <= len;)
  {
    const uint16_t feat_len = u16le(d + at);
    if (feat_len < 4)
      break;
    if (u16le(d + at + 2) == 0x0004)
    {
      prop = d + at;
      break;
    }
    at = (uint16_t) (at + feat_len);
  }
  CHECK(prop != NULL);
  if (!prop)
    return;

  const uint16_t prop_len = u16le(prop);
  CHECK(u16le(prop + 4) == 7); // REG_MULTI_SZ

  const uint16_t name_len = u16le(prop + 6);
  const uint16_t data_len = u16le(prop + 8 + name_len);

  // Header, name, its length field, and the data must account for every byte.
  CHECK(2 + 2 + 2 + 2 + name_len + 2 + data_len == prop_len);

  // Both are UTF-16 strings, so both lengths must be even and both must end in
  // a NUL code unit.
  CHECK(name_len % 2 == 0);
  CHECK(data_len % 2 == 0);
  CHECK(prop[8 + name_len - 2] == 0 && prop[8 + name_len - 1] == 0);

  // REG_MULTI_SZ ends with an empty string, so the data ends in two NULs.
  const uint8_t* data = prop + 8 + name_len + 2;
  CHECK(data[data_len - 4] == 0 && data[data_len - 3] == 0);
  CHECK(data[data_len - 2] == 0 && data[data_len - 1] == 0);

  // And the name really is the one Windows looks for.
  static const char want[] = "DeviceInterfaceGUIDs";
  for (uint16_t i = 0; i < sizeof(want) - 1; i++)
  {
    CHECK(prop[8 + i * 2] == (uint8_t) want[i]);
    CHECK(prop[8 + i * 2 + 1] == 0);
  }
}

int main(void)
{
  RUN_TEST(the_bos_descriptor_is_internally_consistent);
  RUN_TEST(the_bos_descriptor_publishes_both_vendor_codes);
  RUN_TEST(the_msos20_set_nests_correctly);
  RUN_TEST(the_registry_property_declares_its_own_strings);
  return TESTKIT_SUMMARY();
}
