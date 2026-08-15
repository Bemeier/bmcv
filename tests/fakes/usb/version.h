// A fixed version for the host build.
//
// The real one is generated from VERSION by CMake into the firmware's build
// directory, which the native tests do not configure. These three numbers are
// arbitrary and deliberately not the project's - test_usblink.c asserts the
// version string is built from whatever these say, which is a test of the
// stringification, not of what happens to be in VERSION today.

#ifndef BMCV_TEST_FAKE_VERSION_H_
#define BMCV_TEST_FAKE_VERSION_H_

#define FW_VERSION_MAJOR 7
#define FW_VERSION_MINOR 8
#define FW_VERSION_PATCH 9

#endif /* BMCV_TEST_FAKE_VERSION_H_ */
