/* Auto-generated version header */
#ifndef VERSION_H
#define VERSION_H

#define FW_VERSION_MAJOR  1 //16-bit
#define FW_VERSION_MINOR  0 //8-bit
#define FW_VERSION_PATCH  16 //8-bit
#define FW_VERSION_STRING "v1.00.016"

#include <stdint.h>

typedef struct
{
  uint16_t major;
  uint8_t minor;
  uint8_t patch;
} firmware_version_t;

__attribute__((section(".version"), used)) static const firmware_version_t fw_version_struct
    = { .major = FW_VERSION_MAJOR, .minor = FW_VERSION_MINOR, .patch = FW_VERSION_PATCH };

#endif //VERSION_H
