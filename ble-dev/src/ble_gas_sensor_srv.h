#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

// 4001c63b-0b4c-4d95-8451-a0d4a5d77036 from https://www.uuidgenerator.net/version4
#define CUSTOM_SERVICE_UUID_BASE         {0x36, 0x70, 0xD7, 0xA5, 0xD4, 0xA0, 0x51, 0x84, \
                                          0x4D, 0x95, 0x0B, 0x4C, 0x3B, 0xC6, 0x01, 0x40}

#define VOLTAGE_WRISTBAND_SERVICE_UUID               0x1400
#define CUSTOM_VALUE_CHAR_UUID            0x1401                                  