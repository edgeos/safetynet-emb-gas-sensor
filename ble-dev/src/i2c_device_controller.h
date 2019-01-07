/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef EXT_DEVICE_CONTROLLER_H__
#define EXT_DEVICE_CONTROLLER_H__

#include <stdint.h>
#include "nrf.h"
#include "bme280_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NUM_EEPROM_RECORDS_PER_BLE_PAYLOAD 10

#define M24M02_INFO_ADDRESS          (0x00000000)
#define M24M02_DATA_VALID            (0xA55A5AA5)
#define M24M02_UTC_TIME_ADDRESS      (0x00000004)
#define M24M02_NUM_RECORDS_ADDRESS   (0x00000008)
#define M24M02_CURRENT_WRITE_ADDRESS (0x0000000A)
#define M24M02_FIRST_DATA_ADDRESS    (0x0000000E)

#pragma(1)
typedef enum
{
    BME280   = 1 << 0,
    M24M02   = 1 << 1,
    SIMULATE = 1 << 7
} ext_device_type_t;

#pragma(1)
typedef struct
{
    uint32_t magic_number;
    uint32_t utc_time_ref_sec;
    uint16_t num_records;
    uint32_t current_write_address;
} m24m02_info_t;

#pragma(1)
typedef struct
{
    uint32_t utc_time_ref_sec;
    uint16_t gas_sensor_type;
    uint16_t gas_ppm;
    uint32_t pressure;
    int32_t  temperature;
    uint32_t humidity;
} m24m02_storage_block_t;


/**@brief Initialize External Device Interfaces
 *
 */
void ext_device_init(ext_device_type_t use_sensors);

/**@brief Initialize External Device Interfaces
 *
 */
int8_t get_sensor_data(ext_device_type_t get_sensor, uint8_t * p_data, uint16_t * p_data_length);

int8_t update_utc_time_eeprom(uint32_t * utc_time_new);

int8_t store_sensor_data_eeprom(m24m02_storage_block_t *new_data, uint32_t *ms_since_last_update);

int8_t get_sensor_data_eeprom(bool * restart_read, bool * finished_read, m24m02_storage_block_t *recorded_data);

int8_t clear_sensor_data_eeprom(uint8_t * p_data, uint16_t * p_data_length);

#ifdef __cplusplus
}
#endif

#endif /* EXT_DEVICE_CONTROLLER_H__ */
