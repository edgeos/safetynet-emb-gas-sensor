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
#ifndef ADUCM355_CONTROLLER_H__
#define ADUCM355_CONTROLLER_H__

#include <stdint.h>
#include "nrf.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1) 

/**@brief Gas Sensor State */
typedef enum
{
    IDLE,             // aducm355 in sleep state
    PRIMING,          // aducm355 in sleep state, heaters turned on
    PRIMED,           // aducm355 in idle state, heaters in ready state
    MEASURING,        // aducm355 in measure state, heaters still on
    MEASUREMENT_DONE  // aducm355 in idle state, heaters off, algorithms
} gas_sensing_state_t;

/**@brief Gas Sensor Types */
typedef enum
{
    CO2 = 1,            
    CO  = 2             
} gas_sensor_t;

typedef struct
{ 
   gas_sensor_t gas_sensor;
   float gas_ppm;
   float freq;
   float Z_re;
   float Z_im;
} gas_sensor_results_t;

/**@brief Initialize ADuCM355 Uart Interface
 *
 */
uint32_t init_aducm355_iface(void);

uint32_t start_aducm355_measurement_seq(gas_sensor_t gas_sensor);

uint32_t continue_aducm355_measurement_seq(gas_sensor_results_t *gas_results, bool * measurement_done);

uint32_t stop_aducm355_measurement_seq(void);

void check_gas_sensing_state(gas_sensing_state_t *buf);

void check_aducm355_rx_buffer(void);

#ifdef __cplusplus
}
#endif

#endif /* ADUCM355_CONTROLLER_H__ */
