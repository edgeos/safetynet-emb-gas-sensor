/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
#ifndef MATCHBOX_V1_H
#define MATCHBOX_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions
#define LEDS_NUMBER                2
#define LED_1                      NRF_GPIO_PIN_MAP(0,27)
#define LED_2                      NRF_GPIO_PIN_MAP(0,28)
#define LED_START                  LED_1
#define LED_STOP                   LED_2
#define LEDS_ACTIVE_STATE          1
#define LEDS_LIST                  { LED_1, LED_2 }
#define LEDS_INV_MASK              LEDS_MASK
#define BSP_LED_0                  LED_1
#define BSP_LED_1                  LED_2

// Buttons definitions; needed for boards.c
#define BUTTONS_NUMBER             1
#define BUTTONS_ACTIVE_STATE       0
#define BUTTON_1                   NRF_GPIO_PIN_MAP(0,11)
#define BUTTON_PULL                NRF_GPIO_PIN_PULLUP
#define BUTTONS_LIST               { BUTTON_1 }

// Buzzer / Alarm
#define BUZZER_GPIO                NRF_GPIO_PIN_MAP(0,16)

// I2C for BME280 + M24M02
#define SCL_PIN                    NRF_GPIO_PIN_MAP(0,25)  
#define SDA_PIN                    NRF_GPIO_PIN_MAP(0,26)    

// UART for ADuCM355
#define ADUCM355_UART_ENABLE       NRF_GPIO_PIN_MAP(0,8)
#define ADUCM355_RX_PIN            NRF_GPIO_PIN_MAP(0,6) 
#define ADUCM355_TX_PIN            NRF_GPIO_PIN_MAP(0,7)        

// Heaters for Gas Sensors
#define ADUCM355_HEATER1           NRF_GPIO_PIN_MAP(0,3)
#define ADUCM355_HEATER2           NRF_GPIO_PIN_MAP(0,4) 
#define ADUCM355_HEATER3           NRF_GPIO_PIN_MAP(0,14)   
#define ADUCM355_HEATER4           NRF_GPIO_PIN_MAP(0,15)   

// Range Control Mux for Gas Sensor Load
#define ADUCM355_CAP_MUX_NE        NRF_GPIO_PIN_MAP(0,24) 
#define ADUCM355_CAP_MUX_SO        NRF_GPIO_PIN_MAP(0,23) 
#define ADUCM355_CAP_MUX_S1        NRF_GPIO_PIN_MAP(0,22) 

#ifdef __cplusplus
}
#endif

#endif // MATCHBOX_V1_H
