/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_rtc.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "saadc_controller.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAADC_SAMPLES_IN_BUFFER    2    // Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_VOLTAGE_SAMPLES      2    

static nrf_saadc_value_t m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static bool              m_saadc_initialized = false;
static float             vRef = 3.6f;
static float             resCts = 16384; // 8-bit = 255, 10-bit = 1024, 12-bit = 4096, 14-bit = 16384
static float             battery_voltage = 2.1f; // start moving at average at "full" battery charge
static float             new_meas_weight = 0.1f;

float voltO2 = 0.f;
float vRaw = 0.f;

static void clear_FPU_interrupts(void)
{
    __set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
}

/** @brief Function callback for SAADC electrode measurement done
 */
static void saadc_electrode_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
        ret_code_t err_code;

        // convert sample
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        APP_ERROR_CHECK(err_code);

        // moving average, new measurements are weighted by X% and old by (1-X)%
        battery_voltage = battery_voltage*(1-new_meas_weight) + new_meas_weight*(vRef*p_event->data.done.p_buffer[0])/resCts;
        //clear_FPU_interrupts();

		vRaw = (vRef*p_event->data.done.p_buffer[0])/resCts;
		voltO2 = (vRef*p_event->data.done.p_buffer[1])/resCts;

        // turn off SAADC
        m_saadc_initialized = false;
        nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
    }
}

/** @brief Function to initialize the SAADC settings for the battery voltage measurement
 */
static void saadc_channels_init(void)
{
    ret_code_t err_code;
    
    nrf_saadc_channel_config_t channel_config[] = {
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3),
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0) };
    channel_config[0].reference = NRF_SAADC_REFERENCE_INTERNAL;
    channel_config[1].reference = NRF_SAADC_REFERENCE_INTERNAL;


    //Configure SAADC
    nrf_drv_saadc_config_t saadc_config;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;                              //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
    saadc_config.low_power_mode = true;                                                   //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    APP_ERROR_CHECK(nrf_drv_saadc_init(&saadc_config, saadc_electrode_callback));                   //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, channel_config);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(1, channel_config + 1);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    m_saadc_initialized = true; 
}

void saadc_sample_battery_level(uint8_t *battery_level)
{
    static uint8_t prev_level = 0;

    if (m_saadc_initialized == false)
    {
        saadc_channels_init();                     //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
    }

    //    4.20v = 100%
    //    4.03v = 76%
    //    3.86v = 52%
    //    3.83v = 42%
    //    3.79v = 30%
    //    3.70v = 11%
    //    3.6?v = 0%
    if (battery_voltage >= 2.1f)
    {
        *battery_level = 100;
    }
    else if (battery_voltage > 2.015f)
    {
        *battery_level = (uint8_t) ((((battery_voltage - 2.015f)/(2.1f-2.015f)) *24) + 76);
    }
    else if (battery_voltage > 1.93f)
    {
        *battery_level = (uint8_t) ((((battery_voltage - 1.93f)/(2.015f-1.93f))*24) + 52);
    }
    else if (battery_voltage > 1.915f)
    {
        *battery_level = (uint8_t) ((((battery_voltage - 1.915f)/(1.93f-1.915f))*10) + 42);
    }
    else if (battery_voltage > 1.895f)
    {
        *battery_level = (uint8_t) ((((battery_voltage - 1.895f)/(1.915f-1.895f))*12) + 30);
    }
    else if (battery_voltage > 1.85f)
    {
        *battery_level = (uint8_t) ((((battery_voltage - 1.85f)/(1.895f-1.85f))*19) + 11);
    }
    else
    {
        *battery_level = (uint8_t) ((((battery_voltage - 1.8f)/(1.85f-1.8f))*100) + 0);
    }
    clear_FPU_interrupts();

    if (*battery_level == prev_level) 
    {
        if (*battery_level == 0)
        {
            *battery_level += 1;
        }
        else
        {
            *battery_level -= 1;
        }
    }
    prev_level = *battery_level;

    // trigger sampling, first sample will always be 0 waiting for the first interrupt
    nrf_drv_saadc_sample();
}
/** @} */