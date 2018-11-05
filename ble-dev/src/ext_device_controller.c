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
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "m24m02.h"
#include "bme280.h"
#include "ext_device_controller.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifndef SCL_PIN
#define SCL_PIN             NRF_GPIO_PIN_MAP(0,26)    // SCL signal pin
#define SDA_PIN             NRF_GPIO_PIN_MAP(1,15)    // SDA signal pin
#define WAKEUP_PIN          NRF_GPIO_PIN_MAP(1,8)     // Wakeup signal pin
#define INT_PIN             NRF_GPIO_PIN_MAP(1,7)     // Data ready (active low)
#endif

struct bme280_dev i2c_dev_bme280;
struct m24m02_dev i2c_dev_m24m02;

// to track which sensors are enabled / connected
static uint16_t enabled_devices = 0;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
/* Busy bool for I2C */
static volatile bool m_xfer_done1 = false;
/* I2C enable bool */
static volatile bool twi_enabled = false;

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                // do nothing for now
            }
            m_xfer_done1 = true;
            break;
        default:
            break;
    }
}

static void twi_wait(void)
{
    uint16_t ctr = 0;
    uint16_t limit_us = 500*64;
    do
    {
        //nrf_delay_us(1);
        ctr++;
    }while((m_xfer_done1 == false) & (ctr < limit_us));
}


/**
 * @brief TWI initialization.
 */
static void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = true//,
       //.hold_bus_uninit    = true 
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    twi_enabled = true;
}

static uint32_t twi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ret_code_t err_code;

    /* Prepare for the read with a register write */
    uint8_t reg_read[2] = {reg_addr, 0};
    m_xfer_done1 = false; 
    err_code = nrf_drv_twi_tx(&m_twi, dev_id, reg_read, 1, false);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    /* Now read the register */
    m_xfer_done1 = false; 
    err_code = nrf_drv_twi_rx(&m_twi, dev_id, data, len);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    return err_code;
}

static uint32_t twi_16bit_address_read(uint8_t dev_id, uint8_t *reg_addr, uint8_t *data, uint16_t len)
{
    ret_code_t err_code;

    /* Prepare for the read with a register write */
    uint8_t reg_read[2] = {*reg_addr, *(reg_addr++)};
    m_xfer_done1 = false; 
    err_code = nrf_drv_twi_tx(&m_twi, dev_id, reg_read, 2, true);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    /* Now read the register */
    m_xfer_done1 = false; 
    err_code = nrf_drv_twi_rx(&m_twi, dev_id, data, len);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    return err_code;
}

static uint32_t twi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ret_code_t err_code;
    
    /* Build tx buffer */
    uint8_t reg_write[20];
    reg_write[0] = reg_addr;
    if(data) memcpy(&reg_write[1],data,len);

    /* Transfer tx buffer */
    m_xfer_done1 = false;
    err_code = nrf_drv_twi_tx(&m_twi, dev_id, reg_write, 1+len, false);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    return err_code;
}

static bool scan_for_twi_device(uint8_t dev_address)
{
    ret_code_t err_code;

    uint8_t sample_data;
    err_code = nrf_drv_twi_rx(&m_twi, dev_address, &sample_data, sizeof(sample_data));
    twi_wait();
    nrf_delay_ms(10);
    return (err_code == NRF_SUCCESS) ? true:false;
}

static void print_bme280_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        NRF_LOG_INFO("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        //NRF_LOG_INFO("\n")
        NRF_LOG_INFO("Temp:  %ld, Humid:  %ld  \r",comp_data->temperature, comp_data->humidity);
#endif
}

static void sensor_bme280_init(void)
{
    i2c_dev_bme280.dev_id = BME280_I2C_ADDR_SEC;
    i2c_dev_bme280.intf = BME280_I2C_INTF;
    i2c_dev_bme280.read = twi_read;
    i2c_dev_bme280.write = twi_write;
    i2c_dev_bme280.delay_ms = nrf_delay_ms;
    if(scan_for_twi_device(BME280_I2C_ADDR_SEC))
    {
        if (bme280_init(&i2c_dev_bme280) == BME280_OK)
        {
            enabled_devices |= BME280;
        }
    }
    else
    {
        NRF_LOG_WARNING("BME280 not found! Skipping...");
    }
}

static int8_t bme280_stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;
    static bool prepare_measurement = true; // because of the delay for the measurement to complete we toggle what we do

    /* Recommended mode of operation: Humidity Sensing (p17 of datasheet) */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_NO_OVERSAMPLING;
    dev->settings.osr_t = BME280_OVERSAMPLING_1X;
    dev->settings.filter = BME280_FILTER_COEFF_OFF;
    settings_sel = BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, dev);

    /* Ping for sensor data */
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
    nrf_delay_ms(20); // measurement time
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    print_bme280_sensor_data(&comp_data);

    return rslt;
}

static void eeprom_m24m02_init(void)
{
    i2c_dev_m24m02.dev_id = M24M02_I2C_ADDR_SEC;
    i2c_dev_m24m02.read = twi_16bit_address_read;
    i2c_dev_m24m02.write = twi_write;
    if(scan_for_twi_device(M24M02_I2C_ADDR_SEC))
    {
        if (i2c_eeprom_init(&i2c_dev_m24m02) == true)
        {
            enabled_devices |= M24M02;
        }
    }
    else
    {
        NRF_LOG_WARNING("M24M02 not found! Skipping...");
    }
}

void ext_device_init(ext_device_type_t use_devices)
{
    int8_t rslt_bme280 = BME280_OK;
    
    twi_init();
    if(use_devices & BME280)
    {  
        sensor_bme280_init();
    }
    if(use_devices & M24M02)
    {  
        eeprom_m24m02_init();
    }
}

int8_t get_sensor_data(ext_device_type_t get_sensor)
{
    int8_t ret_code = 0;

    // check if sensor is enabled
    if (!(enabled_devices & get_sensor)) return 1;

    // query by sensor type
    switch(get_sensor) 
    {
        case BME280:
            bme280_stream_sensor_data_forced_mode(&i2c_dev_bme280);
            break;
        default:
            break;
    }
    return ret_code;
}