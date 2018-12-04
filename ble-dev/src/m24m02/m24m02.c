
/*
*Lib defining the M24m)@ eeeprom form ST for the Exeger Light band project . 
*26/05/2014
*Copyright Exeger Systems AB  2014
*
*Author:Kartik karuna (Kartik.karuna@exeger.com) 
*
*/
#include <stdbool.h>
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "m24m02.h"

// Defines 
#define MEM_BASE_ADD            0xA0            //!< 6 MSBs of the I2C ADD, the LSB depends on read/write page
#define INT_STATUS		0x00
#define INT_ENABLE		0x01
#define PAGE_SIZE               0x80

uint8_t DEV_REVERSE_LOOKUP[] = {0x08, 0x0A, 0x0C, 0x0E};
/*
#define PAGE_0        0x08            // PAGE 0 (E2 =1 , A17=A16=0,W/R=D.C)
#define PAGE_1        0x0A            // PAGE 1 (E2 =1 , A17=0 A16=1,W/R=D.C)
#define PAGE_2        0x0C            // PAGE 2 (E2 =1 , A17=1 A16=0, W/R=D.C)
#define PAGE_3        0x0E            // PAGE 3 (E2 =1 , A17=A16=1, W/R=D.C)
terms block and devices is used interchangeably
*/

//Memory Blocks 
#define BLOCK_SIZE              0x10000
#define BLOCK_MAX               0x3FFF0 //((4*BLOCK_SIZE)-15)   //0x40000 //saving last 15 bytes to save memory location.
#define BUS                     true            //Bool valve to select bus 

// enable pins
#ifndef M24M02_WC
#define M24M02_WC               NRF_GPIO_PIN_MAP(0,10)    // SCL signal pin 
#define M24M02_E2               NRF_GPIO_PIN_MAP(0,9)    // SCL signal pin
#endif
#define pointer_address         0xFFF5  //on AE device

// used in here only
static struct m24m02_dev *dev;

bool i2c_eeprom_init(struct m24m02_dev *dev_init) 
{
    dev = dev_init;
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true); // false = init low = WAKEUP
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(M24M02_WC, &out_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(M24M02_E2, &out_config));
   
    nrf_drv_gpiote_out_set(M24M02_WC);
    nrf_drv_gpiote_out_set(M24M02_E2);
    nrf_delay_ms(1);
    return true;
}

bool i2c_eeprom_erase() 
{
    // initialize all bytes to 0
    uint8_t data[128] = {0};
    uint32_t addr = 0x0;

     //  Erasing EEPROM
    while (addr < BLOCK_MAX) 
    {
      i2c_eeprom_write(addr, data, sizeof(data));
      addr += sizeof(data); 
    }
    return true; //Erase done
}

bool i2c_eeprom_write(uint32_t address, uint8_t* data, uint32_t length) 
{
    if (address > BLOCK_MAX)  {return false;}
    if (address + length > BLOCK_MAX) {return false;}

    uint32_t start_byte        = address;
    uint32_t end_byte          = address + length;
    uint32_t curr_device_start = (address / BLOCK_SIZE) * BLOCK_SIZE;
    uint32_t next_device_start = curr_device_start + BLOCK_SIZE;

    bool success = true;
    while (end_byte > next_device_start && success) 
    {
        uint8_t dev_offset = start_byte / BLOCK_SIZE;
        uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset];
        success = i2c_eeprom_write_buffer(dev_id, (uint16_t)(start_byte % BLOCK_SIZE),(uint8_t*)&(data[start_byte - address]), next_device_start - start_byte);

        curr_device_start = next_device_start;
        next_device_start = curr_device_start + BLOCK_SIZE;
        start_byte        = curr_device_start;
    }
    if (!success) {return false;}

    uint8_t dev_offset = start_byte / BLOCK_SIZE;
    uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset];
    uint16_t lenght_send = end_byte - start_byte;
    return i2c_eeprom_write_buffer(dev_id, (uint16_t)(start_byte % BLOCK_SIZE),
               (uint8_t*)&(data[start_byte - address]),lenght_send);
}

bool i2c_eeprom_write_buffer(uint8_t dev_id, uint16_t address, uint8_t* data, uint16_t length) 
{
    uint16_t start_byte = address;
    uint16_t end_byte   = address + length;

    uint16_t curr_page_start = (address / PAGE_SIZE) * PAGE_SIZE;
    uint32_t next_page_start = curr_page_start + PAGE_SIZE;
    bool success = true;
    while (end_byte > next_page_start && success) 
    {
      success = i2c_eeprom_write_page(dev_id, start_byte, &(data[start_byte - address]), next_page_start - start_byte);
      curr_page_start = next_page_start;
      next_page_start = curr_page_start + PAGE_SIZE;
      start_byte = curr_page_start;
    }
    if (!success) {return false; }

    uint16_t lenght_send = end_byte - start_byte;
    return i2c_eeprom_write_page(dev_id, start_byte, &(data[start_byte - address]), lenght_send);
}

bool i2c_eeprom_write_page(uint8_t dev_id, uint16_t eeaddress, uint8_t* data, uint8_t length ) 
{
    // enable WC pin (active low)
    nrf_drv_gpiote_out_clear(M24M02_WC);

    uint8_t buffer_len = length+2;
    uint8_t data_buffer[130];

    // figure out memory address bytes
    uint8_t add_high = (uint8_t)((eeaddress >> 8) &0xFF);
    data_buffer[0] = add_high;
    uint8_t add_low = (uint8_t)(eeaddress & 0xFF);
    data_buffer[1] = add_low;

    // copy data to buffer
    memcpy(&data_buffer[2], data, length);

    // write page
    uint32_t err_code;
    err_code = dev->write(dev_id, data_buffer[0], &data_buffer[1], buffer_len-1);

    // disable and add maximum write latency as delay between success writes
    nrf_drv_gpiote_out_set(M24M02_WC);
    nrf_delay_ms(10);

    return (err_code == 0) ? true:false;
}

bool i2c_eeprom_read(uint32_t address, uint8_t* data, uint32_t length) 
{
    if (address > BLOCK_MAX) {return false;}
    if (address + length > BLOCK_MAX) {return false;}

    uint32_t start_byte        = address;
    uint32_t end_byte          = address + length;
    uint32_t curr_device_start = (address / BLOCK_SIZE) * BLOCK_SIZE;
    uint32_t next_device_start = curr_device_start + BLOCK_SIZE;

    bool success = true;
    while (end_byte > next_device_start && success) 
    {
        uint8_t dev_offset = start_byte / BLOCK_MAX;
        uint8_t dev_id     = (MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset]);
        success = i2c_eeprom_read_buffer(&dev_id, (uint16_t)(start_byte % BLOCK_SIZE), (uint8_t*)&(data[start_byte - address]), next_device_start - start_byte);

        curr_device_start = next_device_start;
        next_device_start = curr_device_start + BLOCK_SIZE;
        start_byte        = curr_device_start;
    }
    if (!success) 
    {
        return false;
    }
  
    uint8_t dev_offset = start_byte / BLOCK_SIZE;
    uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset];
    return i2c_eeprom_read_buffer(&dev_id, (uint16_t)(start_byte % BLOCK_SIZE), (uint8_t*)&(data[start_byte - address]), (uint16_t)(end_byte - start_byte));
}

bool i2c_eeprom_read_buffer(uint8_t *dev_id, uint16_t address, uint8_t *buffer, uint16_t length) 
{
    bool success;

    // write protect
    nrf_drv_gpiote_out_set(M24M02_WC); 
    //nrf_delay_ms(5);
    
    uint8_t add_buffer[2];
    add_buffer[0]=(uint8_t)((address >> 8) &0xFF);
    add_buffer[1]=(uint8_t)(address & 0xFF);

    // read page
    uint32_t err_code;
    err_code = dev->read(dev_id, &add_buffer[0], buffer, length);
    return (err_code == 0) ? true:false;	
}

uint32_t eeprom_read_add_pointer(void)
{
    uint8_t dev_id = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[3];
    uint8_t pointer_address_buffer[4];
    uint8_t length = 4;
    if(i2c_eeprom_read_buffer(&dev_id,(uint16_t)pointer_address,(uint8_t*)&pointer_address_buffer[0], (uint16_t)length))
    {
        uint32_t t_buffer = (uint32_t) pointer_address_buffer[0] << 24  |
                            (uint32_t) pointer_address_buffer[1] << 16  |
                            (uint32_t) pointer_address_buffer[2] <<  8  |
                            (uint32_t) pointer_address_buffer[3];
        return t_buffer;
    }
    return ((uint32_t)0);
}

bool eeprom_update_add_pointer(uint32_t address)
{
    uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[3];
    uint8_t pointer_address_buffer[4];
    uint8_t lenght_send = sizeof(uint32_t);
	
    //conver 32 bit address in to 8 bytes chunks.
    pointer_address_buffer[0] = (uint8_t)((address >> 24) & 0x000000FF);
    pointer_address_buffer[1] = (uint8_t)((address >> 16) & 0x000000FF);
    pointer_address_buffer[2] = (uint8_t)((address >> 8) & 0x000000FF);
    pointer_address_buffer[3] = (uint8_t)(address & 0x000000FF);
	
    return i2c_eeprom_write_page(dev_id,(uint16_t)pointer_address,pointer_address_buffer,lenght_send);
}


