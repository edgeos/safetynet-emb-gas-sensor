
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
#include "m24m02_2.h"

#ifndef M24M02_WC
#define M24M02_WC               NRF_GPIO_PIN_MAP(0,10)    // SCL signal pin 
#define M24M02_E2               NRF_GPIO_PIN_MAP(0,9)    // SCL signal pin
#endif

uint32_t eepromAddress;
uint8_t e2bm      = 0xf7; // Clear Bit 3
uint8_t addshf    = 0x01; // # of bit to be shifted - 1 bits
uint8_t rwbm      = 0xfe; // Clear Bit  0

uint8_t addrA1716 = 0;    // = (uint8_t)((eepromAdd & 0x3ff) >> 16 ); //Get the A17 and A16 of the eeprom Address
uint8_t addrMsb   = 0;    // = (0xff00 & (uint8_t)( eepromAdd & 0xffff)) >> 8;
uint8_t addrLsb   = 0;    // = 0x00ff & (uint8_t)( eepromAdd & 0xffff)   ;
uint8_t dscWrite  = 0;    // = 0xA0 | (addrA1716 << addshf) & e2bm & rwbm ;
uint8_t dscRead   = 0;

// used in here only
static struct m24m02_dev *dev;

static void address_calculation(uint32_t eepromAddress)
{
    addrA1716 = (uint8_t)((eepromAddress & 0x030000) >> 16 );  //Get the A17 and A16 of the eeprom Address
    addrMsb   = (uint8_t)((eepromAddress & 0xff00) >> 8);      //get Msb of address
    addrLsb   = 0x00ff & (uint8_t)( eepromAddress & 0xffff);   //get Lsb of address
    dscWrite  = 0xa0 | (addrA1716 << addshf) & e2bm & rwbm ;
}

static void write(uint32_t eepromAddress, uint8_t data_byte, uint8_t *infoArray)
{
    uint8_t write_address[2] = {0};
    for (uint8_t k = 0; k < data_byte; k++)     
    {
        address_calculation(eepromAddress);
        write_address[0] = addrMsb;
        write_address[1] = addrLsb;
        dev->write(dscWrite, &write_address[0], infoArray);
        infoArray++;
        eepromAddress++;
    }
}

static void read(uint32_t eepromAddress, uint8_t data_byte, uint8_t *infoArray)
{
    uint8_t read_address[2] = {0};
    for (uint8_t k = 0; k < data_byte; k++)     
    {
        address_calculation(eepromAddress);
        read_address[0] = addrMsb;
        read_address[1] = addrLsb;
        dev->read(dscWrite, &read_address[0], infoArray);
        infoArray++;
        eepromAddress++;
    }
}

bool m24m02_eeprom_init(struct m24m02_dev *dev_init) 
{
    dev = dev_init;
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true); // false = init low = WAKEUP
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(M24M02_WC, &out_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(M24M02_E2, &out_config));
   
    nrf_drv_gpiote_out_clear(M24M02_WC);
    nrf_drv_gpiote_out_set(M24M02_E2);
    nrf_delay_ms(80);
    return true;
}

/****************************************************************
Function Name:  eeprom
Functional Description: Read/Write Data from/to the EEPROM.
Input:          rw (1 means Write, 0 means Read)
                eepromAddress (Byte Address, for M24M02-DR, There are maximum 2Mbit = 262144 bytes)
                data_type (Number of bytes to be read/write, 
                           the maxiumm should be page size: 256 bytes, see data sheet for details)
                infoArray (Array that contains the data to be written OR
                           Array that store the eeprom reading output)
Output:         char (1=success, 0=fail)
Other info:     Please remind that if the data_byte>256 while rw=1, it rewrites the page from beginning.
*****************************************************************/
bool m24m02_eeprom(uint8_t rw, uint32_t eepromAddress, uint8_t data_byte, uint8_t *infoArray)
{
     //no matter the read or write mode, a boundary of 256-multiple should be noticed
    uint8_t i,j;
    uint8_t k;

    if(rw==1)
    {
        write(eepromAddress, data_byte, infoArray);
    }
    else
    {
        read(eepromAddress, data_byte, infoArray);
    }
}

/*
bool m24m02_eeprom_clear_address_byte(uint32_t eepromAddress, uint8_t data_byte)
{
    uint8_t k;

    addrA1716 = (uint8_t)((eepromAddress & 0x030000) >> 16 ); //Get the A17 and A16 of the eeprom Address
    addrMsb   = (uint8_t)((eepromAddress & 0xff00) >> 8);     //get Msb of address
    addrLsb   = 0x00ff & (uint8_t)( eepromAddress & 0xffff);  //get Lsb of address
    dscWrite  = 0xa0 | (addrA1716 << addshf) & e2bm & rwbm ;

           Soft_I2C_Start();
           Soft_I2C_Write(dscWrite);
           Soft_I2C_Write(addrMsb );
           Soft_I2C_Write(addrLsb);
           for (k=0;k <  data_byte;k++)       //strlen(infoArray)
           {
             Soft_I2C_Write(255);
             Rdelay_ms(10);  //After each write, at least 10ms delay should be provided
           }


           Soft_I2C_Stop();
           nrf_delay_ms(5);
   eeprom_isAvailable=1;
   return 1;
}*/


