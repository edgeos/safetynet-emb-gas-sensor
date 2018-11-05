/* Found on Github and modified to be good
*/


#ifndef M24M02_H__
#define M24M02_H__


#include <stdbool.h>
#include <stdint.h>

#define M24M02_I2C_ADDR_SEC            0xA0   


/*!
 * @brief Type definitions
 */
typedef uint32_t (*m24m02_com_fptr_write_t)(uint8_t dev_id, uint8_t reg_addr,
                                            uint8_t *data, uint16_t len);
typedef uint32_t (*m24m02_com_fptr_read_t)(uint8_t dev_id, uint8_t *reg_addr,
                                            uint8_t *data, uint16_t len);

/*!
 * @brief m24m02 device structure
 */
struct m24m02_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! Read function pointer */
	m24m02_com_fptr_read_t read;
	/*! Write function pointer */
	m24m02_com_fptr_write_t write;
};

bool i2c_eeprom_init(struct m24m02_dev *dev);
/**
 *	Function:	i2c_eeprom_erase
 *
 *	Arguments:	n/a
 *
 *	Returns:	true/false
 *
 *	Description:	Erases the EEPROM.
**/
bool i2c_eeprom_erase();
/**
 *	Function:	i2c_eeprom_write_page
 *
 *	Arguments:	18 bit address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_write(uint32_t address, uint8_t* data, uint32_t length);
bool i2c_eeprom_write_buffer(uint8_t dev_id, uint16_t address, uint8_t* data, uint16_t length);
/**
 *	Function:	i2c_eeprom_write_page
 *
 *	Arguments:	Device Address
 *              16 bit eeprom address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_write_page(uint8_t dev_id, uint16_t eeaddress, uint8_t* data, uint8_t length );
/**
 *	Function:	i2c_eeprom_read_buffer
 *
 *	Arguments:	18 bit eeprom address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_read(uint32_t address, uint8_t* data, uint32_t length);
/**
 *	Function:	i2c_eeprom_read_buffer
 *
 *	Arguments:	Device Address
 *              16 bit eeprom address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_read_buffer(uint8_t dev_id, uint16_t address, uint8_t *buffer, uint16_t length); 

uint32_t eeprom_find_add_pointer(void);

bool eeprom_updateadd_pointer(uint32_t address);

#endif // M24M02_H__
