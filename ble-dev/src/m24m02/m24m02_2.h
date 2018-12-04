/* Found on Github and modified to be good
*/
#ifndef M24M02_H__
#define M24M02_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M24M02_I2C_ADDR_SEC            0xA0 

/*!
 * @brief Type definitions
 */
typedef uint32_t (*m24m02_com_fptr_write_t)(uint8_t dev_id, uint8_t *reg_addr, uint8_t *data);
typedef uint32_t (*m24m02_com_fptr_read_t) (uint8_t dev_id, uint8_t *reg_addr, uint8_t *data);

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

bool m24m02_eeprom_init(struct m24m02_dev *dev_init); 
bool m24m02_eeprom(uint8_t rw, uint32_t eepromAddress, uint8_t data_byte, uint8_t *infoArray);       // r/w mode ok
//bool m24m02_eeprom_clear_address_byte(uint32_t eepromAddress, uint8_t data_byte);

#ifdef __cplusplus
}
#endif

#endif // M24M02_H__
