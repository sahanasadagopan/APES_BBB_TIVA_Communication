/*
 * I2C.h
 *
 *  Created on: Dec 1, 2017
 *      Author: sahan
 */

#ifndef I2C_NEW_H_
#define I2C_NEW_H_
#include<stdint.h>

#define SYSTEM_CLOCK    120000000U


/* slave address of the gesture sensor */
#define SLAVE_ADDR_GESTURE 0X39


typedef enum {DRIVER_SUCCESS, DRIVER_I2C_READ_NULL_PTR, I2C_FAILURE, EEPROM_PROGRAM_NULL_PTR, EEPROM_PROGRAM_INVALID_ADDRESS, EEPROM_PROGRAM_INVALID_SIZE, EEPROM_PROGRAM_FAILED,
    EEPROM_READ_NULL_PTR, EEPROM_READ_INVALID_ADDRESS, EEPROM_READ_INVALID_SIZE, EEPROM_READ_FAILED, EEPROM_INIT_FAILED}driver_rc;

/*TODO: Add function comments */
void I2C_Init(void);

/* read a 16-bit value from the sensor */
driver_rc I2C_read_word(uint8_t slave_address, uint16_t* data_recv);

void I2C_write(uint8_t slave_address, uint8_t data_write);

void I2C_write_single(uint8_t slave_address, uint8_t data_write);

void I2C_write_temp(uint8_t slave_address, uint8_t data_write);


driver_rc I2C_read_byte(uint8_t slave_address, uint8_t* data_recv);


#endif /* I2C_NEW_H_ */
