/*
 * EEPROMdriver.h
 *
 *  Created on: Dec 8, 2017
 *      Author: asgu9779
 */

#ifndef EEPROMDRIVER_H_
#define EEPROMDRIVER_H_

#include<stdint.h>
#include<stdlib.h>
#include "I2C_new.h"
#include"gesture_sensor.h"
#include"driverlib/eeprom.h"



driver_rc eeprom_init();
driver_rc eeprom_program(uint32_t* data_array, uint32_t eeprom_address, uint32_t array_size);

/* wrapper to the library's EEPROMRead */
driver_rc eeprom_read(uint32_t* data_array, uint32_t eeprom_address, uint32_t array_size);




#endif /* EEPROMDRIVER_H_ */
