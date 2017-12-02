/*
 * I2C.h
 *
 *  Created on: Dec 1, 2017
 *      Author: sahan
 */

#ifndef I2C_H_
#define I2C_H_

#define SLAVE_ADDR_GESTURE 0X39
#include "driverlib/i2c.h"


#define SYSTEM_CLOCK    120000000U

typedef enum {DRIVER_SUCCESS, DRIVER_I2C_READ_NULL_PTR, I2C_FAILURE}driver_rc;

/* read a 16-bit value from the sensor */
driver_rc I2C_read_word(uint8_t slave_address, uint16_t* data_recv);


/*gesture stuff */
#define SENSOR_STATUS_REG                   0X93
#define ENABLE_REG_ADDRESS                  0x80
#define GESTURE_MODE_REG_ADDRESS            0xAB
#define GESTURE_UP_REG_ADDRESS              0xFC
#define GESTURE_DOWN_REG_ADDRESS            0xFD
#define GESTURE_RIGHT_REG_ADDRESS           0xFE
#define GESTURE_LEFT_REG_ADDRESS            0xFF
#define PDATA_ADDRESS                       0x9C
#define GESTURE_DIMEN_ADDRESS               0xAA
#define GPENTH_ADDRESS                      0xA0
#define GESTURE_FIFO_LEVEL                  0xAE
#define GESTURE_STATUS_ADDRESS              0XAF
#define EXIT_THRESHOLD_REG_ADDRESS          0xA1
#define GESTURE_CONFIG_ONE_REG_ADDRESS      0xA2

#endif /* I2C_H_ */
