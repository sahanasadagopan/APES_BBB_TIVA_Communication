/*
 * gesture_sensor.h
 *
 *  Created on: Dec 3, 2017
 *      Author: asgu9779
 */

#ifndef GESTURE_SENSOR_H_
#define GESTURE_SENSOR_H_

/* gesture sensor register address */
#define SENSOR_STATUS_REG                   0X93
#define ENABLE_REG_ADDRESS                  0x80
#define GESTURE_CONFIG4_REG_ADDRESS         0xAB
#define GESTURE_UP_REG_ADDRESS              0xFC
#define GESTURE_DOWN_REG_ADDRESS            0xFD
#define GESTURE_RIGHT_REG_ADDRESS           0xFE
#define GESTURE_LEFT_REG_ADDRESS            0xFF
#define PDATA_ADDRESS                       0x9C
#define GESTURE_CONFIG3_REG_ADDRESS         0xAA
#define GPENTH_ADDRESS                      0xA0
#define GESTURE_FIFO_LEVEL                  0xAE
#define GESTURE_STATUS_ADDRESS              0XAF
#define EXIT_THRESHOLD_REG_ADDRESS          0xA1
#define GESTURE_CONFIG_ONE_REG_ADDRESS      0xA2
#define PERSISTENCE_REG_ADDRESS             0x8C
#define PROXIMITY_INTERRUPT_CLEAR           0xE5
#define FORCE_INTERRUPT                     0xE4
#define WAIT_TIME_REG_ADDRESS               0x83

/* proximity low and high threshold registers */
#define PILT_REG_ADDRESS                    0x89
#define PIHT_REG_ADDRESS                    0x8B

#include<stdint.h>
/*
 *  function        :   gesture_sensor_write(uint8_t reg_address, uint8_t data)
 *
 *  Description     :   Function writes data to the gesture sensor given the
 *                      register's address and a uint8_t data type
 *
 */

void gesture_sensor_write(uint8_t reg_address, uint8_t data);

/*
 *  function        :   gesture_sensor_read(uint8_t reg_address, uint8_t* data)
 *
 *  Description     :   Function reads data from the gesture sensor given the
 *                      register's address and a pointer to a uint8_t
 *
 *  Returns         :   DRIVER_SUCCESS: Complete's execution successfully
 *                      DRIVER_I2C_NULL_PTR: If the ptr to uint8_t is a null
 *
 */
driver_rc gesture_sensor_read(uint8_t reg_address, uint8_t* data);



#endif /* GESTURE_SENSOR_H_ */
