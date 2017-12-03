/*
 *  File        : gesture_sensor.c
 *
 *  Created on  : Dec 3, 2017
 *
 *  Description : Gesture sensor driver that calls
 *                the low level I2C functions.
 *
 *
 *  Author      : Ashwath Gundepally, CU, ECEE
 *                Sahana Sadagopan, CU, ECEE
 *
 */
#include<stdint.h>
#include "I2C_new.h"
#include"gesture_sensor.h"

/*TODO: Maybe add macros that read and write to a register for each register */


/*
 *  function        :   gesture_sensor_write(uint8_t reg_address, uint8_t data)
 *
 *  Description     :   Function writes data to the gesture sensor given the
 *                      register's address and a uint8_t data type
 *
 */
void gesture_sensor_write(uint8_t reg_address, uint8_t data)
{
    /* gain access to the register in question */
    I2C_write(SLAVE_ADDR_GESTURE,reg_address);

    /* just do a single byte-write now that the
     * byte is definitely going to be written to the given address */
    I2C_write_temp(SLAVE_ADDR_GESTURE,data);
}

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
driver_rc gesture_sensor_read(uint8_t reg_address, uint8_t* data)
{
    /* do a single write to gain access to a register */
    I2C_write_single(SLAVE_ADDR_GESTURE,reg_address);

    /* now just do a plain I2C read */
    driver_rc i2c_read_byte_rc;
    i2c_read_byte_rc=I2C_read_byte(SLAVE_ADDR_GESTURE, data);

    /* error handling */
    if(i2c_read_byte_rc!=DRIVER_SUCCESS)
        return i2c_read_byte_rc;

    return DRIVER_SUCCESS;

}
