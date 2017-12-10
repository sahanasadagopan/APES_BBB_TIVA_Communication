/*
 *  File        : EEPROMdriver.c
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
#include<stdlib.h>
#include<stdbool.h>
#include "I2C_new.h"
#include"gesture_sensor.h"
#include"driverlib/eeprom.h"
#include"EEPROMdriver.h"


#include "utils/uartstdio.h"

// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

driver_rc eeprom_init()
{
    /* lines 27-45 are borrowed from the example code given*/
    /* Enable the EEPROM module*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    /* Wait for the EEPROM module to be ready.*/
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
    {

    }

    /* Wait for the EEPROM Initialization to complete */
    uint32_t ui32EEPROMInit = EEPROMInit();

    /* check if eeprom init was successfull*/
    if(ui32EEPROMInit != EEPROM_INIT_OK)
    {
        return EEPROM_INIT_FAILED;
    }
    return DRIVER_SUCCESS;

}
/* wrapper to the library's EEPROMProgram */
driver_rc eeprom_program(uint32_t* data_array, uint32_t eeprom_address, uint32_t array_size)
{
    /* error handling */
    if(data_array==NULL)
        return EEPROM_PROGRAM_NULL_PTR;
    /* the address and the size values need to be multiples of four */
    if(eeprom_address%4!=0)
        return EEPROM_PROGRAM_INVALID_ADDRESS;
    if(array_size%4!=0)
        return EEPROM_PROGRAM_INVALID_SIZE;

    /* now it's safe to call the library API */
    uint32_t eeprom_program_return=EEPROMProgram(data_array, eeprom_address, array_size);

    /* check the return */
    if(eeprom_program_return!=0)
        return EEPROM_PROGRAM_FAILED;

    /* return successfully */
    return DRIVER_SUCCESS;
}

/* wrapper to the library's EEPROMRead */
driver_rc eeprom_read(uint32_t* data_array, uint32_t eeprom_address, uint32_t array_size)
{
       /* error handling */
       if(data_array==NULL)
           return EEPROM_READ_NULL_PTR;

       /* the address and the size values need to be multiples of four */
       if(eeprom_address%4!=0)
           return EEPROM_READ_INVALID_ADDRESS;
       if(array_size%4!=0)
           return EEPROM_READ_INVALID_SIZE;

       /* now it's safe to call the library API */
       EEPROMRead(data_array, eeprom_address, array_size);

       /* return successfully */
       return DRIVER_SUCCESS;
}
