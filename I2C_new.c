
/*
 *  File        : I2C.c
 *
 *  Created on  : Dec 3, 2017
 *
 *  Description : High level I2C driver that calls the HAL i2c
 *                library functions- this file provides
 *                a more convenient abstraction.
 *
 *  Author      : Ashwath Gundepally, CU, ECEE
 *                Sahana Sadagopan, CU, ECEE
 *
 */
#include<stdlib.h>
#include<stdint.h>
#include<stdbool.h>
#include "driverlib/i2c.h"
#include "I2C_new.h"
#include "drivers/pinout.h"
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

void I2C_Init(void)
{
        // The I2C2 peripheral must be enabled before use.
        // On GPIO PortL
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

        //
        // Wait for the Peripheral to be ready for programming
        //
        while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));

        //
        // Configure the pin muxing for I2C2 functions on port L0 and L1.
        // This step is not necessary if your part does not support pin muxing.
        //
        ROM_GPIOPinConfigure(GPIO_PL1_I2C2SCL);
        ROM_GPIOPinConfigure(GPIO_PL0_I2C2SDA);

        //
        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        //
        ROM_GPIOPinTypeI2C(GPIO_PORTL_BASE, GPIO_PIN_0);
        GPIOPinTypeI2CSCL(GPIO_PORTL_BASE, GPIO_PIN_1);

        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));
        ROM_I2CMasterInitExpClk(I2C2_BASE,SYSTEM_CLOCK,true);
}
void I2C_write(uint8_t slave_address, uint8_t data_write){

    ROM_I2CMasterSlaveAddrSet(I2C2_BASE,slave_address,false);
    ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    ROM_I2CMasterDataPut(I2C2_BASE, data_write);
    while(ROM_I2CMasterBusy(I2C2_BASE));
}


void I2C_write_single(uint8_t slave_address, uint8_t data_write)
{

    ROM_I2CMasterSlaveAddrSet(I2C2_BASE,slave_address,false);
    ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_SINGLE_SEND);
    ROM_I2CMasterDataPut(I2C2_BASE, data_write);
    while(ROM_I2CMasterBusy(I2C2_BASE));

}

void I2C_write_temp(uint8_t slave_address, uint8_t data_write)
{

    ROM_I2CMasterSlaveAddrSet(I2C2_BASE,slave_address,false);
    ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
    ROM_I2CMasterDataPut(I2C2_BASE, data_write);
    while(ROM_I2CMasterBusy(I2C2_BASE));
}

driver_rc I2C_read_word(uint8_t slave_address, uint16_t* data_recv)
{
    if(data_recv==NULL)
        return DRIVER_I2C_READ_NULL_PTR;

    ROM_I2CMasterSlaveAddrSet(I2C2_BASE,slave_address,true);
    ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(ROM_I2CMasterBusy(I2C2_BASE));
    *((uint8_t*)data_recv) = ROM_I2CMasterDataGet(I2C2_BASE);
    ROM_I2CMasterSlaveAddrSet(I2C2_BASE,slave_address,true);
    ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(ROM_I2CMasterBusy(I2C2_BASE));
    *((uint8_t*)data_recv + 1) = ROM_I2CMasterDataGet(I2C2_BASE);

    return DRIVER_SUCCESS;
}


driver_rc I2C_read_byte(uint8_t slave_address, uint8_t* data_recv)
{
    if(data_recv==NULL)
        return DRIVER_I2C_READ_NULL_PTR;

    ROM_I2CMasterSlaveAddrSet(I2C2_BASE,slave_address,true);
    ROM_I2CMasterControl(I2C2_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(ROM_I2CMasterBusy(I2C2_BASE));
    *(data_recv) = ROM_I2CMasterDataGet(I2C2_BASE);

    return DRIVER_SUCCESS;
}
