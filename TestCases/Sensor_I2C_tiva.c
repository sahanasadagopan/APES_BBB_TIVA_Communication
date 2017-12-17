/* FreeRTOS 8.2 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
 */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"
#include "I2C.h"

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

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define LSM6DS3_ADDR        (0x6A)
#define TMP102_ADDR         (0x48)


void demoSerialTask(void *pvParameters);

int I2C_Init(void)
{
    
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

        return 0;
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
    //UARTprintf("\r\n1st value 0x%x",*data_recv);
    return DRIVER_SUCCESS;
}

driver_rc gesture_sensor_write(uint8_t reg_address, uint8_t data)
{
    /**/
    I2C_write(SLAVE_ADDR_GESTURE,reg_address);

    I2C_write_temp(SLAVE_ADDR_GESTURE,data);

    return DRIVER_SUCCESS;

}

driver_rc gesture_sensor_read(uint8_t reg_address, uint8_t* data)
{
    I2C_write_single(SLAVE_ADDR_GESTURE,reg_address);

    I2C_read_byte(SLAVE_ADDR_GESTURE, data);

    return DRIVER_SUCCESS;

}

// Main function
int main(void)
{
    uint32_t output_clock_rate_hz;
    output_clock_rate_hz=ROM_SysCtlClockFreqSet(
            (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
             SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
            SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);
    // Initialize the GPIO pins for the Launchpad
   PinoutSet(false, false);
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);
    UARTprintf("\n\rconfig reg is");
    int busy=I2C_Init();
    


    /*proximity sensor*/
    gesture_sensor_write(ENABLE_REG_ADDRESS, 5);
    uint8_t data;
    gesture_sensor_read(ENABLE_REG_ADDRESS, &data);

    UARTprintf("\r\n1st value 0x%x",data);

    data=0;
    gesture_sensor_read(PDATA_ADDRESS, &data);
    UARTprintf("\r\n pdata value 0x%x",data);

    /*Gesture Sensor*/
    uint8_t enable_gesture_cmd=0x45;
    gesture_sensor_write(ENABLE_REG_ADDRESS,enable_gesture_cmd);
    uint8_t data;
    gesture_sensor_read(ENABLE_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture value 0x%x",data);
    //GCONFIG4<GMODE>

    

    gesture_sensor_read(GESTURE_UP_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture up value 0x%x",data);

    gesture_sensor_read(GESTURE_DOWN_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture DOWN value 0x%x",data);

    gesture_sensor_read(GESTURE_DIMEN_ADDRESS, &data);
    UARTprintf("\r\ngesture DIMEN value 0x%x",data);

    gesture_sensor_read(GESTURE_LEFT_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture LEFT value 0x%x",data);

    gesture_sensor_read(GESTURE_RIGHT_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture RIGHT value 0x%x",data);

    gesture_sensor_read(GESTURE_FIFO_LEVEL, &data);
    UARTprintf("\r\ngesture LEVEL value 0x%x",data);
}


void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
