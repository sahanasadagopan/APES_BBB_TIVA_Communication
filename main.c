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


// Global instance structure for the I2C master driver.
//tI2CMInstance g_sI2CInst;

// Demo Task declarations
//void demoI2CTask(void *pvParameters);
void demoSerialTask(void *pvParameters);

int I2C_Init(void)
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
void gesture_sensor_interrupt_handler()
{
    GPIOIntClear(GPIO_PORTL_BASE, GPIO_PIN_5);
    UARTprintf("\n\r u");

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

   // Create demo task

  // xTaskCreate(demoSerialTask, (const portCHAR *)"Serial",
    //           configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // vTaskStartScheduler();
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);
    UARTprintf("\n\rconfig reg is");
    int busy=I2C_Init();
    /*uint8_t data_write=0x01;
    I2C_write(TMP102_ADDR,data_write);
    uint16_t datarecv;
    I2C_read(TMP102_ADDR,&datarecv);
    UARTprintf("\r\n1st value 0x%x",*((uint8_t*)&datarecv));
    UARTprintf("\r\n2nd value 0x%x",*((uint8_t*)&datarecv + 1));*/



    /*proximity sensor*/
   /* gesture_sensor_write(ENABLE_REG_ADDRESS, 5);
    uint8_t data;
    gesture_sensor_read(ENABLE_REG_ADDRESS, &data);

    UARTprintf("\r\n1st value 0x%x",data);

    data=0;
    gesture_sensor_read(PDATA_ADDRESS, &data);
    UARTprintf("\r\n pdata value 0x%x",data);*/

    /* interrupts stuff */

    /* first configure the pin as input */
    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, GPIO_PIN_5);

    /* start by enabling the master interrupt */
    IntMasterEnable();

    /* define the handler for the GPIO interrupt */
    GPIOIntRegister(GPIO_PORTL_BASE, gesture_sensor_interrupt_handler);

    /* set the type of interrupt */
    GPIOIntTypeSet(GPIO_PORTL_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);

    /* then enable interrupts for PIN 5 on Port L */
    GPIOIntEnable(GPIO_PORTL_BASE, GPIO_INT_PIN_5);

    /*Gesture Sensor*/
    uint8_t enable_gesture_cmd=0x45;

    gesture_sensor_write(ENABLE_REG_ADDRESS, enable_gesture_cmd);

    uint8_t data;
    gesture_sensor_read(ENABLE_REG_ADDRESS, &data);
    UARTprintf("\r\nValue written to the enable reg:0x%x",data);

    /* do not exit out of the gesture engine again */
    uint8_t stay_in_gesture_mode_forever=0;

    gesture_sensor_write(EXIT_THRESHOLD_REG_ADDRESS, stay_in_gesture_mode_forever);

    gesture_sensor_read(GESTURE_CONFIG_ONE_REG_ADDRESS, &data);
    UARTprintf("\r\nValue at the gesture config one reg:0x%x",data);

    /* OR this value with the FIFO threshold level value */
    /* set it to three for 16 data-sets to be stored in the FIFO */
    uint8_t fifo_thresh_val=3;
    /* seeing as how this value needs to be written to bits 6:7, it requires to be left shifted by 6 */
    data=data|(fifo_thresh_val<<6);

    gesture_sensor_write(GESTURE_CONFIG_ONE_REG_ADDRESS, data);
    UARTprintf("\r\nValue OR'd with the byte read: 0x%x", fifo_thresh_val<<6);

    UARTprintf("\r\nValue written to the config one reg: 0x%x",data);

    gesture_sensor_read(GESTURE_CONFIG_ONE_REG_ADDRESS, &data);

    UARTprintf("\r\nValue at the gesture config one reg post writing to it:0x%x",data);

    uint8_t gesture_mode=0x03;
    gesture_sensor_write(GESTURE_MODE_REG_ADDRESS,gesture_mode);
    gesture_sensor_read(GESTURE_MODE_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture MODE value 0x%x",data);
    //GCONFIG4<GMODE>

    /*uint8_t gesture_mode=0x01;
    gesture_sensor_write(GESTURE_MODE_REG_ADDRESS,gesture_mode);
    gesture_sensor_read(GESTURE_MODE_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture MODE value 0x%x",data);*/

    //gesture_sensor_read(GPENTH_ADDRESS, &data);
    //UARTprintf("\r\ngesture PLENGTH value 0x%x",data);

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
    while(1){
        SysCtlDelay(1000);
        gesture_sensor_read(GESTURE_FIFO_LEVEL, &data);
        UARTprintf("\r\ngesture LEVEL value 0x%x",data);
        gesture_sensor_read(GESTURE_STATUS_ADDRESS, &data);
        UARTprintf("\r\ngesture STATUS value 0x%x",data);


    }
}

/*void demoSerialTask(void *pvParameters)
{
    // Set up the UART which is connected to the virtual COM port
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);


    for (;;)
    {
        UARTprintf("\r\nHello, world from FreeRTOS 9.0!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}*/

void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
