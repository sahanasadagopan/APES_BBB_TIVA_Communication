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
#include "I2C_new.h"

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
#include "gesture_sensor.h"
#define LSM6DS3_ADDR        (0x6A)
#define TMP102_ADDR         (0x48)


// Global instance structure for the I2C master driver.
//tI2CMInstance g_sI2CInst;

// Demo Task declarations
//void demoI2CTask(void *pvParameters);
void demoSerialTask(void *pvParameters);

void gesture_sensor_interrupt_handler()
{
    /* clear pending interrupts on the sensor */
    /* any value written to the PICLEAR reg is good to clear interrupts */
   // uint8_t random_val=10;
   // gesture_sensor_write(PROXIMITY_INTERRUPT_CLEAR, random_val);

    /* clear pending interrupts locally */
    GPIOIntClear(GPIO_PORTL_BASE, GPIO_PIN_5);


    /* read from the proximity register */
    uint8_t pdata_val=0;
    gesture_sensor_read(PDATA_ADDRESS, &pdata_val);
    /* print value on the UART terminal */
    UARTprintf("\r\nvalue:0x%x", pdata_val);

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
    I2C_Init();

    /*uint8_t data_write=0x01;
    I2C_write(TMP102_ADDR,data_write);
    uint16_t datarecv;
    I2C_read(TMP102_ADDR,&datarecv);
    UARTprintf("\r\n1st value 0x%x",*((uint8_t*)&datarecv));
    UARTprintf("\r\n2nd value 0x%x",*((uint8_t*)&datarecv + 1));*/

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


    /* set a persistence of 10 cycles so that false interrupts are not triggered */

    uint8_t persistence_cycles=0xA0;
    gesture_sensor_write(PERSISTENCE_REG_ADDRESS, persistence_cycles);
    persistence_cycles=0;
    gesture_sensor_read(PERSISTENCE_REG_ADDRESS, &persistence_cycles);

    UARTprintf("\r\nPersistence cycles 0x%x",persistence_cycles);

    /* set upper and lower threshold values */

    uint8_t lower_threshold=0, upper_threshold=10;

    gesture_sensor_write(PIHT_REG_ADDRESS , upper_threshold);
    upper_threshold=0;
    gesture_sensor_read(PIHT_REG_ADDRESS, &upper_threshold);

    gesture_sensor_write(PILT_REG_ADDRESS , lower_threshold);

    gesture_sensor_read(PILT_REG_ADDRESS, &lower_threshold);

    UARTprintf("\r\nLower threshold 0x%x",lower_threshold);
    UARTprintf("\r\nUpper threshold 0x%x",upper_threshold);

    /* have a wait time between cycles */
    /* a wait time value of 171 will translate to 236 ms of wait time between read cycles */
    uint8_t wait_time=171;

    gesture_sensor_write(WAIT_TIME_REG_ADDRESS, wait_time);
    gesture_sensor_read(WAIT_TIME_REG_ADDRESS, &wait_time);

    UARTprintf("\r\nWait time 0x%x", wait_time);

    /* power on and enable the proximity sensor; also enable the proximity interrupt and wait time between cycles */

    uint8_t enable_proximity=0x2D;

    gesture_sensor_write(ENABLE_REG_ADDRESS, enable_proximity);

    gesture_sensor_read(ENABLE_REG_ADDRESS, &enable_proximity);

    UARTprintf("\r\nEnable reg value 0x%x", enable_proximity);

    /* wait for an interrupt here */
    while(1);

#ifdef GESTURE_MODE
    /*Gesture Sensor*/
    uint8_t enable_gesture_cmd=0x45;

    gesture_sensor_write(ENABLE_REG_ADDRESS, enable_gesture_cmd);

    uint8_t data;
    gesture_sensor_read(ENABLE_REG_ADDRESS, &data);
    UARTprintf("\r\nValue written to the enable reg:0x%x",data);

    /* do not exit out of the gesture engine again */
    //uint8_t stay_in_gesture_mode_forever=0;

    //gesture_sensor_write(EXIT_THRESHOLD_REG_ADDRESS, stay_in_gesture_mode_forever);

    //gesture_sensor_read(GESTURE_CONFIG_ONE_REG_ADDRESS, &data);
   //UARTprintf("\r\nValue at the gesture config one reg:0x%x",data);

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
    gesture_sensor_write(GESTURE_CONFIG4_REG_ADDRESS,gesture_mode);
    gesture_sensor_read(GESTURE_CONFIG4_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture MODE value 0x%x",data);
    //GCONFIG4<GMODE>

    /*uint8_t gesture_mode=0x01;
    gesture_sensor_write(GESTURE_MODE_REG_ADDRESS,gesture_mode);
    gesture_sensor_read(GESTURE_MODE_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture MODE value 0x%x",data);*/

    //gesture_sensor_read(GPENTH_ADDRESS, &data);
    //UARTprintf("\r\ngesture PLENGTH value 0x%x",data);
 //   uint8_t check_array[4][32];

    gesture_sensor_read(GESTURE_CONFIG3_REG_ADDRESS, &data);
    UARTprintf("\r\ngesture DIMEN value 0x%x",data);


    gesture_sensor_read(GESTURE_FIFO_LEVEL, &data);
    UARTprintf("\r\ngesture LEVEL value 0x%x",data);
    gesture_sensor_read(GESTURE_STATUS_ADDRESS, &data);
    UARTprintf("\r\ngesture STATUS value 0x%x",data);
    gesture_sensor_read(SENSOR_STATUS_REG, &data);
    UARTprintf("\r\ngesture Sensor automatic assert STATUS value 0x%x",data);
    uint8_t fifo_level=0, i;

    while(1)
    {
        SysCtlDelay(25000000);

        gesture_sensor_read(GESTURE_FIFO_LEVEL, &data);
        UARTprintf("\r\ngesture LEVEL value 0x%x",data);

        fifo_level=data;

        for(i=0;i<fifo_level;i++)
        {

          //  UARTprintf("\r\ngesture LEVEL FIFO value 0x%x",data);

            gesture_sensor_read(GESTURE_UP_REG_ADDRESS, &data);
            UARTprintf("\r\ngesture up value 0x%x",data);
           // check_array[0][i]=data;

            gesture_sensor_read(GESTURE_DOWN_REG_ADDRESS, &data);
            UARTprintf("\r\ngesture DOWN value 0x%x",data);
         //   check_array[1][i]=data;

            gesture_sensor_read(GESTURE_LEFT_REG_ADDRESS, &data);
            UARTprintf("\r\ngesture LEFT value 0x%x",data);
           // check_array[2][i]=data;

            gesture_sensor_read(GESTURE_RIGHT_REG_ADDRESS, &data);
            UARTprintf("\r\ngesture RIGHT value 0x%x",data);
            //check_array[3][i]=data;

            i++;
        }
    }
#endif

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
