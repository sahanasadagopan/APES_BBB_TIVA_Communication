/* FreeRTOS 8.2 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib source code is included.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"
#include "I2C_new.h"
#include "EEPROMdriver.h"
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
#include "semphr.h"
#include "gesture_sensor.h"
#include "messaging.h"

#define LSM6DS3_ADDR        (0x6A)
#define TMP102_ADDR         (0x48)


// Global instance structure for the I2C master driver.
//tI2CMInstance g_sI2CInst;

uint32_t g_ui32SysClock;
uint32_t g_ui32IPAddress;


// Demo Task declarations
void HeartBeatChecker(void *pvParameters);
void SensorTask(void *pvParameters);
void LCDTask(void *pvParameters);
void EEPROMTask(void *pvParameters);

SemaphoreHandle_t UartProtector;

TaskHandle_t TaskHandle[TOTAL_NUMBER_OF_TASKS];

QueueHandle_t QueueHandle[TOTAL_NUMBER_OF_TASKS-1];
uint32_t output_clock_rate_hz;

/*void UART0Init(void){
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_UARTConfigSetExpClk(UART0_BASE, output_clock_rate_hz, 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));
}*/

void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    char data=0;
    int i=0;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART3_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART3_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    //while(ROM_UARTCharsAvail(UART3_BASE))
    //{
        //
        // Read the next character from the UART and write it back to the UART.
        //
        data=ROM_UARTCharPutNonBlocking(UART3_BASE, 'U');

        UARTprintf("%c", data);
        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(output_clock_rate_hz / (1000 * 3));



        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
   // }

}

void
UARTSend(char* byte, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //

    //
    // Write the next character to the UART.
    //
    while(ui32Count--)
    {
        ROM_UARTCharPut(UART3_BASE, *byte++);
    }
}

// Main function
int main(void)
{

    // Initialize system clock to 120 MHz
    output_clock_rate_hz= ROM_SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);

    // Set up the UART which is connected to the virtual COM port
   // UARTStdioConfig(0, 57600, SYSTEM_CLOCK);

    // Initialize the GPIO pins for the Launchpad
    //PinoutSet(false, false);
    //UARTprintf("sent\n\r");
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Enable the GPIO pins for the LED (PN0).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);


    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA4_U3RX);
    GPIOPinConfigure(GPIO_PA5_U3TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART3_BASE, output_clock_rate_hz, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable processor interrupts.
    //
    /*ROM_IntMasterEnable();

    UARTIntRegister(UART3_BASE, UARTIntHandler);


    //
    // Configure SysTick for a periodic interrupt.
    //
    ROM_IntEnable(INT_UART3);
    ROM_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
*/
    //
    // Prompt for text to be entered.
    //
    //UARTSend((uint8_t *)"t", 2);

    //
    // Loop forever echoing data through the UART.
    //
    char string_test[6];
    strcpy(string_test, "hello");

    while(1)
    {
        UARTSend("U", 1);
 //       UARTprintf("sent\n\r");

    }
    /*BaseType_t xTaskCreateReturn;

    UartProtector=xSemaphoreCreateMutex();*/

    /* Create all the tasks required */
    /* send the heart beat task's handle info as an argument */
    /*xTaskCreateReturn=xTaskCreate(HeartBeatChecker, (const portCHAR *)"HeartBeat",
                   configMINIMAL_STACK_SIZE, NULL, 1, &TaskHandle[HEARTBEAT_TASK] );

    UARTprintf("\r\nHB handle val:%d", TaskHandle[HEARTBEAT_TASK]);

    if(xTaskCreateReturn==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
        UARTprintf("\r\nHB task creation failed");

    xTaskCreateReturn=xTaskCreate(SensorTask, (const portCHAR *)"Sensor",
                configMINIMAL_STACK_SIZE, NULL, 2, &TaskHandle[SENSOR_TASK]);

    if(xTaskCreateReturn==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
            UARTprintf("\r\nSensor task creation failed");

    xTaskCreateReturn=xTaskCreate(LCDTask, (const portCHAR *)"LCD",
                configMINIMAL_STACK_SIZE, NULL, 2, &TaskHandle[LCD_TASK]);

    if(xTaskCreateReturn==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
            UARTprintf("\r\nLCD task creation failed");


    xTaskCreateReturn=xTaskCreate(EEPROMTask, (const portCHAR *)"EEPROM",
                configMINIMAL_STACK_SIZE, NULL, 2, &TaskHandle[EEPROM_TASK]);

    if(xTaskCreateReturn==errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
            UARTprintf("\r\nEEPROM task creation failed");*/


    /* create Queues for all the above defined tasks except the heart beat task */
    /*QueueHandle[SENSOR_TASK]=xQueueCreate(QUEUE_LENGTH, sizeof(InterTaskPacket_t));*/

    /* error checking */
    /*if( QueueHandle[SENSOR_TASK]== NULL)
        UARTprintf("\r\nSensor Queue creation failed");*/

    //QueueHandle[LCD_TASK]=xQueueCreate(QUEUE_LENGTH, sizeof(InterTaskPacket_t));

    /* error checking */
    /*if( QueueHandle[LCD_TASK]== NULL)
        UARTprintf("\r\nLCD Queue creation failed");

    QueueHandle[EEPROM_TASK]=xQueueCreate(QUEUE_LENGTH, sizeof(InterTaskPacket_t));*/

    /* error checking */
    /*if( QueueHandle[EEPROM_TASK]== NULL)
        UARTprintf("\r\nEEPROM Queue creation failed");


    vTaskStartScheduler();*/

    return 0;
}

// Flash the LEDs on the launch pad
void HeartBeatChecker(void *pvParameters)
{
    uint32_t HeartbeatReceived;
    BaseType_t NotifyWaitReturn;

    /* get heart beats from the tasks created in this loop */
    while(1)
    {
        vTaskDelay(10000);

        /* clear the bits that need to be set by other tasks on exit and get the value written by other tasks */
        NotifyWaitReturn=xTaskNotifyWait( 0, LCD_TASK_HEARTBEAT|SENSOR_TASK_HEARTBEAT|EEPROM_TASK_HEARTBEAT,
                                  &HeartbeatReceived, pdMS_TO_TICKS(10000) );

        /* Now check if the bits corresponding to each task are set */
        if(!(HeartbeatReceived&LCD_TASK_HEARTBEAT))
        {
            /* let it wait indefinitely until it gets the semaphore;
             * INCLUDE_vTaskSuspend is set to 1 in the FreeRTOSConfig.h file
             * */
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            UARTprintf("\r\nHB from LCD missed");
            xSemaphoreGive(UartProtector);
        }

        if(!(HeartbeatReceived&SENSOR_TASK_HEARTBEAT))
        {
           /* let it wait indefinitely until it gets the semaphore;
            * INCLUDE_vTaskSuspend is set to 1 in the FreeRTOSConfig.h file
            * */
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            UARTprintf("\r\nHB from Sensor missed");
            xSemaphoreGive(UartProtector);
        }
        if(!(HeartbeatReceived&EEPROM_TASK_HEARTBEAT))
        {
           /* let it wait indefinitely until it gets the semaphore;
            * INCLUDE_vTaskSuspend is set to 1 in the FreeRTOSConfig.h file
            * */
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            UARTprintf("\r\nHB from EEPROM missed");
            xSemaphoreGive(UartProtector);
        }

        /* no task is alive */
        if(NotifyWaitReturn==pdFALSE)
        {
           /* let it wait indefinitely until it gets the semaphore;
            * INCLUDE_vTaskSuspend is set to 1 in the FreeRTOSConfig.h file
            * */
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            UARTprintf("\r\nTimed out:Didn't receive anything, notification val:%d", HeartbeatReceived);
            xSemaphoreGive(UartProtector);

            /* TODO: Write to the logger here  or set a flag or something which controls log data or data that's sent to the BBG */

        }
        else
        {
           /* let it wait indefinitely until it gets the semaphore;
            *  INCLUDE_vTaskSuspend is set to 1 in the FreeRTOSConfig.h file
            * */
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            UARTprintf("\r\nNotification val:%d", HeartbeatReceived);
            xSemaphoreGive(UartProtector);
        }

    }
}

/* just flashing some LEDs for now */
void SensorTask(void *pvParameters)
{
    TaskHandle_t HeartBeatHandle=TaskHandle[HEARTBEAT_TASK];

    //BaseType_t TaskNotifyReturn;

    xSemaphoreTake( UartProtector, portMAX_DELAY);
    UARTprintf("\r\nHB handle val in SensorTask:%d", HeartBeatHandle);
    xSemaphoreGive(UartProtector);

    for (;;)
    {
        xSemaphoreTake( UartProtector, portMAX_DELAY);
        /* debug message to the UART logger */
        UARTprintf("\r\nSent HB from SensorTask");
        xSemaphoreGive(UartProtector);

        vTaskDelay(2000);
        /* notify the HeartBeat task of your well being */
        /* OR the notification value with the value that represents this task's heart beat using the esetBits argument */
        /* no point in checking the return from this function as it's always going to return successfully */
        xTaskNotify(HeartBeatHandle, SENSOR_TASK_HEARTBEAT, eSetBits);

        /*send req for LCD data */
        if(SendLCDCharsReq(SENSOR_TASK)!=pdPASS)
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\nsend LCD  req failed ");
            xSemaphoreGive(UartProtector);
        }

        /* read queue for response now */
        if(ReadQueueSensor()!=pdPASS)
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\nread sensor queue failed ");
            xSemaphoreGive(UartProtector);
        }


    }
}

/* just writing to UART for now */
void LCDTask(void *pvParameters)
{
    /* */
    TaskHandle_t HeartBeatHandle=TaskHandle[HEARTBEAT_TASK];

    xSemaphoreTake(UartProtector, portMAX_DELAY);
    UARTprintf("\r\nHB handle val in LCDTask:%d", HeartBeatHandle);
    xSemaphoreGive(UartProtector);


    for (;;)
    {
        xSemaphoreTake( UartProtector, portMAX_DELAY);
        UARTprintf("\r\nSent HB from LCD");
        xSemaphoreGive(UartProtector);

        vTaskDelay(2000);

        /* notify the HeartBeat task of your well being */
        /* OR the notification value with the value that represents this task's heart beat using the esetBits argument */
        /* no point in checking the return from this function as it's always going to return successfully */
        xTaskNotify(HeartBeatHandle, LCD_TASK_HEARTBEAT, eSetBits);

        /*send req for LCD data */
        if(SendProximityDataReq(LCD_TASK)!=pdPASS)
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\nsend Pdata  req failed ");
            xSemaphoreGive(UartProtector);
        }

        /* read queue for response now */
        if(ReadQueueLCD()!=pdPASS)
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\nread LCD queue failed ");
            xSemaphoreGive(UartProtector);
        }

    }
}

/* reading and writing to addresses for now */
void EEPROMTask(void *pvParameters)
{
    TaskHandle_t HeartBeatHandle=TaskHandle[HEARTBEAT_TASK];

    //BaseType_t TaskNotifyReturn;

    xSemaphoreTake( UartProtector, portMAX_DELAY);
    UARTprintf("\r\nHB handle val in EEPROM Task:%d", HeartBeatHandle);
    xSemaphoreGive(UartProtector);
    uint32_t eeprom_val=0, eeprom_val_read=0;
    uint32_t addr=0x400;

    /* initialize the EEPROM */
    driver_rc eprom_init_return =eeprom_init();

    if(eprom_init_return!=DRIVER_SUCCESS)
    {
        xSemaphoreTake( UartProtector, portMAX_DELAY);
        UARTprintf("\r\nInitialization failed");
        xSemaphoreGive(UartProtector);
    }

    for (;;)
    {
        xSemaphoreTake( UartProtector, portMAX_DELAY);
        /* debug message to the UART logger */
        UARTprintf("\r\nSent HB from EEPROMTask");
        xSemaphoreGive(UartProtector);

        vTaskDelay(2000);

        /* notify the HeartBeat task of your well being */
        /* OR the notification value with the value that represents this task's heart beat using the esetBits argument */
        /* no point in checking the return from this function as it's always going to return successfully */
        xTaskNotify(HeartBeatHandle, EEPROM_TASK_HEARTBEAT, eSetBits);

        /*send req for LCD data */
        if(SendLCDCharsReq(EEPROM_TASK)!=pdPASS)
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\nsend LCD  req failed ");
            xSemaphoreGive(UartProtector);
        }

        driver_rc eeprom_driver_return=eeprom_program(&eeprom_val, addr, 4);

        if(eeprom_driver_return!=DRIVER_SUCCESS)
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\n eeprom write faield");
            xSemaphoreGive(UartProtector);
        }
        else
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\n val written:%d", eeprom_val);
            xSemaphoreGive(UartProtector);
        }
        eeprom_val_read=0;
        /* read from the same address now */
        eeprom_driver_return=eeprom_read(&eeprom_val_read, addr, 4);

        if(eeprom_driver_return!=DRIVER_SUCCESS)
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\n eeprom write faield");
            xSemaphoreGive(UartProtector);
        }
        else
        {
            xSemaphoreTake( UartProtector, portMAX_DELAY);
            /* debug message to the UART logger */
            UARTprintf("\r\n val read:%d", eeprom_val_read);
            xSemaphoreGive(UartProtector);
        }

        /* write something different now */
        eeprom_val=eeprom_val+1;
        addr=addr+4;
    }
}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}

#ifdef PROXIMITY_STUFF

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
#endif

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

}
#endif

