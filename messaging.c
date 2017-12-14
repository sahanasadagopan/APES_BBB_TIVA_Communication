/*
 *  File        :   messaging.c
 *
 *  Description :   Has routines that deal with messaging among threads
 *
 *  Created on  :   Dec 5, 2017
 *
 *      Author  :   Ashwath Gundepally, CU, ECEE
 *                  Sahana Sadagopan, CU, ECEE
 *
 */
#include "FreeRTOS.h"
#include "messaging.h"
#include "main.h"
#include "queue.h"
#include "utils/uartstdio.h"
#include "semphr.h"
#include "gesture_sensor.h"


/* sends a request command to the Sensor Task's queue for data */
BaseType_t SendProximityDataReq(TaskID SenderTaskID)
{
    /* create packet that needs to be send */
    InterTaskPacket_t PDataRequest=
    {
        .MessageType=REQUEST_MESSAGE,
        .MessageBody.MessageRequest.SenderTaskID=SenderTaskID,
    };

    BaseType_t QueueSendReturn;
    /* Send this packet to the Sensor Task's Queue */
    QueueSendReturn=xQueueSend( QueueHandle[SENSOR_TASK], &PDataRequest, pdMS_TO_TICKS(4000));

    return QueueSendReturn;

}

/* sends a request command to the LCD Task's queue for the number of chars currently on display */
BaseType_t SendLCDCharsReq(TaskID SenderTaskID)
{
    /* create packet that needs to be send */
    InterTaskPacket_t LCDCharsRequest=
    {
       .MessageType=REQUEST_MESSAGE,
       .MessageBody.MessageRequest.SenderTaskID=SenderTaskID,
    };

    BaseType_t QueueSendReturn;

    /* Send this packet to the Sensor Task's Queue */
    QueueSendReturn=xQueueSend( QueueHandle[LCD_TASK], &LCDCharsRequest, pdMS_TO_TICKS(4000));

    return QueueSendReturn;
}

/* sends a request command to the EEPROM Task's queue for the address counter */
void SendEEPROMAddressReq()
{

}

/* Read the LCD Task's queue for messages sent and received
 * And send response if required */
BaseType_t ReadQueueLCD()
{
    InterTaskPacket_t ReceivedLCDMessage;

    uint8_t lcd_position=55;

    /*read from the LCD's queue */
    BaseType_t QueueReceiveLCDReturn=xQueueReceive( QueueHandle[LCD_TASK], &ReceivedLCDMessage, portMAX_DELAY);

    /* if the read is not successful, return */
    if(QueueReceiveLCDReturn!=pdPASS)
        return QueueReceiveLCDReturn;

    BaseType_t QueueSendReturn=pdPASS;

    /* if it's a response, print it over UART. If it's a request, send response to requester */
    if(ReceivedLCDMessage.MessageType==RESPONSE_MESSAGE)
    {
        /* Find out who sent the response */
        if(ReceivedLCDMessage.MessageBody.MessageResponse.SenderTaskID==EEPROM_TASK)
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nEEPROM Address:%x", ReceivedLCDMessage.MessageBody.MessageResponse.DataType.EepromAddress);
            xSemaphoreGive(UartProtector);
        }
        else if(ReceivedLCDMessage.MessageBody.MessageResponse.SenderTaskID==SENSOR_TASK)
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nSensor Data:%x", ReceivedLCDMessage.MessageBody.MessageResponse.DataType.ProximityData);
            xSemaphoreGive(UartProtector);
        }
        else
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nInvalid task ID with response");
            xSemaphoreGive(UartProtector);
        }
    }
    else if(ReceivedLCDMessage.MessageType==REQUEST_MESSAGE)
    {
        /* send a response to this request */
        /* create packet that needs to be send */
        InterTaskPacket_t LCDDataResponse=
        {
            .MessageType=RESPONSE_MESSAGE,
            .MessageBody.MessageResponse.SenderTaskID=LCD_TASK,
            .MessageBody.MessageResponse.DataType.LCDPosition=lcd_position,
        };

        /* Find out who sent the request and send the response to their queue */
        if(ReceivedLCDMessage.MessageBody.MessageResponse.SenderTaskID==EEPROM_TASK)
        {
            QueueSendReturn=xQueueSendToFront( QueueHandle[EEPROM_TASK], &LCDDataResponse, portMAX_DELAY);
        }
        else if(ReceivedLCDMessage.MessageBody.MessageResponse.SenderTaskID==SENSOR_TASK)
        {
            QueueSendReturn=xQueueSendToFront( QueueHandle[SENSOR_TASK], &LCDDataResponse, portMAX_DELAY);
        }
        else
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nInvalid task ID with response");
            xSemaphoreGive(UartProtector);
        }
    }
    return QueueSendReturn;
}
void proximity_data(uint8_t pdata_val)
{
    //uint32_t output_clock_rate_hz;

    /*output_clock_rate_hz=ROM_SysCtlClockFreqSet(
            (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
             SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
            SYSTEM_CLOCK);

    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);
    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    // Set up the UART which is connected to the virtual COM port
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);*/

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
    //IntMasterEnable();

    /* define the handler for the GPIO interrupt */
    //GPIOIntRegister(GPIO_PORTL_BASE, gesture_sensor_interrupt_handler);

    /* set the type of interrupt */
    //GPIOIntTypeSet(GPIO_PORTL_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);

    /* then enable interrupts for PIN 5 on Port L */
    //GPIOIntEnable(GPIO_PORTL_BASE, GPIO_INT_PIN_5);


    /* set a persistence of 10 cycles so that false interrupts are not triggered */

    /*uint8_t persistence_cycles=0xA0;
    gesture_sensor_write(PERSISTENCE_REG_ADDRESS, persistence_cycles);
    persistence_cycles=0;
    gesture_sensor_read(PERSISTENCE_REG_ADDRESS, &persistence_cycles);

    UARTprintf("\r\nPersistence cycles 0x%x",persistence_cycles);*/

    /* set upper and lower threshold values */

    /*uint8_t lower_threshold=0, upper_threshold=10;

    gesture_sensor_write(PIHT_REG_ADDRESS , upper_threshold);
    upper_threshold=0;
    gesture_sensor_read(PIHT_REG_ADDRESS, &upper_threshold);

    gesture_sensor_write(PILT_REG_ADDRESS , lower_threshold);

    gesture_sensor_read(PILT_REG_ADDRESS, &lower_threshold);

    UARTprintf("\r\nLower threshold 0x%x",lower_threshold);
    UARTprintf("\r\nUpper threshold 0x%x",upper_threshold);*/

    /* have a wait time between cycles */
    /* a wait time value of 171 will translate to 236 ms of wait time between read cycles */
    /*uint8_t wait_time=171;

    gesture_sensor_write(WAIT_TIME_REG_ADDRESS, wait_time);
    gesture_sensor_read(WAIT_TIME_REG_ADDRESS, &wait_time);

    UARTprintf("\r\nWait time 0x%x", wait_time);*/

    /* power on and enable the proximity sensor; also enable the proximity interrupt and wait time between cycles */

    uint8_t enable_proximity=0x05;

    gesture_sensor_write(ENABLE_REG_ADDRESS, enable_proximity);

    gesture_sensor_read(ENABLE_REG_ADDRESS, &enable_proximity);

    UARTprintf("\r\nEnable reg value 0x%x", enable_proximity);
    /* read from the proximity register */
   // uint8_t pdata_val;
   // while(1){
         gesture_sensor_read(PDATA_ADDRESS, &pdata_val);
        /* print value on the UART terminal */
        UARTprintf("\r\nvalue:0x%x", pdata_val);
   // }

}


/* Read the Sensor Task's queue for messages sent and received */
BaseType_t ReadQueueSensor()
{
    InterTaskPacket_t ReceivedSensorMessage;

    uint8_t proximity_value=0;

    proximity_data(proximity_value);

    /*read from the LCD's queue */
    BaseType_t QueueReceiveSensorReturn=xQueueReceive( QueueHandle[SENSOR_TASK], &ReceivedSensorMessage, portMAX_DELAY);

    /* if the read is not successful, return */
    if(QueueReceiveSensorReturn!=pdPASS)
        return QueueReceiveSensorReturn;

    BaseType_t QueueSendReturn=pdPASS;

    /* if it's a response, print it over UART. If it's a request, send response to requester */
    if(ReceivedSensorMessage.MessageType==RESPONSE_MESSAGE)
    {
        /* Find out who sent the response */
        if(ReceivedSensorMessage.MessageBody.MessageResponse.SenderTaskID==EEPROM_TASK)
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nEEPROM Address:%x", ReceivedSensorMessage.MessageBody.MessageResponse.DataType.EepromAddress);
            xSemaphoreGive(UartProtector);
        }
        else if(ReceivedSensorMessage.MessageBody.MessageResponse.SenderTaskID==LCD_TASK)
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nLCD Position:%x", ReceivedSensorMessage.MessageBody.MessageResponse.DataType.LCDPosition);
            xSemaphoreGive(UartProtector);
        }
        else
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nInvalid task ID with response");
            xSemaphoreGive(UartProtector);
        }
    }
    else if(ReceivedSensorMessage.MessageType==REQUEST_MESSAGE)
    {
        /* send a response to this request */
        /* create packet that needs to be send */
        InterTaskPacket_t SensorDataResponse=
        {
            .MessageType=RESPONSE_MESSAGE,
            .MessageBody.MessageResponse.SenderTaskID=SENSOR_TASK,
            .MessageBody.MessageResponse.DataType.ProximityData=proximity_value,
        };
        /* Find out who sent the request and send the response to their queue */
        if(ReceivedSensorMessage.MessageBody.MessageResponse.SenderTaskID==EEPROM_TASK)
        {
            QueueSendReturn=xQueueSendToFront(QueueHandle[EEPROM_TASK], &SensorDataResponse, portMAX_DELAY);
        }
        else if(ReceivedSensorMessage.MessageBody.MessageResponse.SenderTaskID==LCD_TASK)
        {
            QueueSendReturn=xQueueSendToFront(QueueHandle[LCD_TASK], &SensorDataResponse, portMAX_DELAY);
        }
        else
        {
            xSemaphoreTake(UartProtector, portMAX_DELAY);
            UARTprintf("\r\nInvalid task ID with request");
            xSemaphoreGive(UartProtector);
        }
    }
    return QueueSendReturn;
}
