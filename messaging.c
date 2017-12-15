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



/* Read the Sensor Task's queue for messages sent and received */
BaseType_t ReadQueueSensor()
{
    InterTaskPacket_t ReceivedSensorMessage;

    uint8_t proximity_value=0;

    proximity_data(&proximity_value);

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
