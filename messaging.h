/*
 *  File        :   messaging.c
 *
 *  Description :   Has prototypes for routines that deal with messaging among threads.
 *                  Also has the prototypes for the data that's sent on the queue.
 *
 *  Created on  :   Dec 5, 2017
 *
 *      Author  :   Ashwath Gundepally, CU, ECEE
 *                  Sahana Sadagopan, CU, ECEE
 *
 */

#ifndef MESSAGING_H_
#define MESSAGING_H_

#include<stdint.h>
#include"main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

extern QueueHandle_t QueueHandle[TOTAL_NUMBER_OF_TASKS-1];


/* this type helps to store all the possible message types */
typedef enum {REQUEST_MESSAGE, RESPONSE_MESSAGE}MessageType_t;

/* this is a mutex type that protects the UART driver */
extern SemaphoreHandle_t UartProtector;

typedef union
{
    uint8_t ProximityData;
    uint16_t EepromAddress;
    uint8_t LCDPosition;
}DataType_t;

typedef struct
{
   /* sender task ID- helps to understand who sent this message.
    * The receiver's queue will have multiple writers
    * and one reader(the receiver) */
    TaskID SenderTaskID;

    DataType_t DataType;

    /* TODO: Add time stamp variable here */

}InterTaskMessageResponsePart_t;

/* this struct type helps to store the various */
typedef struct
{
   /* sender task ID- helps to understand who sent this message.
    * The receiver's queue will have multiple writers
    * and one reader(the receiver) */
    TaskID SenderTaskID;

    /* TODO: Add time stamp variable here */

}InterTaskMessageRequestPart_t;


/* Some members of a message don't make sense depending
 * on whether or not the message is a command or a response.
 * For example, the DataType_t type does not belong in the
 * message packet if the message is a request type. That variable
 * will simply not be initialized and is sent anyway. Thus, having
 * a union which is accessed as per the MessageType_t type
 * makes sense. While this does not offer any improvements in
 * terms of performance, memory. It is much more readable and modular.
 *
 * This type helps to store the message body. And will be accessed
 * depending on the message type.
 *  */
typedef union
{
    /* the message is a request to the receiver, he will read from this variable */
    InterTaskMessageRequestPart_t MessageRequest;

    /* the message is a response to the receiver, he will read from this variable */
    InterTaskMessageResponsePart_t MessageResponse;
}InterTaskMessageBody_t;


typedef struct
{
    /* the type of this message */
    MessageType_t MessageType;

    /* body of the message- access the body as per the type of the message */
    InterTaskMessageBody_t MessageBody;

}InterTaskPacket_t;

//* sends a request command to the Sensor Task's queue for data */
BaseType_t SendProximityDataReq(TaskID SenderTaskID);

/* sends a request command to the LCD Task's queue for the number of chars currently on display */
BaseType_t SendLCDCharsReq(TaskID SenderTaskID);

/* sends a request command to the EEPROM Task's queue for the address counter */
void SendEEPROMAddressReq();

/* Read the LCD Task's queue for messages sent and received */
BaseType_t ReadQueueLCD();

/* Read the Sensor Task's queue for messages sent and received */
BaseType_t ReadQueueSensor();

#endif /* MESSAGING_H_ */
