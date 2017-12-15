/*
 * main.h
 *
 *  Created on: Mar 28, 2015
 *      Author: akobyljanec
 */

#ifndef MAIN_H_
#define MAIN_H_


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
/* System clock rate, 120 MHz */
#define SYSTEM_CLOCK    120000000U
#define SLAVE_ADDR      0X39

/* define the total number of tasks */
#define TOTAL_NUMBER_OF_TASKS       5

#define QUEUE_LENGTH                10

/* these macros store the notification value sent by each task to the heart-beat task */
#define LCD_TASK_HEARTBEAT          0x00008000
#define SENSOR_TASK_HEARTBEAT       0x00000001
#define EEPROM_TASK_HEARTBEAT       0x80000000
#define UARTLOGGER_TASK_HEARTBEAT   0x00800000


/* This type will help store an identifier associated with each task
 * This is just an enum to be able to iterate over the TaskHandle array
 * defined in main  */
typedef enum {SENSOR_TASK, LCD_TASK, EEPROM_TASK, UARTLOGGER_TASK, HEARTBEAT_TASK}TaskID;

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)
void proximity_data(uint8_t *pdata_val);

#endif /* MAIN_H_ */
