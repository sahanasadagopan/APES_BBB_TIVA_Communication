/*
 * main.h
 *
 *  Created on: Mar 28, 2015
 *      Author: akobyljanec
 */

#ifndef MAIN_H_
#define MAIN_H_

// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U
#define SLAVE_ADDR      0X39

/* define the total number of tasks */
#define TOTAL_NUMBER_OF_TASKS       3

/* these macros store the notification value sent by each task to the heart-beat task */
#define LCD_TASK_HEARTBEAT          0x00008000
#define SENSOR_TASK_HEARTBEAT       0x00000001

/* This type will help store an identifier associated with each task
 * This is just an enum to be able to iterate over the TaskHandle array
 * defined in main  */
typedef enum {HEARTBEAT_TASK, SENSOR_TASK, LCD_TASK}TaskID;

#endif /* MAIN_H_ */
