#ifndef UART_NEW_H
#define UART_NEW_H 

#include<stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include "uart_new.h"


typedef enum {HEARTBEAT_RECEIVED=0, HEARTBEAT_MISSED=1} heartbeat_t;
#define NUMBER_OF_THREADS_TIVA      4
/* this packet will be received in the UART task */
typedef struct 
{
    /* collect TIVA heartbeats in this uint8_t type */
    uint8_t tiva_heartbeats[4];

    /* sensor data */
    uint8_t sensor_data;

    /* eeprom data */
    uint8_t eeprom_data;

}log_packet_uart_t;

/* options for serial port config */
void tty_config(struct termios *con, int descriptor);

/* read UART byte */
void read_byte(int fd,char *received);

/* read string */
void read_string(int fd,char *string);

/* read the structure */
void read_struct(int descriptor, log_packet_uart_t* recvd_struct);

/* initialize UART */
void uart_init(void);
#endif
