/*      
 *  File            :   main.c
 *                      
 *  Description     :   Creates threads that send and receive packets from the 
 *                      TIVA board, creates a logger task that logs all received
 *                      data, creates a decision task that logs out of bound
 *                      data and toggles an LED.
 *  
 *  Author          :   Sahana Sadagopan, CU, ECEE
 *                      Ashwath Gundepally, CU, ECEE
 */

#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <errno.h>
#include "uart_new.h"
#include "USRled.h"

#define MAX_MSGS_IN_LOG_QUEUE   100
#define MAX_MSG_SIZE_LOG_QUEUE  50 
#define LOG_QUEUE_NAME "/log_queue"
#define DECISION_QUEUE_NAME "/decision_queue"

/* thread descriptors for each task */
pthread_t uart_td, logger_td, decision_td;

/* Mutex Variables */
pthread_mutex_t uart_mutex=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t logger_mutex=PTHREAD_MUTEX_INITIALIZER;


/* flag variable for Heartbeat*/
static volatile sig_atomic_t uart_flag=1;
static volatile sig_atomic_t logger_flag=1;

/* Conditional Variable */
pthread_cond_t logger_cond_var=PTHREAD_COND_INITIALIZER;
pthread_cond_t uart_cond_var=PTHREAD_COND_INITIALIZER;

/* store the filename in this variable */
char filepath[50];

#define WAIT_TIME_SECONDS   5
#define NUMBER_OF_THREADS   3 


/********************************************************************
 Timer Function Definitions
 Function borrowed from : http://ecee.colorado.edu/~ecen5623/ecen/labs/Linux/IS/Report.pdf
********************************************************************/
double readTOD(void)
{
 struct timeval tv;
 double ft=0.0;
 if( gettimeofday (& tv, NULL) != 0)
 {
 perror("readTOD");
 return 0.0;
 }
 else
 {
 ft = ((double)(((double)tv.tv_sec) + (((double)tv.tv_usec) /
1000000.0)));
 }
 return ft;
}

/* message queue initialisations */
mqd_t logger_queue;
mqd_t decision_queue;
/* log file file descriptor */
FILE* log_fd;

typedef enum {LOG_INFO, LOG_CRITICAL, LOG_DRIVER, LOG_TIVA}log_level_t;

typedef enum {UART_THREAD=0, LOGGER_THREAD=1, MAIN_THREAD=2}thread_id_t;

const char* thread_names[3]={ "UART_THREAD: ", "LOGGER_THREAD: ", "MAIN_THREAD: "};

const char* thread_names_tiva[4]={ "SENSOR_TIVA: ", "LCD_TIVA: ", "EEPROM_TIVA: ", "UART_TIVA: "}; 

/* message packet stuff */
typedef struct 
{
    /* level of the log */
    log_level_t log_level;
    
    /* heartbeat values of all the threads */
    heartbeat_t heartbeat[NUMBER_OF_THREADS-1];
    
    
}log_packet_main_t;

/* this union will store the log content */
typedef union
{
    log_packet_uart_t uart_log_entry;

    log_packet_main_t main_log_entry;
}log_packet_content_t;

/* this is the packet that will be written to the logger queue */
typedef struct 
{
    /* sender id */
    thread_id_t thread_id;
    
    /* this variable will be accessed as per the sender id */  
    log_packet_content_t log_content;
    
    /* TODO: add timestamp variable */
    double timestamp;
}log_packet_t;

/**/
int fd;

char log_entry_isr[1000];
/**/
void int_handler()
{
    
    printf("closing...\n");
    sprintf(log_entry_isr, "[%lf][LOG_CRITICAL][%s]\n", readTOD(), "The Process is Killed");
    fwrite(log_entry_isr, 1, strlen(log_entry_isr), log_fd);
    fclose(log_fd);
    mq_close(logger_queue);
    mq_unlink(LOG_QUEUE_NAME);
    mq_close(decision_queue);
    mq_unlink(DECISION_QUEUE_NAME);
    pthread_cond_destroy(&logger_cond_var);
    pthread_cond_destroy(&uart_cond_var);
    pthread_cancel(uart_td);
    pthread_cancel(decision_td);
    
    exit(0);
    
}



void write_log(log_packet_t log_packet)
{
    /* parse the packet and write a string to the log file */

    /* store various attributes of a log entry in these char arrays */
    char thread_id_string[30], log_level_string[30], heartbeat_stuff[200], log_entry[260];
    
    /* find the thread id and write it to the thread_id_string */
    if(log_packet.thread_id==MAIN_THREAD)
    {
        strcpy(thread_id_string, "MAIN_THREAD");
    }
    else if(log_packet.thread_id==UART_THREAD)
    {
        strcpy(thread_id_string, "UART_THREAD");
    }
    else if(log_packet.thread_id==LOGGER_THREAD)
    {
        strcpy(thread_id_string, "LOGGER_THREAD");
    }
    
    /* create the heartbeat string */    
    strcpy(heartbeat_stuff, "HEART BEATS: ");
    
    if(log_packet.thread_id==MAIN_THREAD)
    {
        /* find the log level and write it to the log_level_string */
        if(log_packet.log_content.main_log_entry.log_level==LOG_INFO)
            strcpy(log_level_string, "LOG_INFO");
        else if(log_packet.log_content.main_log_entry.log_level==LOG_DRIVER)
            strcpy(log_level_string, "LOG_DRIVER");
        else if(log_packet.log_content.main_log_entry.log_level==LOG_CRITICAL)
            strcpy(log_level_string, "LOG_CRITICAL");

        uint8_t index;
    
        /* now just concatenate strings based on whether or not their heartbeat is present or absent */
        for(index=0; index<1; index++)
        {
            strcat(heartbeat_stuff, thread_names[index]);
            if(log_packet.log_content.main_log_entry.heartbeat[index]==HEARTBEAT_RECEIVED)
                strcat(heartbeat_stuff, "HB RECEIVED  ");
            else
                strcat(heartbeat_stuff, "HB MISSED ");
        }
    
        /* put everything together */
        sprintf(log_entry, "[%lf][%s][%s][%s]\n", log_packet.timestamp, thread_id_string, log_level_string, heartbeat_stuff);
        
	puts(log_entry);

    /* write to the log file */
   //fprintf(log_fd, "%s", log_entry);
        fwrite(log_entry, 1, strlen(log_entry), log_fd);
	
    }
    else if(log_packet.thread_id==UART_THREAD)
    {   
        printf("received uart packet\n");
        uint8_t index;                   
        /* now just concatenate strings based on whether or not their heartbeat is present or absent */
        for(index=0; index<NUMBER_OF_THREADS_TIVA; index++)
        {
            strcat(heartbeat_stuff, thread_names_tiva[index]);
	    printf("heartbeat %d :%d\n", index, log_packet.log_content.uart_log_entry.tiva_heartbeats[index]);
            if(log_packet.log_content.uart_log_entry.tiva_heartbeats[index]==HEARTBEAT_RECEIVED)
                strcat(heartbeat_stuff, "HB RECEIVED  ");
            else
                strcat(heartbeat_stuff, "HB MISSED ");
        }
        

        /* put everything together */
        sprintf(log_entry, "[%lf][%s][LOG_INFO][%s][Sensor Value: %u][EEPROM Addr: %u]\n",\
                log_packet.timestamp, thread_id_string, heartbeat_stuff, log_packet.log_content.uart_log_entry.sensor_data,\
                log_packet.log_content.uart_log_entry.eeprom_data);
        
        puts(log_entry);

        /* write to the log file */
        //fprintf(log_fd, "%s", log_entry);
        fwrite(log_entry, 1, strlen(log_entry), log_fd);
    }
}       

void* logger_thread(void* arg)
{
    /* store returns from all the syscall returns */
    int sys_call_rc;
    
    /* open the log file */
    log_fd=fopen(filepath, "w+");
    
    /* check if the file was successfully created */
    if(log_fd==NULL)
    {
        printf("log file path invalid; couldn't open file\n");
        exit(0);
    }
    
    /*intialize uart */
    uart_init();
    log_packet_uart_t uart_log;

    while(1)
    {
        printf("hello from the logger task\n");
        
        
        /* store the  log message in this variable */
        log_packet_t log_packet;
        
        printf("log_packet size:%lu\n", sizeof(log_packet_t));
        
        /* read from the queue */
        sys_call_rc=mq_receive(logger_queue, (char*)&log_packet,\
                                50, NULL);
        
        if(sys_call_rc==-1)
        {
            perror("read from log queue failed");
            exit(0);
        }
         
        printf("sizeof packet received:%lu\n", sizeof(log_packet));
        
        printf(" thread id :%d\n", log_packet.thread_id);
	

        /* write this log to the file */
        write_log(log_packet);
          
        sleep(2);
    }
}

void* decision_thread(void* arg)
{
    int sys_call_rc;
    log_packet_t decision_log_packet;
    //pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
    while(1)
    {
        printf("hello from the decision task\n");
        sys_call_rc=mq_receive(decision_queue, (char*)&decision_log_packet,\
                                50, NULL);
        
        if(sys_call_rc==-1)
        {
            perror("read from log queue failed");
            exit(0);
        }
        printf("The sensor value data %d\n",decision_log_packet.log_content.uart_log_entry.sensor_data);
        if(decision_log_packet.log_content.uart_log_entry.sensor_data>30){
            printf("User too close to sensor\n");
            const char *LEDBrightness = "/sys/class/leds/beaglebone:green:usr0/brightness";
            detection_led(LEDBrightness);

        }
        if((decision_log_packet.log_content.uart_log_entry.tiva_heartbeats[0])==0){
            const char *LEDBrightness = "/sys/class/leds/beaglebone:green:usr1/brightness";
            detection_led(LEDBrightness);
        }
        else if((decision_log_packet.log_content.uart_log_entry.tiva_heartbeats[1])==0){
            const char *LEDBrightness = "/sys/class/leds/beaglebone:green:usr1/brightness";
            detection_led(LEDBrightness);
        }
        else if((decision_log_packet.log_content.uart_log_entry.tiva_heartbeats[2])==0){
            const char *LEDBrightness = "/sys/class/leds/beaglebone:green:usr1/brightness";
            detection_led(LEDBrightness);
         }
        else if((decision_log_packet.log_content.uart_log_entry.tiva_heartbeats[3])==0){
            const char *LEDBrightness = "/sys/class/leds/beaglebone:green:usr1/brightness";
            detection_led(LEDBrightness);
            
        }
        sleep(2);
    }
}
void* uart_thread(void* arg)
{
    /* initialize the uart driver */
    uart_init();
    
    //
    log_packet_uart_t uart_log_recvd; 
    log_packet_t uart_log;
    uart_log.thread_id=UART_THREAD;
    uint8_t index;
    //pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
    while(1)
    {
        printf("hello from the UART task\n");
        sleep(2);
        
        pthread_mutex_lock(&uart_mutex);
        
        if(uart_flag != 1)
        {
            pthread_cond_signal(&uart_cond_var);
            uart_flag = 1;
            printf("uart Thread sent HB\n");
    
        }
        pthread_mutex_unlock(&uart_mutex);
        
        read_struct(fd, &uart_log_recvd);
        
	printf("ssr:%d\n", uart_log_recvd.sensor_data);
        
	for(index=0; index<4; index++)
           uart_log.log_content.uart_log_entry.tiva_heartbeats[index]=uart_log_recvd.tiva_heartbeats[index];
	
	uart_log.log_content.uart_log_entry.sensor_data=uart_log_recvd.sensor_data;
        uart_log.log_content.uart_log_entry.eeprom_data=uart_log_recvd.eeprom_data;
        uart_log.timestamp=readTOD();

	printf("After assigning is %d\n", uart_log.log_content.uart_log_entry.sensor_data);

        if(mq_send(logger_queue, (char*)&uart_log, 50, 0)==-1)
        {
            perror("mq_send failed");
            exit(0);
        }
        if(mq_send(decision_queue, (char*)&uart_log, 50, 0)==-1)
        {
            perror("mq_send failed");
            exit(0);
        }
        
    }
    close(fd);
}

int main(int argc,char **argv)
{
    int ret=0,rc=0;
    
    if(argc!=2)
    {
        printf("usage: <executable> <log filepath>\n");
        exit(0);
    }

    strcpy(filepath, argv[1]);
    
     
    struct timespec heartbeat_time;
    struct timeval tp;
    

    struct mq_attr logger_queue_attr;
    struct mq_attr decision_queue_attr;

    logger_queue_attr.mq_flags=0;
    decision_queue_attr.mq_flags=0;

    logger_queue_attr.mq_maxmsg=MAX_MSGS_IN_LOG_QUEUE;
    decision_queue_attr.mq_maxmsg=MAX_MSGS_IN_LOG_QUEUE;
    logger_queue_attr.mq_msgsize=50;
    decision_queue_attr.mq_msgsize=50;

    /*Logger queue Created and Opened*/
    logger_queue=mq_open(LOG_QUEUE_NAME, O_CREAT|O_RDWR, 666, &logger_queue_attr);
    decision_queue=mq_open(DECISION_QUEUE_NAME,O_CREAT|O_RDWR, 666, &decision_queue_attr);
    if(logger_queue==(mqd_t)-1)
    {
        perror("mq in logger failed");
        exit(0);
    }
    
    signal(SIGINT, int_handler);
        
        
    if(pthread_create(&uart_td, NULL, uart_thread, NULL)<0)
    {
        perror("uart thread not created\n");
        exit(1);
    }
  

    if(pthread_create(&logger_td, NULL, logger_thread, NULL)<0)
    {
        perror("Logger thread not created\n");
	    exit(1);
    }
    
    if(pthread_create(&decision_td, NULL, decision_thread, NULL)<0)
    {
        perror("decision thread not created\n");
	    exit(1);
    }
	
    //signal(SIGINT,int_handler);
    heartbeat_t local_heartbeat[NUMBER_OF_THREADS-1];
    log_packet_t log_packet;
    log_packet_main_t main_log;    
    while(1)
    {
	sleep(6);
        /* Get Heartbeat from the UART task */
        pthread_mutex_lock(&uart_mutex);
        rc = gettimeofday(&tp,NULL);
        heartbeat_time.tv_sec = tp.tv_sec;
        heartbeat_time.tv_nsec = tp.tv_usec * 1000;
        heartbeat_time.tv_sec +=WAIT_TIME_SECONDS;
        
        if(uart_flag == 1)
        {
	        uart_flag =0;
	        rc=pthread_cond_timedwait(&uart_cond_var, &uart_mutex, &heartbeat_time);
	        
            if(rc==ETIMEDOUT)
            {
		        printf("uart task wait timed out!\n");
                local_heartbeat[UART_THREAD]=HEARTBEAT_MISSED;
	        }
            else
            {
                local_heartbeat[UART_THREAD]=HEARTBEAT_RECEIVED;
            }
        }   
        pthread_mutex_unlock(&uart_mutex);
         
        /* log the status over to the logger thread */
        if(rc==ETIMEDOUT)
        {
            main_log.log_level=LOG_CRITICAL; 
        }
        else
        {
            main_log.log_level=LOG_INFO;
        }
        
        main_log.heartbeat[UART_THREAD]=local_heartbeat[UART_THREAD];
        log_packet.thread_id=MAIN_THREAD;
        log_packet.log_content.main_log_entry=main_log;
        log_packet.timestamp=readTOD();
        
	if(mq_send(logger_queue, (char*)&log_packet, 50, 0)==-1)
        {
            perror("mq_send failed");
            exit(0);
        }
    
    }

#ifdef SURE_ABOUT_JOINS 
    /* pthreads are joined after the while loop */
    pthread_join(logger_td, NULL);
    pthread_join(uart_td, NULL);
#endif
}           
