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

#define MAX_MSGS_IN_LOG_QUEUE   100
#define MAX_MSG_SIZE_LOG_QUEUE  50 
#define LOG_QUEUE_NAME "/log_queue"

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

/* message queue initialisations */
mqd_t logger_queue;

/* log file file descriptor */
FILE* log_fd;

typedef enum {LOG_INFO, LOG_CRITICAL, LOG_DRIVER}log_level_t;

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
    
    /* TODO: add timestamp variable */
    
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
}log_packet_t;

/**/
int fd;


/**/
void int_handler()
{
    printf("closing...\n");
    
    fclose(log_fd);

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
#ifdef ENABLE_MAIN_LOGGING 
    /* find the log level and write it to the log_level_string */
    if(log_packet.log_level==LOG_INFO)
        strcpy(log_level_string, "LOG_INFO");
    else if(log_packet.log_level==LOG_DRIVER)
        strcpy(log_level_string, "LOG_DRIVER");
    else if(log_packet.log_level==LOG_CRITICAL)
        strcpy(log_level_string, "LOG_CRITICAL");
    
    /* create the heartbeat string */    
    strcpy(heartbeat_stuff, "HEART BEATS: ");
    uint8_t index;
    
    /* now just concatenate strings based on whether or not their heartbeat is present or absent */
    for(index=0; index<NUMBER_OF_THREADS-1; index++)
    {
        strcat(heartbeat_stuff, thread_names[index]);
        if(log_packet.heartbeat[index]==HEARTBEAT_RECEIVED)
           strcat(heartbeat_stuff, "HB RECEIVED  ");
        else
           strcat(heartbeat_stuff, "HB MISSED ");
    }
    
    /* put everything together */
    sprintf(log_entry, "[%s][%s][%s]\n", thread_id_string, log_level_string, heartbeat_stuff);

    /* write to the log file */
   //fprintf(log_fd, "%s", log_entry);
    fwrite(log_entry, 1, strlen(log_entry), log_fd);
#endif  
        
    if(log_packet.thread_id==UART_THREAD)
    {   
        printf("received uart packet\n");
        uint8_t index;                   
        /* now just concatenate strings based on whether or not their heartbeat is present or absent */
        for(index=0; index<NUMBER_OF_THREADS_TIVA; index++)
        {
            strcat(heartbeat_stuff, thread_names_tiva[index]);
            if(log_packet.log_content.uart_log_entry.tiva_heartbeats[index]==HEARTBEAT_RECEIVED)
                strcat(heartbeat_stuff, "HB RECEIVED  ");
            else
                strcat(heartbeat_stuff, "HB MISSED ");
        }
        

        /* put everything together */
        sprintf(log_entry, "[%s][LOG_INFO][%s]\n\t[Sensor Value: %d][EEPROM Addr: %d\n]",\
                thread_id_string, heartbeat_stuff, log_packet.log_content.uart_log_entry.sensor_data\
                , log_packet.log_content.uart_log_entry.eeprom_data);
        
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
        
        /* TODO: Reevaluate as to whether or now the logger must have a HB */ 
        pthread_mutex_lock(&logger_mutex);
        if(logger_flag != 1)
        {
            pthread_cond_signal(&logger_cond_var);
            logger_flag = 1;
            printf("logger thread sent HB\n");
        }
        pthread_mutex_unlock(&logger_mutex);
        
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
        
//        printf("Log level:%d\n, sender id:%d\n, first heartbeat:%d\n", log_packet.log_level, log_packet.thread_id,\
  //                                                                  log_packet.heartbeat[0]);
	
	read_struct(fd, &uart_log);

        /* write this log to the file */
        write_log(log_packet);
          
        sleep(20);
    }
}

void* uart_thread(void* arg)
{
    /* initialize the uart driver */
    uart_init();
    
    //
    log_packet_uart_t uart_log_recvd; 
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
        
	printf("sizeof packet sent:%lu\n", sizeof(log_packet_uart_t));

        if(mq_send(logger_queue, (char*)&uart_log_recvd, sizeof(log_packet_uart_t), 0)==-1)
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

    logger_queue_attr.mq_flags=0;

    logger_queue_attr.mq_maxmsg=MAX_MSGS_IN_LOG_QUEUE;
    logger_queue_attr.mq_msgsize=50;

    /*Logger queue Created and Opened*/
    logger_queue=mq_open(LOG_QUEUE_NAME, O_CREAT|O_RDWR, 666, &logger_queue_attr);
    
    if(logger_queue==(mqd_t)-1)
    {
        perror("mq in logger failed");
        exit(0);
    }
    
    signal(SIGINT, int_handler);
        
    /* thread descriptors for each task */
    pthread_t uart_td, logger_td;
        
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
	
    //signal(SIGINT,int_handler);
    heartbeat_t local_heartbeat[NUMBER_OF_THREADS-1];
        
    while(1)
    {
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
            
	        //heartbeat_api("temp",&hb_message);
	        
#ifdef MESG_QUEUE_LOG
            if(mq_send(logger_queue, (char*)&hb_message, MAX_MSG_SIZE_LOG_QUEUE, 0)==-1)
            {
                perror("mq_send failed");
                exit(0);
            }   
#endif      
        }   
        pthread_mutex_unlock(&uart_mutex);
            
        /* Get Heartbeat from the logger task */
        pthread_mutex_lock(&logger_mutex);
            
        rc = gettimeofday(&tp,NULL);
        heartbeat_time.tv_sec = tp.tv_sec;
        heartbeat_time.tv_nsec = tp.tv_usec * 1000;
        heartbeat_time.tv_sec +=WAIT_TIME_SECONDS;
            
        if(logger_flag == 1)
        {   
	        logger_flag = 0;
            
            /*conditional wait for logger thread*/
	        rc=pthread_cond_timedwait(&logger_cond_var, &logger_mutex, &heartbeat_time);
	        if(rc==ETIMEDOUT)
            {
		        printf("logger task missed hb!\n");
#ifdef MAIN_LOGGING 
                log_packet_t log_packet;
                log_packet.log_level=LOG_CRITICAL;
                log_packet.heartbeat[LOGGER_THREAD]=HEARTBEAT_MISSED;
                log_packet.heartbeat[UART_THREAD]=local_heartbeat[UART_THREAD];
                log_packet.thread_id=MAIN_THREAD; 
                /* update local heartbeat val */
                local_heartbeat[LOGGER_THREAD]=HEARTBEAT_MISSED;
                
                printf("sizeof packet sent:%lu\n", sizeof(log_packet));
                if(mq_send(logger_queue, (char*)&log_packet, sizeof(log_packet), 0)==-1)
                {
                    perror("mq_send failed");
                    exit(0);
                }
#endif
	        }
            else
            {   
                /* update local heartbeat */
                local_heartbeat[LOGGER_THREAD]=HEARTBEAT_RECEIVED;
            }
            
            //heartbeat_api("decision",&hb_message);
#ifdef MESG_QUEUE_LOG	    
            if(mq_send(logger_queue, (char*)&hb_message, MAX_MSG_SIZE_LOG_QUEUE, 0)==-1)
            {
                perror("mq_send failed");
                exit(0);
            }
#endif
        }
	    pthread_mutex_unlock(&logger_mutex);
    }

#ifdef SURE_ABOUT_JOINS 
    /* pthreads are joined after the while loop */
    pthread_join(logger_td, NULL);
    pthread_join(uart_td, NULL);
#endif
}           
