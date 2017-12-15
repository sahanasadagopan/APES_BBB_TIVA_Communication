#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include <syslog.h>
#include "uart_new.h"

struct termios *configure;
void tty_config(struct termios *con, int descriptor);
char *device = "/dev/ttyO2";
int fd;



void tty_config(struct termios *con, int descriptor)
{
    tcgetattr(descriptor, con);
    con->c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    con->c_oflag = 0;
    con->c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    con->c_cc[VMIN] = 1;
    con->c_cc[VTIME] = 0;

    if(cfsetispeed(con, B115200) || cfsetospeed(con, B115200))
    {
        perror("ERROR in baud set\n");
    }

    if(tcsetattr(descriptor, TCSAFLUSH, con) < 0)
    {
        perror("ERROR in set attr\n");
    }
}

void read_byte(int fd,char *received){
    int n;
    if( (n=read(fd, received, 1))>0)
    {
    	if (n < 0)
            fputs("read failed", stderr);    
        
    	printf("data: %d\n", *received);
    }
    else{
        printf("Cannot read");
    }
    
}

void uart_init(void)
{
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd==-1)
    {
        perror("ERROR opening file descriptor\n");
    }

    configure = (struct termios*)malloc(sizeof(struct termios));
    if(configure==NULL){
        printf("Malloc fail");
    }
    tty_config(configure, fd);
}

void read_string(int fd,char *string){
    int i=0;
    
    while(i<4)
    {
        read_byte(fd,&string[i]);
        //if(string[i]=='\0')
          //  break;
        //printf("rs: %c\n",string[i]);
        i++;
    }
     //printf("string read: %s\n", string);
}

void read_struct(int descriptor, log_packet_uart_t* recvd_struct)
{
    int struct_size=sizeof(log_packet_uart_t);
    
    printf("size:%d\n", struct_size);
     
    for(int i=0; i<4; i++)
    	read_byte(descriptor, (char*)&(recvd_struct->tiva_heartbeats[i]));	    
    read_byte(descriptor, &(recvd_struct->sensor_data));
    read_string(descriptor, (char*)&(recvd_struct->eeprom_data));
    printf("Sensor data %d\n",(recvd_struct->sensor_data));
    
}
#ifdef TEST_FUNCS
int main()
{
    uart_init();
    char buff[2];
    sleep(1);
    //buff[1]='\0';
    buff[0]=0;
    
    char recd_string[40];
    
    device_message_packet_t recvd_struct;

    while(1)
    {
        read_struct(fd, &recvd_struct);
    }
    close(fd);
}
#endif
