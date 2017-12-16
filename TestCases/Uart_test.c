/**Uses minicom to test with baud rate 9600*/
/** Enable pin configuration Pin 24 and 26 each time the board is rebooted**/
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <syslog.h>
/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */

int
open_port(void)
{
    int fd; /* File descriptor for the port */



    fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (fd == -1)
    {
        /*
         * Could not open the port.
         */
        perror("open_port: Unable to open /dev/ttyf1 - ");
    }
    else
       fcntl(fd, F_SETFL, 0);
    
    return (fd);
}

void send_byte(int fd,char data ){
    int n = write(fd, &data, 1);
    //tcdrain(fd);
       if (n < 0)
          fputs("write() of 4 bytes failed!\n", stderr);
}

int send_string(int fd,char *string){
    int i=0;
    while(1){
        send_byte(fd,string[i]);
        if(string[i]=='\0')
            break;
        i++;
    }
    //printf("string sent:%s\n", string);
    
}

void read_byte(int fd,char *received){
    int n;
    if( (n=read(fd, received, 1))>0){
         if (n < 0)
            fputs("read failed", stderr);    
        
    //printf("data: %c\n", *received);
    syslog(LOG_INFO,"DATA:%c\n",*received);
    }
    else{
        printf("Cannot read");
    }
    
}

int read_string(int fd,char *string){
    int i=0;
    while(1){
        read_byte(fd,&string[i]);
        if(string[i]=='\0')
            break;
        //printf("rs: %c\n",string[i]);
        i++;
    }
     //printf("string read: %s\n", string);
}


int main()
{
    int fd=open_port();
    char received[25];
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSAFLUSH, &options);
    
    char data[25];
    strcpy(data,"Ujksdhfkjhds");
    send_string(fd,data);
    //send_byte(fd,data);
    while(1){
        read_byte(fd,received);
    }
    //read_string(fd,received);
    
    
    
    
    close(fd); 
    return 0;
}




















