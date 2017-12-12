/**Uses minicom to test with baud rate 9600*/

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

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

void read_byte(int fd,char received){
    int n;
    if( (n=read(fd, &received, 1))>0){
         if (n < 0)
            fputs("read failed", stderr);    
        
    printf("data:%c", received);
    }
    else{
        printf("Cannot read");
    }
    
}

int main()
{
    int fd=open_port();
    char received=0;
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSAFLUSH, &options);
    
    char data='U';
    send_byte(fd,data);
    read_byte(fd,received);
    
    
    
    
    close(fd); 
    return 0;
}



























