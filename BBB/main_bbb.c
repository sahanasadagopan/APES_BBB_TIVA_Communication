#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <unistd.h>
#include <string.h>

#define PORT 3000

pthread_t decision_td;
pthread_t logger_td;

void startServer(int *port)
{
	int sin_size;
	struct sockaddr_in addr_local; /* client addr */
	struct sockaddr_in addr_remote; /* server addr */
	/* Get the Socket file descriptor */
	if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
	{
		fprintf(stderr, "ERROR: Failed to obtain Socket Descriptor. (errno = %d)\n", errno);
		exit(1);
	}
	else
		printf("[Server] Obtaining socket descriptor successfully.\n");

	/* Fill the client socket address struct */
	addr_local.sin_family = AF_INET; // Protocol Family
	addr_local.sin_port = htons((port)); // Port number
	addr_local.sin_addr.s_addr = INADDR_ANY; // AutoFill local address
	bzero(&(addr_local.sin_zero), 8); // Flush the rest of struct

	/* Bind a special Port */
	if( bind(sockfd, (struct sockaddr*)&addr_local, sizeof(struct sockaddr)) == -1 )
	{
		fprintf(stderr, "ERROR: Failed to bind Port. (errno = %d)\n", errno);
		exit(1);
	}
	else
		printf("[Server] Binded tcp port %s in addr 127.0.0.1 sucessfully.\n",port);

	/* Listen remote connect/calling */
	if(listen(sockfd,BACKLOG) == -1)
	{
		fprintf(stderr, "ERROR: Failed to listen Port. (errno = %d)\n", errno);
		exit(1);
	}
	else
		printf ("[Server] Listening the port %s successfully.\n", port);

}

void *logger(void *logger_param){
    
}

void *decision(void *decision_param){
    
}

int main(){
    
    startServer(PORT);
    if(pthread_create(&logger_td, NULL, logger, NULL)<0){
        perror("Thread not created\n");
	    exit(1);
    }
    if(ret = pthread_create(&decision_td,(void *)0,decision,(void *)0)<0){
		perror("Thread not created\n");
		exit(1);
	}
	
	pthread_join(logger_td, NULL);
	pthread_join(decision_td,NULL);

    
    
}
