#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#define PORT 3000

pthread_t decision_td;
pthread_t logger_td;
pthread_t server_td;
pthread_t recv_conn_td;

void *logger(void *logger_param);

void *startServer(void *server)
{
	int sin_size, client_sock,sockfd;
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
	addr_local.sin_port = htons(PORT); // Port number
	addr_local.sin_addr.s_addr = INADDR_ANY; // AutoFill local address
	bzero(&(addr_local.sin_zero), 8); // Flush the rest of struct

	/* Bind a special Port */
	if( bind(sockfd, (struct sockaddr*)&addr_local, sizeof(struct sockaddr)) == -1 )
	{
		fprintf(stderr, "ERROR: Failed to bind Port. (errno = %d)\n", errno);
		exit(1);
	}
	else
		printf("[Server] Binded tcp port %d in addr 127.0.0.1 sucessfully.\n",PORT);

	/* Listen remote connect/calling */
	if(listen(sockfd,3) == -1)
	{
		fprintf(stderr, "ERROR: Failed to listen Port. (errno = %d)\n", errno);
		exit(1);
	}
	else
		printf ("[Server] Listening the port %d successfully.\n", PORT);

	while((client_sock=accept(sockfd,(struct sockaddr *)&addr_remote, sizeof(addr_remote)))){
		if(pthread_create(&recv_conn_td,NULL,logger,(void *)&client_sock)<0){
			perror("Thread not created\n");
	    	exit(1);
		}
	}

}

void *connection(void *client_sock){
	int sock = *(int*)client_sock;
	int read_size;
	char *message, client_message[1000];
	printf("recieving connecting stuff\n");


}

void *logger(void *logger_param){

    
}

void *decision(void *decision_param){
    
}

int main(){
    int ret;
   // startServer();

    if(pthread_create(&server_td,NULL,startServer,NULL)<0){
    	perror("Thread for server and client connection not created\n");
    	exit(1);
    }
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
	pthread_join(server_td,NULL);

    
    
}
