#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <sys/sem.h>
#include <iostream>
#include <semaphore.h>
#include <fcntl.h>
#include <signal.h>

#include <shared_memory.hpp>
//#include "shared_memory.hpp"

#define BLOCK_SIZE 4096

int sock; /* Socket descriptor */
sem_t *sem_full_odo;
sem_t *sem_empty_odo;
sem_t *init_odo;
sem_t *mutex_odo;
struct SharedMemoryODO *block;

#define RCVBUFSIZE 5000 /* Size of receive buffer */


void signalHandler(int sig)
{
    printf("Close all\n");
    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(init_odo);
    sem_close(mutex_odo);

    detach_memory_block_Odometrie(block);
    if (destroy_memory_block(FILENAME_ODO))
    {
        printf("Destroyed block: %s\n", FILENAME_ODO);
    }
    else
    {
        printf("Could not destroy block: %s\n", FILENAME_ODO);
    }
    close(sock);
    exit(0);

} 

int attachSemaphores()
{
    // Setup some semaphores
    sem_full_odo = sem_open(FULL_ODO, 0);
    if (sem_full_odo == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/producer");
        exit(EXIT_FAILURE);
    }

    sem_empty_odo = sem_open(EMPTY_ODO, 0);
    if (sem_empty_odo == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/consumer");
        exit(EXIT_FAILURE);
    }

    init_odo = sem_open(SEM_INIT_ODO, 0);
    if (init_odo == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/consumer");
        exit(EXIT_FAILURE);
    }

    mutex_odo = sem_open(MUTEX_ODO, 0);
    if (mutex_odo == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/consumer");
        exit(EXIT_FAILURE);
    }

    return 0;
}

void writeSharedMemory(struct SharedMemoryODO *block, struct SharedMemoryODO *data)
{
    block = attach_memory_block_Odometrie(FILENAME_ODO);
    if (block == NULL)
    {
        printf("Error: could not get block\n");
        // return -1;
    }
    block->testData = data->testData;
    printf("Writing: \"%d\"\n", block->testData);
   
}

int checkMessage(const std::string &buffer, const std::string &start_delimimter, const std::string &ende_delimimter)
{
   
    if(buffer.length() <= start_delimimter.length() || buffer.length() <= ende_delimimter.length() )
    {
        printf("Buffer lenght");
        return 0;
    }
    unsigned pos_start_delimimter = buffer.find(start_delimimter, 0);
    if(pos_start_delimimter == -1)
    {
        printf("del 1 not found");
        return 0;
    }
    unsigned pos_ende_delimimter = buffer.find(ende_delimimter, pos_start_delimimter);
    if(pos_ende_delimimter  == -1)
    {
        printf("del 2 not found");
        return 0;
    }
   
    return 1;
}

std::string getMessage(const std::string &buffer, const std::string &start_delimimter, const std::string &ende_delimimter)
{
    unsigned pos_start_delimimter = buffer.find(start_delimimter, 0);
    unsigned pos_ende_delimimter = buffer.find(ende_delimimter, pos_start_delimimter);
    printf("\n first delim pos: %d | last delim pos: %d \n", pos_start_delimimter, pos_ende_delimimter);

    return buffer.substr(pos_start_delimimter, pos_ende_delimimter + ende_delimimter.length() - pos_start_delimimter);
}

int main(int argc, char *argv[])
{

    struct sockaddr_in echoServAddr; /* Echo server address */
    unsigned short echoServPort;     /* Echo server port */
    char *servIP;                    /* Server IP address (dotted quad) */
    char const *echoString;          /* String to send to echo server */
    char echoBuffer[RCVBUFSIZE];     /* Buffer for echo string */
    unsigned int echoStringLen;      /* Length of string to echo */
    int bytesRcvd, totalBytesRcvd;   /* Bytes read in single recv()
                                        and total bytes read */

    if ((argc < 2) || (argc > 3)) /* Test for correct number of arguments */
    {
        fprintf(stderr, "Usage: %s <Server IP> [<Echo Port>]\n",
                argv[0]);
        exit(1);
    }

    servIP = argv[1];     /* First arg: server IP address (dotted quad) */
    echoString = argv[2]; /* Second arg: string to echo */

    if (argc == 3)
        echoServPort = atoi(argv[2]); /* Use given port, if any */
    else
        echoServPort = 9998; /* 7 is the well-known port for the echo service */

    /* Create a reliable, stream socket using TCP */

     //---------------------------------------------------------------------------

     

    /* Create a reliable, stream socket using TCP */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        printf("socket() failed");

    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
    echoServAddr.sin_family = AF_INET;                /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP); /* Server IP address */
    echoServAddr.sin_port = htons(echoServPort);      /* Server port */

    /* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
        printf("connect() failed");

    //---------------------------------------------------------------------------

    attachSemaphores();
    signal(SIGINT, signalHandler); // catch SIGINT
    
    sem_post(init_odo);
    
    if(SIMULATIONS_ON == 0)
    {
        close(sock);
    }
    

    struct SharedMemoryODO *test = new SharedMemoryODO();
    test->testData = 100;
   

    //---------------------------------------------------------------------------

    echoStringLen = strlen(echoString); /* Determine input length */

    /* Receive the same string back from the server */
    totalBytesRcvd = 0;
    int count = 0;
    printf("Received: "); /* Setup to print the echoed string */
    while (1)
    {
       
        sem_wait(sem_empty_odo);
        sem_wait(mutex_odo);



        if(SIMULATIONS_ON == 0)
        {
            
            if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
            printf("socket() failed");

            /* Construct the server address structure */
            memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
            echoServAddr.sin_family = AF_INET;                /* Internet address family */
            echoServAddr.sin_addr.s_addr = inet_addr(servIP); /* Server IP address */
            echoServAddr.sin_port = htons(echoServPort);      /* Server port */

            /* Establish the connection to the echo server */
            if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
                printf("connect() failed");
        }

        //nanosleep((const struct timespec[]){{0, 500000000L}}, NULL);
        

        if ((bytesRcvd = recv(sock, echoBuffer, RCVBUFSIZE - 1, 0)) <= 0)
            printf("recv() failed or connection closed prematurely");
        totalBytesRcvd += bytesRcvd;  
        echoBuffer[bytesRcvd] = '\0'; 
        printf("%s", echoBuffer);      
        if(checkMessage(echoBuffer, "--START---", "___END___") == 1)
        {
            std::cout << getMessage(echoBuffer, "--START---", "___END___") << std::endl;
            test->testData++;
          
        }
        else
        {
            count++;
        }
        
        if(SIMULATIONS_ON == 0)
        {
            close(sock);
        }
        
        
        writeSharedMemory(block, test);
        detach_memory_block_Odometrie(block);

        sem_post(mutex_odo);
        sem_post(sem_full_odo);

        printf("\nwaiting\n");
        printf("\n count = %d\n", count);
    }
    printf("\n"); /* Print a final linefeed */

    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(init_odo);
    sem_close(mutex_odo);

    detach_memory_block_Odometrie(block);
    close(sock);
    exit(0);
}
