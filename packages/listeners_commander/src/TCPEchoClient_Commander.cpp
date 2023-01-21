#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <iostream>
#include <sstream>
#include <string>
#include <sys/sem.h>
#include <semaphore.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <shared_memory.hpp>
//#include "shared_memory.hpp"

#define BLOCK_SIZE 4096

int sock; /* Socket descriptor */
sem_t *sem_full_lidar;
sem_t *sem_empty_lidar;
sem_t *sem_full_odo;
sem_t *sem_empty_odo;
sem_t *init_lidar;
sem_t *init_odo;
sem_t *mutex_lidar;
sem_t *mutex_odo;
struct SharedMemoryLIDAR *block;
struct SharedMemoryODO *block2;

// Compile mit:     g++ TCPEchoClient_Commander.cpp -lpthread -o TCPEchoClient_Commander
// Start mit:       ./TCPEchoClient_Commander 127.0.0.1 10001

void signalHandler(int sig)
{
    printf("Close all\n");
    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(init_lidar);
    sem_close(init_odo);
    sem_close(mutex_lidar);
    sem_close(mutex_odo);

    detach_memory_block_LIDAR(block);
    detach_memory_block_Odometrie(block2);

    close(sock);
    exit(0);

} // end producerHandler


void greateSemahpore()
{
    sem_unlink(EMPTY_LIDAR);
    sem_unlink(FULL_ODO);
    sem_unlink(EMPTY_ODO);
    sem_unlink(FULL_LIDAR);
    sem_unlink(MUTEX_LIDAR);
    sem_unlink(SEM_INIT_LIDAR);
    sem_unlink(SEM_INIT_ODO);
     sem_unlink(MUTEX_ODO);

//test - comment
    init_lidar = sem_open(SEM_INIT_LIDAR, O_CREAT, 0777, 0);
    if (init_lidar == SEM_FAILED)
    {
        // perror("sem_open/producer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    init_odo = sem_open(SEM_INIT_ODO, O_CREAT, 0777, 0);
    if (init_odo == SEM_FAILED)
    {
        // perror("sem_open/producer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    sem_full_lidar = sem_open(FULL_LIDAR, O_CREAT, 0777, 0);
    if (sem_full_lidar == SEM_FAILED)
    {
        // perror("sem_open/producer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    // printf("open Semaphore succesfull\n");
    sem_full_odo = sem_open(FULL_ODO, O_CREAT, 0777, 0);
    if (sem_full_odo == SEM_FAILED)
    {
        // perror("sem_open/producer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    sem_empty_lidar = sem_open(EMPTY_LIDAR, O_CREAT, 0777, 1);
    if (sem_empty_lidar == SEM_FAILED)
    {
        // perror("sem_open/consumer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    sem_empty_odo = sem_open(EMPTY_ODO, O_CREAT, 0777, 1);
    if (sem_empty_odo == SEM_FAILED)
    {
        // perror("sem_open/consumer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    mutex_lidar = sem_open(MUTEX_LIDAR, O_CREAT, 0777, 1);
    if (mutex_lidar == SEM_FAILED)
    {
        // perror("sem_open/consumer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

     mutex_odo = sem_open(MUTEX_ODO, O_CREAT, 0777, 1);
    if (mutex_odo == SEM_FAILED)
    {
        // perror("sem_open/consumer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

}

struct SharedMemoryLIDAR *readSharedMemoryLidar()
{
    printf("Herer...\n");
    block = attach_memory_block_LIDAR(FILENAME_LIDAR);
    if (block == NULL)
    {
        printf("Error: could not get block\n");
        // return -1;
    }
    // printf("Reading: \"%s\"\n", block);
    return block;
}

struct SharedMemoryODO *readSharedMemoryOdometrie()
{
    printf("Herer...\n");
    block2 = attach_memory_block_Odometrie(FILENAME_ODO);
    if (block2 == NULL)
    {
        printf("Error: could not get block\n");
        // return -1;
    }
    // printf("Reading: \"%s\"\n", block);
    return block2;
}


void connectSocket(struct sockaddr_in echoServAddr, unsigned short echoServPort, char *servIP)
{
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

}

int main(int argc, char *argv[])
{
   
    struct sockaddr_in echoServAddr;        /* Echo server address */
    unsigned short echoServPort;            /* Echo server port */
    char *servIP;                           /* Server IP address (dotted quad) */
    unsigned int bytesRcvd, totalBytesRcvd; /* Bytes read in single recv()  // changed to unsigned int due to comparison problems
                                      and total bytes read */

    if ((argc < 3) || (argc > 3)) /* Test for correct number of arguments */
    {
        fprintf(stderr, "Usage: %s <Server IP> [<Echo Port>]\n",
                argv[0]);
        exit(1);
    }

    servIP = argv[1]; /* First arg: server IP address (dotted quad) */

    if (argc == 3)
        echoServPort = atoi(argv[2]); /* Use given port, if any */
    else
        echoServPort = 9999; /* 7 is the well-known port for the echo service */

 

    connectSocket(echoServAddr, echoServPort, servIP);
    close(sock);

    //------------------------------------------------------------------------

    greateSemahpore();

    signal(SIGINT, signalHandler); // catch SIGINT
    
    sem_wait(init_lidar);
    sem_wait(init_odo);
  

    while (1)
    {
        
        
        printf("Waiting Lidar...\n");
                
        sem_wait(sem_full_lidar);
        sem_wait(mutex_lidar);

        block = readSharedMemoryLidar();
        
        printf("Reading Lidar: \"%d\"\n", block->testData);

        detach_memory_block_LIDAR(block);

        sem_post(mutex_lidar);
        sem_post(sem_empty_lidar);


        printf("Waiting Odo...\n");

        
                
        sem_wait(sem_full_odo);
        sem_wait(mutex_odo);

         printf("Wait...sh\n");

        block2 = readSharedMemoryOdometrie();
        
        printf("Reading Odo: \"%d\"\n", block2->testData);

        detach_memory_block_Odometrie(block2);

        sem_post(mutex_odo);
        sem_post(sem_empty_odo);

      

        

      
        //------------------------------------------------------------------------
        // Great Message-String
        float lin = 0;
        float angular = 0;
        std::ostringstream oss;
        //"---START---{\"linear\": 0.1, \"angular\": 0.10}___END___\0";
        oss << "---START---{\"linear\": " << lin << ", \"angular\": " << angular << "}___END___\0";
        // std::cout << oss.str();
        std::string str = oss.str();
        const char *echoString = str.c_str();
        unsigned int echoStringLen = strlen(echoString); /* Length of string to echo */

        //------------------------------------------------------------------------
        connectSocket(echoServAddr, echoServPort, servIP);
       
        /* Send the string to the server */
        if (send(sock, echoString, echoStringLen, 0) != echoStringLen)
            printf("send() sent a different number of bytes than expected");

        printf("%s\n", echoString);
        close(sock);

        //sem_post(sem_empty_lidar);
        
    }

    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(init_lidar);
    sem_close(init_odo);

    detach_memory_block_LIDAR(block);
    detach_memory_block_Odometrie(block2);
    close(sock);
    exit(0);
}