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

#define BLOCK_SIZE 4096

int sock; /* Socket descriptor */
sem_t *sem_prod;
sem_t *sem_cons;
sem_t *sem_prod2;
sem_t *sem_cons2;
struct SharedMemoryLIDAR *block;
struct SharedMemoryODO *block2;

// Compile mit:     g++ TCPEchoClient_Commander.cpp -lpthread -o TCPEchoClient_Commander
// Start mit:       ./TCPEchoClient_Commander 127.0.0.1 10001

void signalHandler(int sig)
{
    printf("Close all\n");
    sem_close(sem_cons2);
    sem_close(sem_prod2);
    sem_close(sem_cons);
    sem_close(sem_prod);

    detach_memory_block_LIDAR(block);
    detach_memory_block_Odometrie(block2);

    close(sock);
    exit(0);

} // end producerHandler

void greateSemahpore()
{
    sem_unlink(SEM_CONSUMER_FNAME);
    sem_unlink(SEM_PRODUCER_FNAME);
    sem_unlink(SEM_CONSUMER2_FNAME);
    sem_unlink(SEM_PRODUCER2_FNAME);

    sem_prod = sem_open(SEM_PRODUCER_FNAME, O_CREAT, 0660, 0);
    if (sem_prod == SEM_FAILED)
    {
        // perror("sem_open/producer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    sem_cons = sem_open(SEM_CONSUMER_FNAME, O_CREAT, 0660, 1);
    if (sem_cons == SEM_FAILED)
    {
        // perror("sem_open/consumer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }
    // printf("open Semaphore succesfull\n");
    sem_prod2 = sem_open(SEM_PRODUCER2_FNAME, O_CREAT, 0660, 0);
    if (sem_prod2 == SEM_FAILED)
    {
        // perror("sem_open/producer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }

    sem_cons2 = sem_open(SEM_CONSUMER2_FNAME, O_CREAT, 0660, 1);
    if (sem_cons2 == SEM_FAILED)
    {
        // perror("sem_open/consumer");
        printf("open Semaphore failed\n");
        exit(EXIT_FAILURE);
    }
}

struct SharedMemoryLIDAR *readSharedMemoryLidar()
{
    printf("Herer...\n");
    block = attach_memory_block_LIDAR(FILENAME);
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
    block2 = attach_memory_block_Odometrie(FILENAME2);
    if (block2 == NULL)
    {
        printf("Error: could not get block\n");
        // return -1;
    }
    // printf("Reading: \"%s\"\n", block);
    return block2;
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
        echoServPort = 7; /* 7 is the well-known port for the echo service */

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

    //------------------------------------------------------------------------

    greateSemahpore();
    signal(SIGINT, signalHandler); // catch SIGINT
    struct SharedMemoryLIDAR *test;
    struct SharedMemoryODO *test2;

    while (1)
    {
        printf("Waiting...\n");

        sem_wait(sem_prod);
        test = readSharedMemoryLidar();

        printf("Reading Lidar: \"%d\"\n", test->testData);

        sem_post(sem_cons2);

        sem_wait(sem_prod2);
        test2 = readSharedMemoryOdometrie();

        printf("Reading Odometrie: \"%d\"\n", test2->testData);

        //------------------------------------------------------------------------
        // Great Message-String
        float lin = 0.3;
        float angular = 1;
        std::ostringstream oss;
        //"---START---{\"linear\": 0.1, \"angular\": 0.10}___END___\0";
        oss << "---START---{\"linear\": " << lin << ", \"angular\": " << angular << "}___END___\0";
        // std::cout << oss.str();
        std::string str = oss.str();
        const char *echoString = str.c_str();
        unsigned int echoStringLen = strlen(echoString); /* Length of string to echo */

        //------------------------------------------------------------------------

        /* Send the string to the server */
        if (send(sock, echoString, echoStringLen, 0) != echoStringLen)
            printf("send() sent a different number of bytes than expected");

        printf("%s\n", echoString);
        sem_post(sem_cons);
    }

    sem_close(sem_cons);
    sem_close(sem_prod);
    sem_close(sem_cons2);
    sem_close(sem_prod2);

    detach_memory_block_LIDAR(block);
    detach_memory_block_Odometrie(block2);
    close(sock);
    exit(0);
}