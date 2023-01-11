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

#define BLOCK_SIZE 4096

int sock; /* Socket descriptor */
sem_t *sem_prod;
sem_t *sem_cons;
struct SharedMemoryLIDAR *block;

#define RCVBUFSIZE 14000 /* Size of receive buffer */

// Compile with:  g++ TCPEchoClient_Lidar.cpp shared_memory.cpp -lpthread  -o TCPEchoClient_Lidar
// Start with: ./TCPEchoClient_Lidar

void signalHandler(int sig)
{
    printf("Close all\n");
    sem_close(sem_cons);
    sem_close(sem_prod);

    detach_memory_block_LIDAR(block);
    if (destroy_memory_block(FILENAME))
    {
        printf("Destroyed block: %s\n", FILENAME);
    }
    else
    {
        printf("Could not destroy block: %s\n", FILENAME);
    }
    close(sock);
    exit(0);

} // end producerHandler

int greatSharedMemory()
{
    // Setup some semaphores
    sem_prod = sem_open(SEM_PRODUCER_FNAME, 0);
    if (sem_prod == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/producer");
        exit(EXIT_FAILURE);
    }

    sem_cons = sem_open(SEM_CONSUMER_FNAME, 0);
    if (sem_cons == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/consumer");
        exit(EXIT_FAILURE);
    }

    // grab the shared memory block

    block = attach_memory_block_LIDAR(FILENAME);
    if (block == NULL)
    {
        printf("Error: could not get block\n");
        return -1;
    }

    return 0;
}

void writeSharedMemory(struct SharedMemoryLIDAR *block, struct SharedMemoryLIDAR *data)
{
    block->testData = data->testData;
    printf("Writing: \"%d\"\n", block->testData);
    // strncpy(block, testChar, BLOCK_SIZE);
}

std::string getMessage(const std::string &buffer, const std::string &start_delimimter, const std::string &ende_delimimter)
{
    // std::cout << s << std::endl;
    unsigned pos_start_delimimter = buffer.find(start_delimimter, 0);
    unsigned pos_ende_delimimter = buffer.find(ende_delimimter, pos_start_delimimter);
    printf("\n first delim pos: %d | last delim pos: %d \n", pos_start_delimimter, pos_ende_delimimter);

    return buffer.substr(pos_start_delimimter, pos_ende_delimimter + ende_delimimter.length() - pos_start_delimimter);
}

int main(int argc, char *argv[])
{
    // int sock;                        /* Socket descriptor */
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

    //---------------------------------------------------------------------------
    sleep(1);
    greatSharedMemory();
    signal(SIGINT, signalHandler); // catch SIGINT

    struct SharedMemoryLIDAR *test = new SharedMemoryLIDAR();
    test->testData = 5;

    //---------------------------------------------------------------------------

    echoStringLen = strlen(echoString); /* Determine input length */

    /* Receive the same string back from the server */
    totalBytesRcvd = 0;
    printf("Received: "); /* Setup to print the echoed string */
    while (1)
    {
        /* Receive up to the buffer size (minus 1 to leave space for
           a null terminator) bytes from the sender */

        sem_wait(sem_cons);
        if ((bytesRcvd = recv(sock, echoBuffer, RCVBUFSIZE - 1, 0)) <= 0)
            printf("recv() failed or connection closed prematurely");
        totalBytesRcvd += bytesRcvd;  /* Keep tally of total bytes */
        echoBuffer[bytesRcvd] = '\0'; /* Terminate the string! */
        // printf("%s", echoBuffer);      /* Print the echo buffer */
        std::cout << echoBuffer << std::endl;
        std::cout << getMessage(echoBuffer, "--START---", "___END___") << std::endl;

        writeSharedMemory(block, test);
        test->testData++;
        sem_post(sem_prod);
        printf("waiting\n");
    }
    printf("\n"); /* Print a final linefeed */

    sem_close(sem_cons);
    sem_close(sem_prod);

    detach_memory_block_LIDAR(block);
    close(sock);
    exit(0);
}
