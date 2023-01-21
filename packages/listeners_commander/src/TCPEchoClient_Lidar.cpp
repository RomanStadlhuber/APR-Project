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
sem_t *sem_full_lidar;
sem_t *sem_empty_lidar;
sem_t *init_lidar;
sem_t *mutex_lidar;
struct SharedMemoryLIDAR *block;

#define RCVBUFSIZE 8000 /* Size of receive buffer */


void signalHandler(int sig)
{
    printf("Close all\n");
    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(init_lidar);
    sem_close(mutex_lidar);

    detach_memory_block_LIDAR(block);
    if (destroy_memory_block(FILENAME_LIDAR))
    {
        printf("Destroyed block: %s\n", FILENAME_LIDAR);
    }
    else
    {
        printf("Could not destroy block: %s\n", FILENAME_LIDAR);
    }
    close(sock);
    exit(0);

} 

int attachSemahpore()
{
    // Setup some semaphores
    sem_full_lidar = sem_open(FULL_LIDAR, 0);
    if (sem_full_lidar == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/producer");
        exit(EXIT_FAILURE);
    }

    sem_empty_lidar = sem_open(EMPTY_LIDAR, 0);
    if (sem_empty_lidar == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/consumer");
        exit(EXIT_FAILURE);
    }

    init_lidar = sem_open(SEM_INIT_LIDAR, 0);
    if (init_lidar == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/consumer");
        exit(EXIT_FAILURE);
    }

    mutex_lidar = sem_open(MUTEX_LIDAR, 0);
    if (mutex_lidar == SEM_FAILED)
    {
        printf("open Semaphore failed");
        // perror("sem_open/consumer");
        exit(EXIT_FAILURE);
    }

    return 0;
}

void writeSharedMemory(struct SharedMemoryLIDAR *block, struct SharedMemoryLIDAR *data)
{   
    block = attach_memory_block_LIDAR(FILENAME_LIDAR);
    if (block == NULL)
    {
        printf("Error: could not get block\n");
        // return -1;
    }
    //Casper: hier die Daten raufschreiben auf den block für SharedMemroy
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
        echoServPort = 9997; /* 7 is the well-known port for the echo service */

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
  
    attachSemahpore();
    signal(SIGINT, signalHandler); // catch SIGINT


    sem_post(init_lidar);

    if(SIMULATIONS_ON == 0)
    {
        close(sock);
    }
    
    struct SharedMemoryLIDAR *test = new SharedMemoryLIDAR();
    test->testData = 5;

    //---------------------------------------------------------------------------

    echoStringLen = strlen(echoString); /* Determine input length */
    int count = 0;
    /* Receive the same string back from the server */
    totalBytesRcvd = 0;
    printf("Received: "); /* Setup to print the echoed string */
    while (1)
    {
        /* Receive up to the buffer size (minus 1 to leave space for
           a null terminator) bytes from the sender */
        
        sem_wait(sem_empty_lidar);
        sem_wait(mutex_lidar);

        if(SIMULATIONS_ON == 0)
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
        
        //nanosleep((const struct timespec[]){{0, 500000000L}}, NULL);

        if ((bytesRcvd = recv(sock, echoBuffer, RCVBUFSIZE - 1, 0)) <= 0)
            printf("recv() failed or connection closed prematurely");
        totalBytesRcvd += bytesRcvd;  /* Keep tally of total bytes */
        echoBuffer[bytesRcvd] = '\0'; /* Terminate the string! */
        // printf("%s", echoBuffer);      /* Print the echo buffer */
        std::cout << echoBuffer << std::endl;
        std::cout << "______________________________-" << std::endl;
        
        if(checkMessage(echoBuffer, "--START---", "___END___") == 1)
        {
            std::cout << getMessage(echoBuffer, "--START---", "___END___") << std::endl; //Casper: mit der getMessage() Funktion bekommst du deinen String zum Parsen (Lidar)
            //Casper: Am besten dann in ein vorrübergehendes struct hier speichern
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
        
        writeSharedMemory(block, test); //Casper: in dieser Funktion werden die Daten des structs auf den SharedMemory geschrieben, bitte diese Funktion anpassen
        detach_memory_block_LIDAR(block);
        sem_post(mutex_lidar);
        sem_post(sem_full_lidar);

        printf("waiting\n");
        printf("\n count = %d\n", count);
      
        
    }
    printf("\n"); /* Print a final linefeed */

    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(init_lidar);
    sem_close(mutex_lidar);


    detach_memory_block_LIDAR(block);
    close(sock);
    exit(0);
}
