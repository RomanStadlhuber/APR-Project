#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/sem.h>
#include <iostream>
#include <semaphore.h>
#include <fcntl.h>
#include <signal.h>
#include <iomanip>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include "json.hpp"

#include <shared_memory.hpp>
// #include "shared_memory.hpp"

// #define BLOCK_SIZE 4096

int sock; // socket
// Used Semaphores
sem_t *sem_full_odo;
sem_t *sem_empty_odo;
sem_t *init_odo;
sem_t *mutex_odo;
// Used SharedMemory-Block
struct SharedMemoryODO *block;

#define RCVBUFSIZE 5000 // Size of buffer

void signalHandler(int sig)
{
    // Close alle sempahores and detach from shared memory block
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
    // Close socket
    close(sock);
    exit(0);
}

int attachSemaphores()
{
    // Open used semaphores
    sem_full_odo = sem_open(FULL_ODO, 0);
    if (sem_full_odo == SEM_FAILED)
    {
        printf("open Semaphore \"sem_full_odo\" failed");
        exit(EXIT_FAILURE);
    }

    sem_empty_odo = sem_open(EMPTY_ODO, 0);
    if (sem_empty_odo == SEM_FAILED)
    {
        printf("open Semaphore \"sem_empty_odo\" failed");
        exit(EXIT_FAILURE);
    }

    init_odo = sem_open(SEM_INIT_ODO, 0);
    if (init_odo == SEM_FAILED)
    {
        printf("open Semaphore \"init_odo\" failed");
        exit(EXIT_FAILURE);
    }

    mutex_odo = sem_open(MUTEX_ODO, 0);
    if (mutex_odo == SEM_FAILED)
    {
        printf("open Semaphore \"mutex_odo\" failed");
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
    }
    // Casper: hier die Daten raufschreiben auf den block für SharedMemroy
    block->testData = data->testData;
    block->header = data->header;
    block->pose = data->pose;
    printf("Writing: \"%d\"\n", block->testData);
}

int checkMessage(const std::string &buffer, const std::string &start_delimimter, const std::string &ende_delimimter)
{

    if (buffer.length() <= start_delimimter.length() || buffer.length() <= ende_delimimter.length())
    {
        printf("Buffer lenght");
        return 0;
    }
    unsigned pos_start_delimimter = buffer.find(start_delimimter, 0);
    if (pos_start_delimimter == -1)
    {
        printf("del 1 not found");
        return 0;
    }
    unsigned pos_ende_delimimter = buffer.find(ende_delimimter, pos_start_delimimter);
    if (pos_ende_delimimter == -1)
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

void json2Struct(nlohmann::json_abi_v3_11_2::json j_msg_O, struct SharedMemoryODO *msgO) // get parsed json file to struct
{

    j_msg_O["header"]["seq"].get_to(msgO->header.seq);
    j_msg_O["header"]["stamp"]["secs"].get_to(msgO->header.stamp.secs);
    j_msg_O["header"]["stamp"]["nsecs"].get_to(msgO->header.stamp.nsecs);
    j_msg_O["header"]["frame_id"].get_to(msgO->header.frame_id);
    j_msg_O["child_frame_id"].get_to(msgO->child_frame_id);
    struct oriCoordinates quat;
    struct posiCoordinates pos;
    j_msg_O["pose"]["pose"]["position"]["x"].get_to(pos.x);
    j_msg_O["pose"]["pose"]["position"]["y"].get_to(pos.y);
    j_msg_O["pose"]["pose"]["position"]["z"].get_to(pos.z);
    msgO->pose.position = Eigen::Vector3d(pos.x, pos.y, pos.z);
    j_msg_O["pose"]["pose"]["orientation"]["x"].get_to(quat.x);
    j_msg_O["pose"]["pose"]["orientation"]["y"].get_to(quat.y);
    j_msg_O["pose"]["pose"]["orientation"]["z"].get_to(quat.z);
    j_msg_O["pose"]["pose"]["orientation"]["w"].get_to(quat.w);
    msgO->pose.orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

void outputOdomStruct(struct SharedMemoryODO *msgO) // test ouuput of Odom Struct
{
    std::cout << std::endl
              << std::endl
              << "Scan Id: \t\t" << msgO->header.frame_id << std::endl
              << "Seq. Nr: \t\t" << msgO->header.seq << std::endl
              << "\t Sek.: \t\t\t" << msgO->header.stamp.secs << std::endl
              << "\t nano Sek.: \t\t" << msgO->header.stamp.nsecs << std::endl
              << "Position:" << std::endl
              << "\t X:\t" << msgO->pose.position.x() << std::endl
              << "\t Y:\t" << msgO->pose.position.y() << std::endl
              << "\t Z:\t" << msgO->pose.position.z() << std::endl
              << "Orientation:" << std::endl
              << "\t X:\t" << msgO->pose.orientation.x() << std::endl
              << "\t Y:\t" << msgO->pose.orientation.y() << std::endl
              << "\t Z:\t" << msgO->pose.orientation.z() << std::endl
              << "\t W:\t" << msgO->pose.orientation.w() << std::endl;
    std::cout << "Covariance at 0 - 9:" << std::endl;
}

int main(int argc, char *argv[])
{

    struct sockaddr_in echoServAddr; // server address
    unsigned short echoServPort;     // port
    char *servIP;                    // Server IP address
    char echoBuffer[RCVBUFSIZE];     // Buffer
    int bytesRcvd, totalBytesRcvd;

    if ((argc < 2) || (argc > 3)) // Check number of arguments
    {
        fprintf(stderr, "Usage: %s <Server IP> [<Echo Port>]\n",
                argv[0]);
        exit(1);
    }

    servIP = argv[1]; // Get server IP address from argv[1]

    if (argc == 3)
        echoServPort = atoi(argv[2]); // Get port address from argv[2]
    else
        echoServPort = 9998; // default port ardess

    //---------------------------------------------------------------------------

    // Create socket using TCP for sending to server
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        printf("socket() for reciving odo data failed");

    // Structure to connect to server
    memset(&echoServAddr, 0, sizeof(echoServAddr));
    echoServAddr.sin_family = AF_INET;
    echoServAddr.sin_addr.s_addr = inet_addr(servIP); // IP address from Server
    echoServAddr.sin_port = htons(echoServPort);      // Port adress

    // Connect to echo server
    if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
        printf("connect() for reciving odo data failed");

    //---------------------------------------------------------------------------

    attachSemaphores();            // attached to all semaphores
    signal(SIGINT, signalHandler); // catch SIGINT
    // initial Sempahore - post to show that the listener is ready
    sem_post(init_odo);
    // Close socket on the live-systme | let the socket open in the simulation
    if (SIMULATIONS_ON == 0)
    {
        close(sock);
    }
    // data for the shared memory
    struct SharedMemoryODO *dataOdom = new SharedMemoryODO();
    dataOdom->testData = 100;

    //---------------------------------------------------------------------------

    totalBytesRcvd = 0;
    int count = 0; // counter for not correct received messages
    printf("Start receiving: ");
    while (1)
    {
        // Wait for commander
        sem_wait(sem_empty_odo);
        // Wait for mutex - critical section
        sem_wait(mutex_odo);

        if (SIMULATIONS_ON == 0) // ckeck if simulation or live-system
        {
            // Create socket using TCP for sending to server
            if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
                printf("socket() failed");

            // Structure to connect to server
            memset(&echoServAddr, 0, sizeof(echoServAddr));
            echoServAddr.sin_family = AF_INET;
            echoServAddr.sin_addr.s_addr = inet_addr(servIP);
            echoServAddr.sin_port = htons(echoServPort);

            // Connect to echo server
            if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
                printf("connect() failed");
        }

        // nanosleep((const struct timespec[]){{0, 500000000L}}, NULL);

        if ((bytesRcvd = recv(sock, echoBuffer, RCVBUFSIZE - 1, 0)) <= 0)
            printf("recv() failed or connection closed prematurely");
        totalBytesRcvd += bytesRcvd;
        echoBuffer[bytesRcvd] = '\0';
        // printf("%s", echoBuffer);
        if (checkMessage(echoBuffer, "--START---", "___END___") == 1)
        {

            std::string tempStrng = getMessage(echoBuffer, "--START---", "___END___"); // get message to string for parsing

            tempStrng.erase(0, 10);                // delete --START--- for parsing
            tempStrng.erase(tempStrng.size() - 9); // delete ___END___  for parsing

            nlohmann::json_abi_v3_11_2::json j_msg_O = nlohmann::json_abi_v3_11_2::json::parse(tempStrng); // parse tempStrng to j_msg_O

            json2Struct(j_msg_O, dataOdom); // get parsed json file to struct

            outputOdomStruct(dataOdom); // Test ouput

            dataOdom->testData++;
        }
        else
        {
            count++;
        }

        if (SIMULATIONS_ON == 0) // ckeck if simulation or live-system
        {
            close(sock);
        }
        // attach to shared memroy odo and write data to shared memory odo
        writeSharedMemory(block, dataOdom); // Casper: in dieser Funktion werden die Daten des structs auf den SharedMemory geschrieben, bitte diese Funktion anpassen
        // detach from shared memory
        detach_memory_block_Odometrie(block);
        // Post mutex - writing finished
        sem_post(mutex_odo);
        // Post lidar/commander - all data written
        sem_post(sem_full_odo);

        printf("waiting for commander\n");
        printf("\n count = %d\n", count);
    }
    printf("\n");
    // Close alle sempahores and detach from shared memory block
    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(init_odo);
    sem_close(mutex_odo);

    detach_memory_block_Odometrie(block);
    // Close socket
    close(sock);
    exit(0);
}
