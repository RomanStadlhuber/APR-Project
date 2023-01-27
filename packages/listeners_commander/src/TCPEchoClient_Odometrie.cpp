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

int sock_listener; // socket listener
// Used Semaphores
sem_t *sem_full_odo;
sem_t *sem_empty_odo;
sem_t *init_odo;
sem_t *mutex_odo;
// Used SharedMemory-Block
struct SharedMemoryODO *block;

#define BUFFER_SIZE 5000 // Size of buffer

void signalHandler(int sig)
{
    // Close alle sempahores and detach from shared memory block
    printf("Close all\n");
    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(init_odo);
    sem_close(mutex_odo);

    detach_shared_memory_Odometrie(block);
    if (remove_shared_memory(FILENAME_ODO))
    {
        printf("Removed shared memory block: %s\n", FILENAME_ODO);
    }
    else
    {
        printf("Could not remove shared memory block: %s\n", FILENAME_ODO);
    }
    // Close socket
    close(sock_listener);
    exit(0);
}

int attachSemaphores()
{
    // Open used semaphores
    sem_full_odo = sem_open(FULL_ODO, 0);
    if (sem_full_odo == SEM_FAILED)
    {
        printf("open semaphore \"sem_full_odo\" failed");
        exit(EXIT_FAILURE);
    }

    sem_empty_odo = sem_open(EMPTY_ODO, 0);
    if (sem_empty_odo == SEM_FAILED)
    {
        printf("open semaphore \"sem_empty_odo\" failed");
        exit(EXIT_FAILURE);
    }

    init_odo = sem_open(SEM_INIT_ODO, 0);
    if (init_odo == SEM_FAILED)
    {
        printf("open semaphore \"init_odo\" failed");
        exit(EXIT_FAILURE);
    }

    mutex_odo = sem_open(MUTEX_ODO, 0);
    if (mutex_odo == SEM_FAILED)
    {
        printf("open semaphore \"mutex_odo\" failed");
        exit(EXIT_FAILURE);
    }

    return 0;
}

void writeSharedMemory(struct SharedMemoryODO *block, struct SharedMemoryODO *data)
{
    block = attach_shared_memory_Odometrie(FILENAME_ODO);
    if (block == NULL)
    {
        printf("Attaching shared memory block Odo not successfull\n");
    }
    
    block->testData = data->testData;
    block->header = data->header;
    block->pose = data->pose;
    printf("Writing: \"%d\"\n", block->testData);
}

int checkMessage(const std::string &buffer, const std::string &start_delimimter, const std::string &ende_delimimter)
{

    if (buffer.length() <= start_delimimter.length() || buffer.length() <= ende_delimimter.length())
    {
        printf("Error Buffer lenght");
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

    // get the values from the j_msg_O String into the msgO Struct wit 'get_to()'
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

    struct sockaddr_in server_addr; // server address
    unsigned short echoServPort;     // port
    char *server_IP;                    // Server IP address
    char recv_buffer[BUFFER_SIZE];     // Buffer
    int bytes_recivied, total_bytes_recivied;

    if ((argc < 2) || (argc > 3)) // Check number of arguments
    {
        fprintf(stderr, "Usage: %s <Server IP> [<Echo Port>]\n",
                argv[0]);
        exit(1);
    }

    server_IP = argv[1]; // Get server IP address from argv[1]

    if (argc == 3)
        echoServPort = atoi(argv[2]); // Get port address from argv[2]
    else
        echoServPort = 9998; // default port ardess

    //---------------------------------------------------------------------------

    // Create socket using TCP for sending to server
    if ((sock_listener = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        printf("socket() for reciving odo data failed");

    // Structure to connect to server
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(server_IP); // IP address from Server
    server_addr.sin_port = htons(echoServPort);      // Port adress

    // Connect to echo server
    if (connect(sock_listener, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        printf("connect() for reciving odo data failed");

    //---------------------------------------------------------------------------

    attachSemaphores();            // attached to all semaphores
    signal(SIGINT, signalHandler); // catch SIGINT
    // initial Sempahore - post to show that the listener is ready
    sem_post(init_odo);
    // Close socket on the live-systme | let the socket open in the simulation
    if (SIMULATIONS_ON == 0)
    {
        close(sock_listener);
    }
    // data for the shared memory
    struct SharedMemoryODO *dataOdom = new SharedMemoryODO();
    dataOdom->testData = 100;

    //---------------------------------------------------------------------------

    total_bytes_recivied = 0;
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
            if ((sock_listener = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
                printf("socket() failed");

            // Structure to connect to server
            memset(&server_addr, 0, sizeof(server_addr));
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = inet_addr(server_IP);
            server_addr.sin_port = htons(echoServPort);

            // Connect to echo server
            if (connect(sock_listener, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
                printf("connect() failed");
        }

        // nanosleep((const struct timespec[]){{0, 500000000L}}, NULL);

        if ((bytes_recivied = recv(sock_listener, recv_buffer, BUFFER_SIZE - 1, 0)) <= 0)
            printf("recv() failed or connection closed prematurely");
        total_bytes_recivied += bytes_recivied;
        recv_buffer[bytes_recivied] = '\0';
        // printf("%s", recv_buffer);
        if (checkMessage(recv_buffer, "--START---", "___END___") == 1)
        {

            // get message to string for parsing
            std::string tempStrng = getMessage(recv_buffer, "--START---", "___END___"); 

            tempStrng.erase(0, 10);                // delete --START--- for parsing
            tempStrng.erase(tempStrng.size() - 9); // delete ___END___  for parsing

            // parse tempStrng to j_msg_O with nlohmann library
            nlohmann::json_abi_v3_11_2::json j_msg_O = nlohmann::json_abi_v3_11_2::json::parse(tempStrng); 

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
            close(sock_listener);
        }
        // attach to shared memroy odo and write data to shared memory odo
        writeSharedMemory(block, dataOdom); 
        // detach from shared memory
        detach_shared_memory_Odometrie(block);
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

    detach_shared_memory_Odometrie(block);
    // Close socket
    close(sock_listener);
    exit(0);
}
