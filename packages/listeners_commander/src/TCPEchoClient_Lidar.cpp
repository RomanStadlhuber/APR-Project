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
#include <CircleDetection.hpp>
#include <shared_memory.hpp>
// #include "shared_memory.hpp"

int sock; //socket
//Used Semaphores
sem_t *sem_full_lidar;
sem_t *sem_empty_lidar;
sem_t *init_lidar;
sem_t *mutex_lidar;
//Used SharedMemory-Block
struct SharedMemoryLIDAR *block;

#define RCVBUFSIZE 10000 //Size of buffer

void signalHandler(int sig)
{
    //Close alle sempahores and detach from shared memory block
    printf("Close all\n");
    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(init_lidar);
    sem_close(mutex_lidar);

    detach_shared_memory_LIDAR(block);
    if (remove_shared_memory(FILENAME_LIDAR))
    {
        printf("Removed shared memory block: %s\n", FILENAME_LIDAR);
    }
    else
    {
        printf("Could not remove shared memory block: %s\n", FILENAME_LIDAR);
    }
    //Close socket
    close(sock);
    exit(0);
}

int attachSemaphore()
{
    //Open used semaphores
    sem_full_lidar = sem_open(FULL_LIDAR, 0);
    if (sem_full_lidar == SEM_FAILED)
    {
        printf("open Semaphore \"sem_full_lidar\" failed");
        exit(EXIT_FAILURE);
    }

    sem_empty_lidar = sem_open(EMPTY_LIDAR, 0);
    if (sem_empty_lidar == SEM_FAILED)
    {
        printf("open Semaphore \"sem_empty_lidar\" failed");
        exit(EXIT_FAILURE);
    }

    init_lidar = sem_open(SEM_INIT_LIDAR, 0);
    if (init_lidar == SEM_FAILED)
    {
        printf("open Semaphore \"init_lidar\" failed");
        exit(EXIT_FAILURE);
    }

    mutex_lidar = sem_open(MUTEX_LIDAR, 0);
    if (mutex_lidar == SEM_FAILED)
    {
        printf("open Semaphore \"mutex_lidar\" failed");
        exit(EXIT_FAILURE);
    }

    return 0;
}

void writeSharedMemory(struct SharedMemoryLIDAR *block, struct SharedMemoryLIDAR *data)
{
    block = attach_shared_memory_LIDAR(FILENAME_LIDAR);
    if (block == NULL)
    {
        printf("Attaching shared memory block LIDAR not successfull\n");
    }
    // Casper: hier die Daten raufschreiben auf den block für SharedMemroy
    block->testData = data->testData;
    block->relative_landmark_position = data->relative_landmark_position;
    printf("Writing: \"%d\"\n", block->testData);
}

int checkMessage(const std::string &buffer, const std::string &start_delimimter, const std::string &ende_delimimter)
{

    if (buffer.length() <= start_delimimter.length() || buffer.length() <= ende_delimimter.length())
    {
        printf("Error buffer lenght");
        return 0;
    }
    unsigned pos_start_delimimter = buffer.find(start_delimimter, 0);
    if (pos_start_delimimter == -1)
    {
        printf("delimimter 1 not found");
        return 0;
    }
    unsigned pos_ende_delimimter = buffer.find(ende_delimimter, pos_start_delimimter);
    if (pos_ende_delimimter == -1)
    {
        printf("delimimter 2 not found");
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

std::vector<Eigen::Vector2d> json2Struct(nlohmann::json_abi_v3_11_2::json j_msg_L, const double &cutoff) // get parsed json file to struct
{
    struct msgLIDAR msgL;

    j_msg_L["header"]["seq"].get_to(msgL.header.seq);
    j_msg_L["header"]["stamp"]["secs"].get_to(msgL.header.stamp.secs);
    j_msg_L["header"]["stamp"]["nsecs"].get_to(msgL.header.stamp.nsecs);
    j_msg_L["header"]["frame_id"].get_to(msgL.header.frame_id);
    j_msg_L["angle_increment"].get_to(msgL.angle_increment);
    j_msg_L["angle_max"].get_to(msgL.angle_max);
    j_msg_L["angle_min"].get_to(msgL.angle_min);
    j_msg_L["time_increment"].get_to(msgL.time_increment);
    j_msg_L["scan_time"].get_to(msgL.scan_time);
    j_msg_L["range_max"].get_to(msgL.range_max);
    j_msg_L["range_min"].get_to(msgL.range_min);
    j_msg_L["intensities"].get_to(msgL.intensities);
    j_msg_L["ranges"].get_to(msgL.ranges);


    std::vector<Eigen::Vector2d> lidar_scans;

    for (int i = 0; i < 360; i++) // polar coordinates (range, angle) into artesian coordinates (x, y)
    {
        msgL.XYcoordinates.x.push_back(msgL.ranges.at(i) * cos(msgL.angle_min + msgL.angle_increment * i));
        msgL.XYcoordinates.y.push_back(msgL.ranges.at(i) * sin(msgL.angle_min + msgL.angle_increment * i));

        

        const Eigen::Vector2d scan_pos(msgL.XYcoordinates.x.at(i), msgL.XYcoordinates.y.at(i));
       
        if (0.1 <= msgL.ranges.at(i) && msgL.ranges.at(i) <= 0.8)
        {
            lidar_scans.push_back(scan_pos);
            // std::cout <<"Range:" << msgL.ranges.at(i) << " ";
            std::cout << "X: " << scan_pos.x() << " // Y: " << scan_pos.y() << std::endl;

        }
        
    }
    std::cout << "\n\n-+-+- " << lidar_scans.size() << " -+-+-\n";

    return lidar_scans;
}

void outputLIDARStruct(struct msgLIDAR msgL) // test ouuput of Odom Struct
{
    std::cout << std::endl
              << std::endl
              << "Scan Id: \t\t" << msgL.header.frame_id << std::endl
              << "Seq. Nr: \t\t" << msgL.header.seq << std::endl
              << "\t Sek.: \t\t" << msgL.header.stamp.secs << std::endl
              << "\t nano Sek.: \t" << msgL.header.stamp.nsecs << std::endl
              << "Angle min: \t" << msgL.angle_min << std::endl
              << "Angle max: \t" << msgL.angle_max << std::endl
              << "Angle increment:" << msgL.angle_increment << std::endl
              << "Range min: \t" << msgL.range_min << std::endl
              << "Range max: \t" << msgL.range_max << std::endl;
    std::cout << "36 Ranges (every 10 degrees):" << std::endl
              << "\t";
    for (int i = 0; i < 360; i++)
    {
        if (i % 10 == 0)
            std::cout << msgL.ranges.at(i) << ", ";
    }
    std::cout << std::endl;

    std::cout << "36 X and Y Coordinates (every 10 degrees):" << std::endl
              << "\t";
    for (int i = 0; i < 360; i++)
    {
        if (i % 10 == 0)
        {
            std::cout << std::fixed << std::setprecision(3) << "(" << msgL.XYcoordinates.x.at(i) << " - " << msgL.XYcoordinates.y.at(i) << ")\n";
        }
    }
    std::cout << std::endl
              << std::endl;
}

int main(int argc, char *argv[])
{
    double LANDMARK_RADIUS = 0.031;
    double LIDAR_SCAN_DISTANCE_CUTOFF = 0.75;
    // load landmark RADIUS in [m] from arguments
    if (argc >= 3)
    {
        LANDMARK_RADIUS = std::strtod(argv[2], NULL);
        std::cout << "Setting landmark radius " << LANDMARK_RADIUS << std::endl;
    }
    // load lidar scan cutoff in [m] from arguments
    if (argc >= 4)
    {
        LIDAR_SCAN_DISTANCE_CUTOFF = std::strtod(argv[3], NULL);
        std::cout << "Setting scan range cutoff " << LIDAR_SCAN_DISTANCE_CUTOFF << std::endl;
    }

    // the utility class used to detect the relative landmark position
    lidar_loc ::CircleDetection circle_detector = lidar_loc::CircleDetection(LANDMARK_RADIUS);

    struct sockaddr_in echoServAddr; // server address
    unsigned short echoServPort;     // port
    char *servIP;                    // Server IP address
    char echoBuffer[RCVBUFSIZE];     // Buffer
    int bytesRcvd, totalBytesRcvd;   

    if ((argc < 2) || (argc > 4)) //Check number of arguments
    {
        fprintf(stderr, "Usage: %s <Server IP> [<Landmark Radius [m]>] [<Max. Scan Range [m]>]\n",
                argv[0]);
        exit(1);
    }

    servIP = argv[1];     // Get server IP address from argv[1]
    

    // the port for lidar data remains the same
    echoServPort = 9997;

    //---------------------------------------------------------------------------
    //Create socket using TCP for sending to server
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        printf("socket() for reciving lidar data  failed");

    //Structure to connect to server
    memset(&echoServAddr, 0, sizeof(echoServAddr));   
    echoServAddr.sin_family = AF_INET;                
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   //IP address from Server
    echoServAddr.sin_port = htons(echoServPort);        //Port adress

    //Connect to echo server
    if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
        printf("connect() for reciving lidar data failed");

    //---------------------------------------------------------------------------

    attachSemaphore(); //attached to all semaphores
    signal(SIGINT, signalHandler); // catch SIGINT
    //initial Sempahore - post to show that the listener is ready
    sem_post(init_lidar);

    //Close socket on the live-systme | let the socket open in the simulation
    if (SIMULATIONS_ON == 0)
    {
        close(sock);
    }
    //data for the shared memory
    struct SharedMemoryLIDAR *test = new SharedMemoryLIDAR();
    test->testData = 5;

    //---------------------------------------------------------------------------

    
    int count = 0;  //counter for not correct received messages
    totalBytesRcvd = 0;

    printf("Start receiving: "); 
    while (1)
    {
        //Wait for commander
        sem_wait(sem_empty_lidar);
        //Wait for mutex - critical section
        sem_wait(mutex_lidar);

        if (SIMULATIONS_ON == 0) //ckeck if simulation or live-system
        {

            //Create socket using TCP for sending to server
            if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
                printf("socket() for reciving lidar data failed");

            //Structure to connect to server
            memset(&echoServAddr, 0, sizeof(echoServAddr));   
            echoServAddr.sin_family = AF_INET;                
            echoServAddr.sin_addr.s_addr = inet_addr(servIP); 
            echoServAddr.sin_port = htons(echoServPort);      

            //Connect to echo server
            if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
                printf("connect() for reciving lidar data failed");
        }

        // nanosleep((const struct timespec[]){{0, 500000000L}}, NULL);

        if ((bytesRcvd = recv(sock, echoBuffer, RCVBUFSIZE - 1, 0)) <= 0)
            printf("recv() failed or connection closed prematurely");
        totalBytesRcvd += bytesRcvd;  
        echoBuffer[bytesRcvd] = '\0'; 
        // printf("%s", echoBuffer);      
        // std::cout << echoBuffer << std::endl;
        std::cout << "______________________________-" << std::endl;

        if (checkMessage(echoBuffer, "--START---", "___END___") == 1)
        {
            // std::cout << getMessage(echoBuffer, "--START---", "___END___") << std::endl;

            std::string tempStrng = getMessage(echoBuffer, "--START---", "___END___"); // get message to string for parsing

            tempStrng.erase(0, 10);                // delete --START--- for parsing
            tempStrng.erase(tempStrng.size() - 9); // delete ___END___  for parsing

            nlohmann::json_abi_v3_11_2::json j_msg_L = nlohmann::json_abi_v3_11_2::json::parse(tempStrng); // parse tempStrng to j_msg_L

            // load all lidar scans in the form relative (x, y) that are closer than the cutoff
            const std::vector<Eigen::Vector2d> scans = json2Struct(j_msg_L, LIDAR_SCAN_DISTANCE_CUTOFF); // get parsed json file to struct
            // compute the center from scan data
            const lidar_loc::MaybeVector2d result = circle_detector.compute_center(scans);

            if(result.has_value())
            {
                std::cout << "result: \t" << result->x() << " , " << result->y() << std::endl << std::endl;  
            }

            // outputLIDARStruct(msgL);   // Test ouput

            // TODO: shared memobry

            test->testData++;
            test->relative_landmark_position = result;
        }
        else
        {
            count++;
        }

        if (SIMULATIONS_ON == 0) //ckeck if simulation or live-system
        {
            close(sock);
        }
        //attach to shared memroy lidar and write data to shared memory lidar
        writeSharedMemory(block, test); // Casper: in dieser Funktion werden die Daten des structs auf den SharedMemory geschrieben, bitte diese Funktion anpassen
        //detach from shared memory
        detach_shared_memory_LIDAR(block);
        //Post mutex - writing finished
        sem_post(mutex_lidar);
        //Post lidar/commander - all data written
        sem_post(sem_full_lidar);

        printf("waiting for commander\n");
        printf("\n count = %d\n", count);
    }
    printf("\n"); 
    //Close alle sempahores and detach from shared memory block
    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(init_lidar);
    sem_close(mutex_lidar);

    detach_shared_memory_LIDAR(block);
    //Close socket
    close(sock);
    exit(0);
}
