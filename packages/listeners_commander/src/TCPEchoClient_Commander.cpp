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
#include <Fusion.hpp>
#include <shared_memory.hpp>
#include <controller.hpp>

// #include "shared_memory.hpp"

#define BLOCK_SIZE 4096


/*
    TODO: give shape per argument ääh
    1 = line
    2 = triangle
    3 = square
    4 = circle
*/



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

// the format used for printing Eigen Vectors and Matrices
Eigen::IOFormat fmt_clean(4, 0, ", ", "\n", "[", "]");

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
}

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
    // printf("Herer...\n");
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
    // printf("Herer...\n");
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

    if ((argc < 2) || (argc > 3)) /* Test for correct number of arguments */
    {
        fprintf(stderr, "Usage: %s <Server IP> [<Echo Port>]\n",
                argv[0]);
        exit(1);
    }

    // TODO: read frmo argumntes
    double fusion_weight_odom = 0.9;
    double fusion_weight_lidar = 0.1;
    Eigen::Vector2d fusion_landmark_position(0.5, 0.5);

    lidar_loc::Fusion fusion(
        fusion_weight_odom,
        fusion_weight_lidar,
        fusion_landmark_position, // relative to the starting position (we need to set it later)
        {}                        // starting position is None (can only set it once data is first received)
    );
    bool fusion_initial_pose_is_set = false;

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

    Eigen::Vector3d odometry_measurement;
    lidar_loc::MaybeVector2d lidar_measurement = {};
    pid_controler PID_cntrl;

    double current_pos_x = 0;
    double current_pos_y = 0;
    double current_th = 0;

    double vel = 0;
    double omega = 0;

    int goals = 0;
    int curr_goal = 0;

    int shape = 1; // because of line

    if(shape == 1) goals = 2;
    if(shape == 2) goals = 4;
    if(shape == 3) goals = 5;
    if(shape == 4) goals = 9;

    int cntr = 0;


    while (curr_goal < goals)
    {

        // printf("Waiting Lidar...\n");

        sem_wait(sem_full_lidar);
        sem_wait(mutex_lidar);

        block = readSharedMemoryLidar();

        // printf("Reading Lidar: \"%d\"\n", block->testData);

        lidar_measurement = block->relative_landmark_position;

        detach_memory_block_LIDAR(block);

        sem_post(mutex_lidar);
        sem_post(sem_empty_lidar);

        // printf("Waiting Odo...\n");

        sem_wait(sem_full_odo);
        sem_wait(mutex_odo);

        block2 = readSharedMemoryOdometrie();

        if (!fusion_initial_pose_is_set)
        {
            const auto initial_pose = block2->pose;
            // convert to (x, y, yaw)
            const auto initial_pose_as_twist = fusion.pose2twist(
                initial_pose.position, initial_pose.orientation);
            // set reference frame for odometry measurements
            fusion.set_reference(initial_pose_as_twist);
            // set the flag so we know we're done
            fusion_initial_pose_is_set = true;
        }
        else
        {
            auto const curr_odom_pose = block2->pose;
            odometry_measurement = fusion.pose2twist(curr_odom_pose.position, curr_odom_pose.orientation);
        }

        // printf("Reading Odo: \"%d\"\n", block2->testData);

        detach_memory_block_Odometrie(block2);

        if (fusion_initial_pose_is_set)
        {
            // print relative odometry pose
            // std::cout << "odometry:\t" << odometry_measurement.format(fmt_clean) << std::endl;
            odometry_measurement.format(fmt_clean);
            // print lidar landmark pos if available
            if (lidar_measurement.has_value())
                // std::cout << "lidar" << lidar_measurement->format(fmt_clean) << std::endl;
                lidar_measurement->format(fmt_clean);

            
            Eigen::Vector3d fused_pose = fusion.fuse_to_pose(odometry_measurement, {});

            //double th = fused_pose.z();
            //if(th > M_PI) th =  th - 2*M_PI;

            // calculate linear and angular velocity 

            if (cntr != 0) // to skip first iteration because first position values are fals
            {
                std::cout << std::endl << std::endl << "Fused Pose (x - y - z) :" << "(" << fused_pose.x() << " , " << fused_pose.y() << " , " << fused_pose.z()<< " ,  " << ")" << std::endl << std::endl;

                if (PID_cntrl.error(fused_pose.x(), fused_pose.y(), fused_pose.z(), pose.line_pos_x[curr_goal], pose.line_pos_y[curr_goal], pose.line_th[curr_goal]) < 0.08)
                {
                    std::cout << "reached goal: " << curr_goal << "(" << pose.line_pos_x[curr_goal] << pose.line_pos_y[curr_goal] << ")" << std::endl;
                    curr_goal++;
                }

                vel = PID_cntrl.get_linear_velocity();
                omega = PID_cntrl.get_angular_velocity();
            
            }
            cntr++;

        }

        sem_post(mutex_odo);
        sem_post(sem_empty_odo);

        //------------------------------------------------------------------------
        // great Message-String, Grü Controller Output auf lin und angular

        float lin = 0.05;
        float angular = -0.1;

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
    }



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
}