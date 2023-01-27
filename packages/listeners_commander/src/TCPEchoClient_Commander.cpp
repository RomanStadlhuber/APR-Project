#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
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
#include <CircleDetection.hpp>
#include <shared_memory.hpp>
#include <controller.hpp>
#include <cmath>

// #include "shared_memory.hpp"

int sock_com; // socket Commander
// Used Semaphores
sem_t *sem_full_lidar;
sem_t *sem_empty_lidar;
sem_t *sem_full_odo;
sem_t *sem_empty_odo;
sem_t *init_lidar;
sem_t *init_odo;
sem_t *mutex_lidar;
sem_t *mutex_odo;
// Used SharedMemory-Blocks
struct SharedMemoryLIDAR *block_LIDAR;
struct SharedMemoryODO *block_ODO;

// the format used for printing Eigen Vectors and Matrices
Eigen::IOFormat fmt_clean(4, 0, ", ", " ", "", "", "[", "]");

void signalHandler(int sig)
{
    // Close alle sempahores and detach from shared memory blocks
    printf("Close all\n");
    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(init_lidar);
    sem_close(init_odo);
    sem_close(mutex_lidar);
    sem_close(mutex_odo);

    detach_shared_memory_LIDAR(block_LIDAR);
    detach_shared_memory_Odometrie(block_ODO);
    // Close socket
    close(sock_com);
    exit(0);
}

void createSemahpore()
{
    // Unlink all sempahores bevor starting!
    sem_unlink(EMPTY_LIDAR);
    sem_unlink(FULL_ODO);
    sem_unlink(EMPTY_ODO);
    sem_unlink(FULL_LIDAR);
    sem_unlink(MUTEX_LIDAR);
    sem_unlink(SEM_INIT_LIDAR);
    sem_unlink(SEM_INIT_ODO);
    sem_unlink(MUTEX_ODO);

    // Open used semaphores
    init_lidar = sem_open(SEM_INIT_LIDAR, O_CREAT, 0777, 0);
    if (init_lidar == SEM_FAILED)
    {

        printf("open semaphore \"init_lidar\" failed\n");
        exit(EXIT_FAILURE);
    }

    init_odo = sem_open(SEM_INIT_ODO, O_CREAT, 0777, 0);
    if (init_odo == SEM_FAILED)
    {
        printf("open semaphore \"init_odo\" failed\n");
        exit(EXIT_FAILURE);
    }

    sem_full_lidar = sem_open(FULL_LIDAR, O_CREAT, 0777, 0);
    if (sem_full_lidar == SEM_FAILED)
    {
        printf("open semaphore \"sem_full_lidar\" failed\n");
        exit(EXIT_FAILURE);
    }

    sem_full_odo = sem_open(FULL_ODO, O_CREAT, 0777, 0);
    if (sem_full_odo == SEM_FAILED)
    {
        printf("open semaphore \"sem_full_odo\" failed\n");
        exit(EXIT_FAILURE);
    }

    sem_empty_lidar = sem_open(EMPTY_LIDAR, O_CREAT, 0777, 1);
    if (sem_empty_lidar == SEM_FAILED)
    {
        printf("open semaphore \"sem_empty_lidar\" failed\n");
        exit(EXIT_FAILURE);
    }

    sem_empty_odo = sem_open(EMPTY_ODO, O_CREAT, 0777, 1);
    if (sem_empty_odo == SEM_FAILED)
    {
        printf("open semaphore \"sem_empty_odo\" failed\n");
        exit(EXIT_FAILURE);
    }

    mutex_lidar = sem_open(MUTEX_LIDAR, O_CREAT, 0777, 1);
    if (mutex_lidar == SEM_FAILED)
    {
        printf("open semaphore \"mutex_lidar\" failed\n");
        exit(EXIT_FAILURE);
    }

    mutex_odo = sem_open(MUTEX_ODO, O_CREAT, 0777, 1);
    if (mutex_odo == SEM_FAILED)
    {
        printf("open semaphore \"mutex_odo\" failed\n");
        exit(EXIT_FAILURE);
    }

    // printf("open all semaphores was succesfull\n");
}

struct SharedMemoryLIDAR *readSharedMemoryLidar()
{

    block_LIDAR = attach_shared_memory_LIDAR(FILENAME_LIDAR);
    if (block_LIDAR == NULL)
    {
        printf("Attaching shared memory block LIDAR not successfull\n");
    }
    return block_LIDAR;
}

struct SharedMemoryODO *readSharedMemoryOdometrie()
{
    block_ODO = attach_shared_memory_Odometrie(FILENAME_ODO);
    if (block_ODO == NULL)
    {
        printf("Attaching shared memory block Odo not successfull\n");
    }
    return block_ODO;
}

void connectSocket(struct sockaddr_in server_addr, unsigned short server_port, char *server_IP)
{
    // Create socket using TCP for sending to server
    if ((sock_com = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        printf("creat socket() for sending failed");

    // Structure to connect to server
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(server_IP); // IP address from Server
    server_addr.sin_port = htons(server_port);      // Port adress

    // Connect to echo server
    if (connect(sock_com, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        printf("connect() to server for sending failed");
}

int main(int argc, char *argv[])
{

    struct sockaddr_in server_addr; // server address
    unsigned short server_port;     // port
    char *server_IP;                    // Server IP address

    if ((argc < 2) || (argc > 4)) // Check number of arguments
    {
        fprintf(stderr, "Usage: %s <Server IP> <landmark-x> <landmark-y>\n",
                argv[0]);
        exit(1);
    }

    // TODO: read frmo argumntes
    double fusion_weight_odom = 0.5;
    double fusion_weight_lidar = 0.5;
    // NOTE should be something about ~ x: 0.53, y: 0.22 or so
    Eigen::Vector2d fusion_landmark_position(std::strtod(argv[2], NULL), std::strtod(argv[3], NULL));
    std::cout << "setting landmark reference position: " << fusion_landmark_position.format(fmt_clean) << std::endl;

    lidar_loc::Fusion fusion(
        fusion_weight_odom,
        fusion_weight_lidar,
        fusion_landmark_position, // relative to the starting position (we need to set it later)
        {}                        // starting position is None (can only set it once data is first received)
    );
    bool fusion_initial_pose_is_set = false;

    server_IP = argv[1]; // Get server IP address from argv[1]

    if (argc == 3)
        server_port = atoi(argv[2]); // Get port address from argv[2]
    else
        server_port = 9999; // default port ardess

    // Connect to server at beginning, to check if its possible
    connectSocket(server_addr, server_port, server_IP);
    // Close socket
    close(sock_com);

    //------------------------------------------------------------------------

    createSemahpore(); // Creat all semaphores

    signal(SIGINT, signalHandler); // catch SIGINT
    // initial Sempahores to check if all programms are ready
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

    int shape = 1;

    if (shape == 1)
        goals = 2; // Line
    if (shape == 2)
        goals = 4; // Triangle
    if (shape == 3)
        goals = 5; // Square
    if (shape == 4)
        goals = 9; // Circle

    int cntr = 0;

    while (curr_goal < goals)
    {
        // Wait for Lidar
        sem_wait(sem_full_lidar);
        // Wait for mutex - critical section
        sem_wait(mutex_lidar);
        // attach to shared memroy lidar and read data from shared memory lidar
        block_LIDAR = readSharedMemoryLidar();

        lidar_measurement = block_LIDAR->relative_landmark_position;

        // detach from shared memory
        detach_shared_memory_LIDAR(block_LIDAR);

        // Post mutex - reading finished
        sem_post(mutex_lidar);
        // Post lidar - all data read
        sem_post(sem_empty_lidar);

        // Wait for Odometrie
        sem_wait(sem_full_odo);
        // Wait for mutex - critical section
        sem_wait(mutex_odo);

        // attach to shared memroy odometrie and read data from shared memory odometrie
        block_ODO = readSharedMemoryOdometrie();

        if (!fusion_initial_pose_is_set)
        {
            const auto initial_pose = block_ODO->pose;
            // convert to (x, y, yaw)
            const auto initial_pose_as_twist = fusion.pose2twist(
                initial_pose.position, initial_pose.orientation);
            // set reference frame for odometry measurements
            fusion.set_reference(initial_pose_as_twist);
            std::cout << "setting reference frame to x: "
                      << initial_pose_as_twist.x() << " y: "
                      << initial_pose_as_twist.y() << " theta: "
                      << initial_pose_as_twist.z() << std::endl;
            // set the flag so we know we're done
            fusion_initial_pose_is_set = true;
        }
        else
        {
            auto const curr_odom_pose = block_ODO->pose;

            odometry_measurement = fusion.pose2twist(curr_odom_pose.position, curr_odom_pose.orientation);
        }

        // printf("Reading Odo: \"%d\"\n", block_ODO->testData);

        // detach from shared Memory
        detach_shared_memory_Odometrie(block_ODO);

        if (fusion_initial_pose_is_set)
        {
            // print relative odometry pose
            // std::cout << "odometry:\t" << odometry_measurement.format(fmt_clean) << std::endl;
            odometry_measurement.format(fmt_clean);
            // print lidar landmark pos if available
            if (lidar_measurement.has_value())
            {
                std::cout << "////////////////////////////////////// \n ------ Got a Landmark position! -----\n"
                          << "\tLandmark position: "
                          << lidar_measurement->format(fmt_clean) << std::endl;
            }

            const Eigen::Vector3d fused_pose = fusion.fuse_to_pose(odometry_measurement, {});

            // double sec = block_ODO->header.stamp.secs;
            // double nsec =  block_ODO->header.stamp.nsecs * 0,000000001;
            // std::cout << std::endl << "Sek: \t" << sec+nsec << std::endl << std::endl

            if (cntr != 0) // to skip first iteration because first position values are fals
            {
                std::cout << std::endl
                          << std::endl
                          << "Fused Pose (x - y - z) :"
                          << "(" << fused_pose.x() << " , " << fused_pose.y() << " , " << fused_pose.z() << ""
                          << ")" << std::endl
                          << std::endl;

                if (shape == 1) // Line
                {
                    if (PID_cntrl.error(fused_pose.x(), fused_pose.y(), fused_pose.z(), pose.line_pos_x[curr_goal], pose.line_pos_y[curr_goal], pose.line_th[curr_goal]) < 0.08)
                    {
                        std::cout << "reached goal: " << curr_goal << "(" << pose.line_pos_x[curr_goal] << pose.line_pos_y[curr_goal] << ")" << std::endl;
                        curr_goal++;
                    }
                }

                if (shape == 2) // Triangle
                {
                    if (PID_cntrl.error(fused_pose.x(), fused_pose.y(), fused_pose.z(), pose.triangle_pos_x[curr_goal], pose.triangle_pos_y[curr_goal], pose.triangle_th[curr_goal]) < 0.08)
                    {
                        std::cout << "reached goal: " << curr_goal << "("
                                  << pose.triangle_pos_x[curr_goal] << pose.triangle_pos_y[curr_goal] << ")" << std::endl;
                        curr_goal++;
                    }
                }

                if (shape == 3) // Square
                {
                    if (PID_cntrl.error(fused_pose.x(), fused_pose.y(), fused_pose.z(), pose.square_pos_x[curr_goal], pose.square_pos_y[curr_goal], pose.square_th[curr_goal]) < 0.08)
                    {
                        std::cout << "reached goal: " << curr_goal << "("
                                  << pose.square_pos_x[curr_goal] << pose.square_pos_y[curr_goal] << ")" << std::endl;
                        std::cout << "///////////////////////////////////////////\n ///////////////////////////////////////////" << std::endl;
                        curr_goal++;
                    }
                }

                if (shape == 4) // Circle
                {
                    if (PID_cntrl.error(fused_pose.x(), fused_pose.y(), fused_pose.z(), pose.circle_pos_x[curr_goal], pose.circle_pos_y[curr_goal], pose.circle_th[curr_goal]) < 0.08)
                    {
                        std::cout << "reached goal: " << curr_goal << "("
                                  << pose.circle_pos_x[curr_goal] << pose.circle_pos_y[curr_goal] << ")" << std::endl;
                        std::cout << "///////////////////////////////////////////\n ///////////////////////////////////////////" << std::endl;
                        curr_goal++;
                    }
                }

                vel = PID_cntrl.get_linear_velocity();
                omega = PID_cntrl.get_angular_velocity();
            }
            cntr++;
        }
        // Post mutex - reading finished
        sem_post(mutex_odo);
        // Post Odometrie - all data read
        sem_post(sem_empty_odo);

        //------------------------------------------------------------------------
        // great Message-String, Grü Controller Output auf lin und angular

        float lin = vel;
        float angular = omega;

        if (curr_goal == goals)
        {
            angular = 0;
            lin = 0;
        }

        // Create Message for sending
        std::ostringstream oss;

        //"---START---{\"linear\": 0.1, \"angular\": 0.10}___END___\0";
        oss << "---START---{\"linear\": " << lin << ", \"angular\": " << angular << "}___END___\0";
        std::string send_str = oss.str();
        const char *msg = send_str.c_str();
        unsigned int msgStringLen = strlen(msg);

        //------------------------------------------------------------------------

        // Connect to socket bevor sending
        connectSocket(server_addr, server_port, server_IP);

        // Send string
        if (send(sock_com, msg, msgStringLen, 0) != msgStringLen)
            printf("send() not correct");

        printf("%s\n", msg);
        // Close socket again after sending
        close(sock_com);
    }

    // Close alle sempahores and detach from shared memory blocks
    sem_close(sem_empty_odo);
    sem_close(sem_full_odo);
    sem_close(sem_empty_lidar);
    sem_close(sem_full_lidar);
    sem_close(init_lidar);
    sem_close(init_odo);
    sem_close(mutex_lidar);
    sem_close(mutex_odo);

    detach_shared_memory_LIDAR(block_LIDAR);
    detach_shared_memory_Odometrie(block_ODO);
    // Close socket
    close(sock_com);
    exit(0);
}