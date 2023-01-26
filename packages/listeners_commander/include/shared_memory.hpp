#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <stdbool.h>
#include <signal.h>
#include <iomanip>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <CircleDetection.hpp>
#define SIMULATIONS_ON 1

// struchts to save the json massage
struct stampStrc
{
public:
    int secs = 0;
    int nsecs = 0;
};

struct headerStrct
{
public:
    std::string frame_id;
    int seq = 0;
    stampStrc stamp;
};

struct coordinatesCalculadet
{
public:
    std::vector<double> x;
    std::vector<double> y;
};

struct msgLIDAR
{
public:
    headerStrct header;
    double angle_increment = 0;
    double angle_max = 0;
    double angle_min = 0;
    double time_increment = 0;
    double scan_time = 0;
    double range_max;
    double range_min;
    coordinatesCalculadet XYcoordinates;
    std::array<Eigen::Vector2d, 360> scan_positions; // nur das ist nötig für den shared memory
    std::vector<double> intensities;
    std::vector<double> ranges;
};

struct posiCoordinates
{
public:
    double x = 0;
    double y = 0;
    double z = 0;
};

struct oriCoordinates
{
public:
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

struct poseStrct
{
public:
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct msgOddomtr
{
public:
    headerStrct header;
    std::string child_frame_id;
    poseStrct pose;
};

// Casper: deine structs hier ergänzen, dise structs werden dann auf den SharedMemory geschrieben
struct SharedMemoryLIDAR
{
    lidar_loc::MaybeVector2d relative_landmark_position;
    int testData;
};

// Casper: deine structs hier ergänzen, dise structs werden dann auf den SharedMemory geschrieben
struct SharedMemoryODO
{
public:
    headerStrct header;
    std::string child_frame_id;
    poseStrct pose;
    int testData;
};

//Template functions for shared memory 
//char *attach_memory_block(const char *filename, int size); //(not used)
//bool detach_memory_block(const char *block); //(not used)
bool destroy_memory_block(const char *filename); //Is used for SharedMemory-block from LIDAR and Odometrie

//SharedMemeory-block LIDAR
struct SharedMemoryLIDAR *attach_memory_block_LIDAR(const char *filename);
bool detach_memory_block_LIDAR(struct SharedMemoryLIDAR *block);
//SharedMemeory-block Odometrie
struct SharedMemoryODO *attach_memory_block_Odometrie(const char *filename);
bool detach_memory_block_Odometrie(struct SharedMemoryODO *block);


//#define BLOCK_SIZE 4096
//Filenames of our shared memeory
#define FILENAME_LIDAR "src/TCPEchoClient_Lidar.cpp"
#define FILENAME_ODO "src/TCPEchoClient_Odometrie.cpp"

//Filenames of our semaphores
#define FULL_LIDAR "/fullLidar"
#define FULL_ODO "/fullOdo"
#define EMPTY_LIDAR "/emptyLidar"
#define EMPTY_ODO "/emptyOdo"
#define SEM_INIT_LIDAR "/initLidar"
#define SEM_INIT_ODO "/initOdo"
#define MUTEX_LIDAR "/mutexLidar"
#define MUTEX_ODO "/mutexOdo"

#endif