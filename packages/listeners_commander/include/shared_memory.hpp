#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <stdbool.h>

struct SharedMemoryLIDAR
{
    int testData;
};

struct SharedMemoryODO
{
    int testData;
};

// attach a shared memroy block
// Associated with filename
// Create it if it does not exist
char *attach_memory_block(const char *filename, int size);
bool detach_memory_block(const char *block);
bool destroy_memory_block(const char *filename);
// LIDAR
struct SharedMemoryLIDAR *attach_memory_block_LIDAR(const char *filename);
bool detach_memory_block_LIDAR(struct SharedMemoryLIDAR *block);
// Odometrie
struct SharedMemoryODO *attach_memory_block_Odometrie(const char *filename);
bool detach_memory_block_Odometrie(struct SharedMemoryODO *block);

// all of the programs will share these values
#define BLOCK_SIZE 4096
#define FILENAME "src/TCPEchoClient_Lidar.cpp"
#define FILENAME2 "src/TCPEchoClient_Odometrie.cpp"

//define FILENAME "/apr/shmem_lidar"
//#define FILENAME2 "/apr/shmem_odom"

// Filenames for four semaphores
#define SEM_PRODUCER_FNAME "/myproducer"
#define SEM_PRODUCER2_FNAME "/myproducer2"
#define SEM_CONSUMER_FNAME "/myconsumer"
#define SEM_CONSUMER2_FNAME "/myconsumer2"

#endif