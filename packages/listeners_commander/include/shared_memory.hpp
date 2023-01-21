#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <stdbool.h>

#define SIMULATIONS_ON 1

//Casper: deine structs hier ergänzen, dise structs werden dann auf den SharedMemory geschrieben
struct SharedMemoryLIDAR
{
    int testData;
};
//Casper: deine structs hier ergänzen, dise structs werden dann auf den SharedMemory geschrieben
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

#define FILENAME_LIDAR "src/TCPEchoClient_Lidar.cpp"
#define FILENAME_ODO "src/TCPEchoClient_Odometrie.cpp"

//#define FILENAME_LIDAR "TCPEchoClient_Lidar.cpp"
//#define FILENAME_ODO "TCPEchoClient_Odometrie.cpp"


// Filenames for four semaphores
#define FULL_LIDAR "/fullLidar"
#define FULL_ODO "/fullOdo"
#define EMPTY_LIDAR "/emptyLidar"
#define EMPTY_ODO "/emptyOdo"
#define SEM_INIT_LIDAR "/initLidar"
#define SEM_INIT_ODO "/initOdo"
#define MUTEX_LIDAR "/mutexLidar"
#define MUTEX_ODO "/mutexOdo"


#endif