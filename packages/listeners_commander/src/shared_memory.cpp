#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <shared_memory.hpp>
// #include "shared_memory.hpp"

#define ERROR (-1)

//Template function to get shared memory 
static int get_shared_memory(const char *shared_memory_filename, int shared_memory_size)
{
    key_t shared_memory_key; //key for named shared memory

    shared_memory_key = ftok(shared_memory_filename, 0); //returns key from shared memory filename
    if (shared_memory_key == ERROR)                      //Check key
    {
        return ERROR;
    }

    return shmget(shared_memory_key, shared_memory_size, 0644 | IPC_CREAT); //Get shared memory id
}
//Function to get shared memory LIDAR
static int get_shared_memory_LIDAR(const char *shared_memory_filename)
{
    key_t shared_memory_key; //key for named shared memory

    shared_memory_key = ftok(shared_memory_filename, 0);    //returns key from shared memory filename
    if (shared_memory_key == ERROR)                         //Check key
    {
        return ERROR;
    }

    return shmget(shared_memory_key, sizeof(struct SharedMemoryLIDAR), 0644 | IPC_CREAT); //Get shared memory id
}
//Function to get shared memory Odo
static int get_shared_memory_Odometrie(const char *shared_memory_filename)
{
    key_t shared_memory_key;    //key for named shared memory

    shared_memory_key = ftok(shared_memory_filename, 0);    //returns key from shared memory filename
    if (shared_memory_key == ERROR)                         //Check key
    {
        return ERROR;
    }

    return shmget(shared_memory_key, sizeof(struct SharedMemoryODO), 0644 | IPC_CREAT); //Get shared memory id
}
//Function to attach shared memory LIDAR
struct SharedMemoryLIDAR *attach_shared_memory_LIDAR(const char *shared_memory_filename) 
{
    int shared_memory_block_id = get_shared_memory_LIDAR(shared_memory_filename); //Get shared memory id
    struct SharedMemoryLIDAR *result;

    if (shared_memory_block_id == ERROR) //Check shared memory id
    {
        return NULL;
    }

    result = (struct SharedMemoryLIDAR *)shmat(shared_memory_block_id, NULL, 0); //Get shared memory block
    if (result == (struct SharedMemoryLIDAR *)ERROR)  //Check shared memory block
    {
        return NULL;
    }

    return result; //return shared memory block
}
//Function to attach shared memory Odo
struct SharedMemoryODO *attach_shared_memory_Odometrie(const char *shared_memory_filename)
{
    int shared_memory_block_id = get_shared_memory_Odometrie(shared_memory_filename); //Get shared memory id
    struct SharedMemoryODO *result;

    if (shared_memory_block_id == ERROR) //Check shared memory id
    {
        return NULL;
    }

    result = (struct SharedMemoryODO *)shmat(shared_memory_block_id, NULL, 0); //Get shared memory block
    if (result == (struct SharedMemoryODO *)ERROR) //Check shared memory block
    {
        return NULL;
    }

    return result; //return shared memory block
}

//Function to detach shared memory LIDAR
bool detach_shared_memory_LIDAR(struct SharedMemoryLIDAR *block)
{
    return (shmdt(block) != ERROR); //detach shared memory
}
//Function to detach shared memory Odo
bool detach_shared_memory_Odometrie(struct SharedMemoryODO *block)
{
    return (shmdt(block) != ERROR); //detach shared memory
}

//Function to remove a shared memory 
bool remove_shared_memory(const char *shared_memory_filename) 
{
    int shared_memory_id = get_shared_memory(shared_memory_filename, 0); //Get shared memory id

    if (shared_memory_id == ERROR) //Check shared memory id
    {
        return NULL;
    }
    return (shmctl(shared_memory_id, IPC_RMID, NULL) != ERROR); //Remove shared memory segment
}
