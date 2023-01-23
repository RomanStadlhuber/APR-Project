#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <shared_memory.hpp>
//#include "shared_memory.hpp"

#define IPC_RESULT_ERROR (-1)

static int get_shared_block(const char *filename, int size)
{
    key_t key;

    key = ftok(filename, 0);
    if (key == IPC_RESULT_ERROR)
    {
        return IPC_RESULT_ERROR;
    }

    return shmget(key, size, 0644 | IPC_CREAT);
}

static int get_shared_block_LIDAR(const char *filename)
{
    key_t key;

    key = ftok(filename, 0);
    if (key == IPC_RESULT_ERROR)
    {
        return IPC_RESULT_ERROR;
    }

    return shmget(key, sizeof(struct SharedMemoryLIDAR), 0644 | IPC_CREAT);
}

static int get_shared_block_Odometrie(const char *filename)
{
    key_t key;

    key = ftok(filename, 0);
    if (key == IPC_RESULT_ERROR)
    {
        return IPC_RESULT_ERROR;
    }

    return shmget(key, sizeof(struct SharedMemoryODO), 0644 | IPC_CREAT);
}

char *attach_memory_block(const char *filename, int size)
{
    int shared_block_id = get_shared_block(filename, size);
    char *result;

    if (shared_block_id == IPC_RESULT_ERROR)
    {
        return NULL;
    }

    result = (char *)shmat(shared_block_id, NULL, 0);
    if (result == (char *)IPC_RESULT_ERROR)
    {
        return NULL;
    }

    return result;
}

struct SharedMemoryLIDAR *attach_memory_block_LIDAR(const char *filename)
{
    int shared_block_id = get_shared_block_LIDAR(filename);
    struct SharedMemoryLIDAR *result;

    if (shared_block_id == IPC_RESULT_ERROR)
    {
        return NULL;
    }

    result = (struct SharedMemoryLIDAR *)shmat(shared_block_id, NULL, 0);
    if (result == (struct SharedMemoryLIDAR *)IPC_RESULT_ERROR)
    {
        return NULL;
    }

    return result;
}

struct SharedMemoryODO *attach_memory_block_Odometrie(const char *filename)
{
    int shared_block_id = get_shared_block_Odometrie(filename);
    struct SharedMemoryODO *result;

    if (shared_block_id == IPC_RESULT_ERROR)
    {
        return NULL;
    }

    result = (struct SharedMemoryODO *)shmat(shared_block_id, NULL, 0);
    if (result == (struct SharedMemoryODO *)IPC_RESULT_ERROR)
    {
        return NULL;
    }

    return result;
}

bool detach_memory_block(const char *block)
{
    return (shmdt(block) != IPC_RESULT_ERROR);
}

bool detach_memory_block_LIDAR(struct SharedMemoryLIDAR *block)
{
    return (shmdt(block) != IPC_RESULT_ERROR);
}

bool detach_memory_block_Odometrie(struct SharedMemoryODO *block)
{
    return (shmdt(block) != IPC_RESULT_ERROR);
}

bool destroy_memory_block(const char *filename)
{
    int shared_block_id = get_shared_block(filename, 0);

    if (shared_block_id == IPC_RESULT_ERROR)
    {
        return NULL;
    }
    return (shmctl(shared_block_id, IPC_RMID, NULL) != IPC_RESULT_ERROR);
}