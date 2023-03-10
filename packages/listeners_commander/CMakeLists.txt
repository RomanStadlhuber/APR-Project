cmake_minimum_required(VERSION 3.16)
# define the project name
set(PROJECT_NAME interface)
project(${PROJECT_NAME})
# the program REQUIRES features that are only available from C++17 and onward
set(CMAKE_CXX_STANDARD 17)
# load dependencies and include locations for all projects
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# include relevant headers
include_directories(include)
# shared memory library
add_library(${PROJECT_NAME}_shared_mem_lib
    src/shared_memory.cpp
)
# controller library
add_library(${PROJECT_NAME}_controller_lib
    src/controller.cpp
)

# ------------- LIDAR CIRCLE DETECTION LIBRARY ----------------
include_directories(include)
# the library that enables to detect the position of a circle from lidar data
add_library(${PROJECT_NAME}_circledetection_lib
    src/CircleDetection.cpp
)
target_link_libraries(${PROJECT_NAME}_circledetection_lib
    Eigen3::Eigen
)
# ------------- POSE FUSION LIBRARY ----------------
add_library(${PROJECT_NAME}_fusion_lib
    src/Fusion.cpp
)
target_link_libraries(${PROJECT_NAME}_fusion_lib
    Eigen3::Eigen
)


# the odometry listener executable
add_executable(${PROJECT_NAME}_odometry
    src/TCPEchoClient_Odometrie.cpp
)
# add shmem dependency to listeners
target_link_libraries(${PROJECT_NAME}_odometry
    PRIVATE
    ${PROJECT_NAME}_shared_mem_lib
    ${PROJECT_NAME}_controller_lib
    Threads::Threads
)

# the Lidar listener executable
add_executable(${PROJECT_NAME}_lidar
    src/TCPEchoClient_Lidar.cpp
)
# add shmem dependency to listeners
target_link_libraries(${PROJECT_NAME}_lidar
    PRIVATE
    ${PROJECT_NAME}_shared_mem_lib
    ${PROJECT_NAME}_controller_lib
    Threads::Threads
    ${PROJECT_NAME}_circledetection_lib # the circle detection is made directly inside the lidar listener
)


# the commander listener executable
add_executable(${PROJECT_NAME}_commander
    src/TCPEchoClient_Commander.cpp
)
# add shmem dependency to listeners
target_link_libraries(${PROJECT_NAME}_commander
    PRIVATE
    ${PROJECT_NAME}_shared_mem_lib
    ${PROJECT_NAME}_controller_lib
    Threads::Threads
    ${PROJECT_NAME}_fusion_lib
)