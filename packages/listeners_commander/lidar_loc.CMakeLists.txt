cmake_minimum_required(VERSION 3.16)
# define the project name
set(PROJECT_NAME lidar_loc)
project(${PROJECT_NAME})



# the program REQUIRES features that are only available from C++17 and onward
set(CMAKE_CXX_STANDARD 17)
# load dependencies and include locations for all projects
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# fech the latest Google Test Library release
# from: https://google.github.io/googletest/quickstart-cmake.html
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)


# ------------- LIBRARIES AND PROGRAMS ----------------
include_directories(include)
# the library that enables to detect the position of a circle from lidar data
add_library(${PROJECT_NAME}_circledetection_lib
    src/CircleDetection.cpp
)
target_link_libraries(${PROJECT_NAME}_circledetection_lib
    Eigen3::Eigen
)
add_library(${PROJECT_NAME}_fusion_lib
    src/Fusion.cpp
)
target_link_libraries(${PROJECT_NAME}_fusion_lib
    Eigen3::Eigen
)
# -------------------- TESTS --------------------------
enable_testing()
add_executable(${PROJECT_NAME}_circledetection_lib_test
    src/tests/circle_detection.cpp
)
target_link_libraries(${PROJECT_NAME}_circledetection_lib_test
    GTest::gtest_main
    ${PROJECT_NAME}_circledetection_lib
)
add_executable(${PROJECT_NAME}_fusion_lib_test
    src/tests/fusion.cpp
)
target_link_libraries(${PROJECT_NAME}_fusion_lib_test
    GTest::gtest_main
    ${PROJECT_NAME}_fusion_lib
)
include(GoogleTest)
gtest_discover_tests(
    ${PROJECT_NAME}_circledetection_lib_test
    ${PROJECT_NAME}_fusion_lib_test
)
