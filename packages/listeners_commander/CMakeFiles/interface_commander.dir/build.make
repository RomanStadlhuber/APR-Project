# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander

# Include any dependencies generated for this target.
include CMakeFiles/interface_commander.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/interface_commander.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/interface_commander.dir/flags.make

CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.o: CMakeFiles/interface_commander.dir/flags.make
CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.o: src/TCPEchoClient_Commander.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/grilli/MF_Turtles/APR-Project/packages/listeners_commander/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.o -c /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander/src/TCPEchoClient_Commander.cpp

CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander/src/TCPEchoClient_Commander.cpp > CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.i

CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander/src/TCPEchoClient_Commander.cpp -o CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.s

# Object files for target interface_commander
interface_commander_OBJECTS = \
"CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.o"

# External object files for target interface_commander
interface_commander_EXTERNAL_OBJECTS =

interface_commander: CMakeFiles/interface_commander.dir/src/TCPEchoClient_Commander.cpp.o
interface_commander: CMakeFiles/interface_commander.dir/build.make
interface_commander: libinterface_shared_mem_lib.a
interface_commander: CMakeFiles/interface_commander.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/grilli/MF_Turtles/APR-Project/packages/listeners_commander/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable interface_commander"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/interface_commander.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/interface_commander.dir/build: interface_commander

.PHONY : CMakeFiles/interface_commander.dir/build

CMakeFiles/interface_commander.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/interface_commander.dir/cmake_clean.cmake
.PHONY : CMakeFiles/interface_commander.dir/clean

CMakeFiles/interface_commander.dir/depend:
	cd /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander /home/grilli/MF_Turtles/APR-Project/packages/listeners_commander/CMakeFiles/interface_commander.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/interface_commander.dir/depend

