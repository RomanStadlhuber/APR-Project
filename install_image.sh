#!/bin/bash

apt-get update
apt-get install -y -q \
less htop  byobu gosu python3-pip git vim tree python3-pip \
ros-noetic-amcl ros-noetic-angles ros-noetic-base-local-planner ros-noetic-clear-costmap-recovery ros-noetic-global-planner* \
ros-noetic-costmap-2d ros-noetic-diagnostic-updater ros-noetic-hls-lfcd-lds-driver ros-noetic-interactive-markers \
ros-noetic-joint-state-publisher ros-noetic-kdl-parser ros-noetic-laser-geometry ros-noetic-map-msgs \
ros-noetic-map-server ros-noetic-move-base ros-noetic-move-base-msgs ros-noetic-nav-core ros-noetic-navfn \
ros-noetic-robot-state-publisher ros-noetic-rotate-recovery ros-noetic-dwa-local-planner* \
ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-geometry-msgs ros-noetic-tf2-kdl ros-noetic-tf2-msgs ros-noetic-tf2-py \
ros-noetic-tf2-ros ros-noetic-turtlebot3 ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-description \
ros-noetic-turtlebot3-example ros-noetic-turtlebot3-navigation ros-noetic-turtlebot3-slam \
ros-noetic-turtlebot3-teleop ros-noetic-urdf ros-noetic-voxel-grid ros-noetic-xacro \
ros-noetic-rosdoc-lite ros-noetic-gmapping ros-noetic-rqt* ros-noetic-gazebo-ros ros-noetic-gazebo-plugins* \
ros-noetic-pid ros-noetic-turtlebot3-simulations \
checkinstall python-dev python-numpy libtbb2 libgtest-dev doxygen \
libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev cmake git pkg-config \
libavcodec-dev libavformat-dev libswscale-dev cmake git libgtk2.0-dev pkg-config libavcodec-dev \
libavformat-dev libswscale-dev libopencv-dev build-essential checkinstall  cmake pkg-config \
yasm libjpeg-dev libswscale-dev libdc1394-22-dev libxine2-dev  libv4l-dev python-dev python-numpy \
libtbb-dev  qtbase5-dev  libgtk2.0-dev libfaac-dev libmp3lame-dev  libopencore-amrnb-dev \
libopencore-amrwb-dev libtheora-dev  libvorbis-dev libxvidcore-dev x264 v4l-utils ffmpeg \
libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev


pip3 install opencv-contrib-python
