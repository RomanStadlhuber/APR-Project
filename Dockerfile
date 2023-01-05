FROM osrf/ros:noetic-desktop-full-focal

# deactivate interactive package manager
ENV DEBIAN_FRONTEND noninteractive

# install locales
RUN apt update
RUN apt install apt-utils wget locales -y

# Set the locale
RUN locale-gen en_US.UTF-8
RUN update-locale LANG=en_US.UTF-8

# set linke library path
ENV LD_LIBRARY_PATH /usr/local/lib
ENV TURTLEBOT3_MODEL=burger

# copy vim configuration
WORKDIR /root/.config/nvim
COPY init.vim .
# create t
WORKDIR /catkin_ws
COPY install_image.sh .

RUN bash install_image.sh
