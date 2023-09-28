FROM ubuntu:16.04
MAINTAINER Alexandros Philotheou alefilot@auth.gr

RUN apt-get update
RUN apt-get install -y sudo apt-utils build-essential g++ git libfftw3-dev libcgal-dev=4.7-4 libcgal-qt5-dev=4.7-4 libboost-random-dev curl

RUN apt-get install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update
RUN apt install -y python-catkin-tools ros-kinetic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep ros-kinetic-csm
RUN rosdep init

# Use bash and create user
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN useradd -ms /bin/bash li9i
USER li9i
WORKDIR /home/li9i

COPY map.png /home/li9i
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/li9i/.bashrc
RUN echo "source /home/li9i/catkin_ws/devel/setup.bash" >> /home/li9i/.bashrc

RUN rosdep update
RUN mkdir -p /home/li9i/catkin_ws/src && \
    cd /home/li9i/catkin_ws/src/ && \
    git clone https://github.com/li9i/cbgl.git && \
    cd /home/li9i/catkin_ws && \
    export CC=gcc && \
    export CXX=g++ && \
    alias g++='g++ -std=c++11' && \
    alias clang++='clang++ -std=c++11' && \
    source /opt/ros/kinetic/setup.bash && \
    catkin build cbgl && \
    source /opt/ros/kinetic/setup.bash && \
    source /home/li9i/catkin_ws/devel/setup.bash

# The next five lines + the entrypoint command will make sure that
# when the container is run the cbgl node is roslaunched immediately
RUN echo "#!/bin/bash" > /home/li9i/cbgl_launch.sh
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/li9i/cbgl_launch.sh
RUN echo "source /home/li9i/catkin_ws/devel/setup.bash" >> /home/li9i/cbgl_launch.sh
RUN echo "roslaunch cbgl cbgl.launch map_png_file:=/home/li9i/map.png" >> /home/li9i/cbgl_launch.sh
RUN chmod +x /home/li9i/cbgl_launch.sh

ENTRYPOINT bash /home/li9i/cbgl_launch.sh
