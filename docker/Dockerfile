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
RUN useradd -ms /bin/bash user_cbgl
USER user_cbgl
WORKDIR /home/user_cbgl

COPY map.png /home/user_cbgl
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/user_cbgl/.bashrc
RUN echo "source /home/user_cbgl/catkin_ws/devel/setup.bash" >> /home/user_cbgl/.bashrc

RUN rosdep update
RUN mkdir -p /home/user_cbgl/catkin_ws/src && \
    cd /home/user_cbgl/catkin_ws/src/ && \
    git clone https://github.com/li9i/cbgl.git && \
    cd /home/user_cbgl/catkin_ws && \
    export CC=gcc && \
    export CXX=g++ && \
    alias g++='g++ -std=c++11' && \
    alias clang++='clang++ -std=c++11' && \
    source /opt/ros/kinetic/setup.bash && \
    catkin build cbgl && \
    source /opt/ros/kinetic/setup.bash && \
    source /home/user_cbgl/catkin_ws/devel/setup.bash

# The next five lines + the entrypoint command will make sure that
# when the container is run the cbgl node is roslaunched immediately
RUN echo "#!/bin/bash" > /home/user_cbgl/cbgl_launch.sh
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/user_cbgl/cbgl_launch.sh
RUN echo "source /home/user_cbgl/catkin_ws/devel/setup.bash" >> /home/user_cbgl/cbgl_launch.sh
RUN echo "roslaunch cbgl cbgl.launch" >> /home/user_cbgl/cbgl_launch.sh
RUN chmod +x /home/user_cbgl/cbgl_launch.sh

ENTRYPOINT bash /home/user_cbgl/cbgl_launch.sh
