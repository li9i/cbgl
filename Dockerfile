FROM osrf/ros:kinetic-desktop
MAINTAINER Alexandros Philotheou alefilot@auth.gr

RUN apt-get update
RUN apt-get install -y sudo apt-utils build-essential g++ git libfftw3-dev libcgal-dev=4.7-4 libcgal-qt5-dev=4.7-4 libboost-random-dev curl lsb-release python-catkin-tools python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep ros-kinetic-csm ros-kinetic-pcl-ros

# Use bash and create user
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN useradd -ms /bin/bash user_cbgl
USER user_cbgl
WORKDIR /home/user_cbgl

RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/user_cbgl/.bashrc
RUN echo "source /home/user_cbgl/catkin_ws/devel/setup.bash" >> /home/user_cbgl/.bashrc

RUN rosdep update
RUN mkdir -p /home/user_cbgl/catkin_ws/src && \
    cd /home/user_cbgl/catkin_ws/src/

COPY cbgl/ /home/user_cbgl/catkin_ws/src/cbgl/

RUN cd /home/user_cbgl/catkin_ws && \
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
