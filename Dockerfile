FROM ros:melodic

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-melodic-rviz
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# melodic
RUN apt-get update -y
RUN apt-get install ros-melodic-ros-control -y
RUN apt-get install ros-melodic-ros-controllers -y
RUN apt-get install ros-melodic-gazebo-ros -y
RUN apt-get install ros-melodic-gazebo-ros-control -y
RUN apt-get install ros-melodic-rqt-robot-steering -y
RUN apt-get install ros-melodic-teleop-twist-keyboard -y
RUN apt-get install ros-melodic-roslint -y
RUN apt-get install ros-melodic-xacro -y
RUN apt-get install ros-melodic-robot-state-publisher -y
RUN apt-get install ros-melodic-geodesy -y
RUN apt-get install ros-melodic-pcl-ros  -y
RUN apt-get install ros-melodic-nmea-msgs -y
RUN apt-get install ros-melodic-libg2o -y

# networking
RUN apt-get install net-tools -y
RUN apt-get install iputils-ping -y
RUN apt-get install nmap -y

# serial
RUN pip3 install pyserial

RUN mkdir -p /ese650/src
RUN source /opt/ros/melodic/setup.bash && \
    cd ese650/ && \
    apt-get update --fix-missing && \
    rosdep install --from-paths src --ignore-src -r -y
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bash_aliases
RUN echo "alias .s='source devel/setup.bash'" >> ~/.bash_aliases

RUN echo "ROS_MASTER_URI=http://192.168.84.187:11311" >> ~/.bash_aliases
RUN echo "ROS_IP=192.168.84.185" >> ~/.bash_aliases
RUN source /opt/ros/melodic/setup.bash

WORKDIR '/ese650/src'
ENTRYPOINT ["/bin/bash"]