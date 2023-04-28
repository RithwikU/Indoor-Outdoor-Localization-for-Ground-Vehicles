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

# networking
RUN apt-get install net-tools -y
RUN apt-get install iputils-ping -y
RUN apt-get install nmap -y

# serial
RUN pip3 install pyserial

COPY . /sick_ws/src/SickTiM
RUN source /opt/ros/melodic/setup.bash && \
    cd sick_ws/ && \
    apt-get update --fix-missing && \
    rosdep install --from-paths src --ignore-src -r -y
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bash_aliases
RUN echo "alias .s='source devel/setup.bash'" >> ~/.bash_aliases
RUN source /opt/ros/melodic/setup.bash && \
    cd sick_ws/ && \
    catkin_make

WORKDIR '/ese650/src'
ENTRYPOINT ["/bin/bash"]