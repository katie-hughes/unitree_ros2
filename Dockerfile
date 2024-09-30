FROM osrf/ros:humble-desktop-full

# install LCM

RUN sudo apt-get update
RUN yes | sudo apt-get -y install unzip
RUN yes | sudo apt-get -y install openjdk-8-jdk

# sudo update-alternatives --config javac
# and choose java-8-openjdk (here option 2)

RUN echo 2 | sudo update-alternatives --config javac

WORKDIR /lcm
RUN curl -O -L https://github.com/lcm-proj/lcm/releases/download/v1.4.0/lcm-1.4.0.zip
RUN unzip -q lcm-1.4.0.zip && cd lcm-1.4.0 && mkdir build && cd build && cmake .. && make && sudo make install


RUN mkdir -p /unitree_ws/src/unitree_ros2
WORKDIR /
COPY go1_description/ /unitree_ws/src/unitree_ros2/go1_description
COPY ros2_unitree_legged_msgs/ /unitree_ws/src/unitree_ros2/ros2_unitree_legged_msgs
COPY unitree_legged_real/ /unitree_ws/src/unitree_ros2/unitree_legged_real

WORKDIR /unitree_ws