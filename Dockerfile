FROM osrf/ros:humble-desktop-full

RUN sudo apt-get update
RUN yes | sudo apt-get -y install unzip
RUN yes | sudo apt-get -y install openjdk-8-jdk

# run sudo update-alternatives --config javac
# and choose java-8-openjdk (here option 2)
RUN echo 2 | sudo update-alternatives --config javac

# install LCM
WORKDIR /lcm
RUN curl -O -L https://github.com/lcm-proj/lcm/releases/download/v1.4.0/lcm-1.4.0.zip
RUN unzip -q lcm-1.4.0.zip && cd lcm-1.4.0 && mkdir build && cd build && cmake .. && make && sudo make install
RUN sudo ldconfig -v
# Revert to java-11-openjdk (here option 0)
RUN echo 0 | sudo update-alternatives --config javac

# Copy ROS packages over and build
RUN mkdir -p /unitree_ws/src/unitree_ros2
WORKDIR /
COPY go1_description/ /unitree_ws/src/unitree_ros2/go1_description
COPY ros2_unitree_legged_msgs/ /unitree_ws/src/unitree_ros2/ros2_unitree_legged_msgs
COPY unitree_legged_real/ /unitree_ws/src/unitree_ros2/unitree_legged_real
WORKDIR /unitree_ws
RUN . /opt/ros/humble/setup.sh && colcon build
