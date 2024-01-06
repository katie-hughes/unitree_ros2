# Unitree ROS2

**Authors**: Katie Hughes, Nick Morales

This is a set of ROS2 packages designed to control the Unitree Go1 robot. This repository was forked from 
[Unitree's 'Ros2 to Real'](https://github.com/unitreerobotics/unitree_ros2_to_real) package with some modifications to make adding custom control more streamlined. It also includes a copy of [unitree_legged_sdk v3.5.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.5.1).

## How to run
1. Clone this repository in `unitree_ws/src`.
2. In `unitree_ws`, run `colcon build`. There is a warning about `BOOST_PRAGMA_MESSAGE` that I do not know how to resolve at the moment, but otherwise should build without error.
3. In another terminal, run `source install/setup.bash`
   
   (If running the low level controls, ensure that the dog is raised off the ground in the harness before executing the next steps!! Before raising it off the floor, you will need to press L2+A to sit the robot down and then L1+L2+start to put the robot into the correct mode -- otherwise, the robot will flail its legs when it leaves the floor.)

4. To load the robot and connect to it via UDP communication, run `ros2 launch unitree_legged_real low.launch.py` (low level controls) or `ros2 launch unitree_legged_real high.launch.py` (high level controls).

## Low Level Executables:
Again, before running any low level controls, ensure that the dog is safely raised off the ground in the harness.
* `udp_low`: This takes the framework defined in Unitree's `ros2_udp` and translates it into a more traditional ROS2 C++ node. Additionally, this also fixes a bug where you can only read robot state messages if you are currently publishing joint commands. Finally, this also adds a joint state publisher that is connected to the joint messages received for visualizaiton in rviz.
* `jsp_low`: ROS2 C++ node for low level joint state publishing. It subscribes either to the `low_state` or `low_cmd` topic (depending on the `js_source` parameter) and publishes joint states present in the message.
* `custom_gait`: This takes the framework defined in Unitree's `ros2_position_example` and translates it into a more traditional ROS2 C++ node. This also enables multi-joint control. In the future it will also subscribe to the `low_state` message from the Go1 in order for more precise control. In order for this node to physically move the dog, `custom_udp` must also be running.

## High Level Executables:
* `udp_high` - ROS2 C++ node for high level UDP pass command and state pass through. It subscribes to the `high_cmd` topic, converts these commands to a UDP message, and sends it to the Go1. It publishes state (received over UDP from the Go1) to the `high_state` topic.
* `jsp_high` - ROS2 C++ node for high level joint state publishing. It subscribes to the `high_state` topic and publishes joint states present in the state message.

## Dependencies
This package is designed to run on Ubuntu 22.04 with ROS 2 Humble. In addition to these standard ROS dependencies, there are also dependencies needed to build the Unitree Legged SDK.
### lcm 1.4.0
* Download lcm version 1.4.0 from [lcm releases](https://github.com/lcm-proj/lcm/releases) and unzip.
* For this version of lcm, you need java openjdk 8. Don't uninstall other versions of openjdk! This can interfere with your installation of ROS 2. Instead, do the following: 
```
sudo apt install openjdk-8-jdk
sudo update-alternatives --config javac
# choose the java-8-openjdk option
cd lcm-1.4.0
mkdir build
cd build
cmake ..
make
sudo make install # this will install lcm
sudo ldconfig -v # updates the shared library cache
# after this, revert back to the previous version of openjdk:
sudo update-alternatives --config javac # and choose whichever version was selected previously (typically java-11-openjdk)
```
