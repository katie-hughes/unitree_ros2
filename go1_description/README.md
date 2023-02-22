# Go1 Description

**Author**: Katie Hughes, Nick Morales

This is a ROS2 package to visualize the Unitree Go1 in rviz. 

The URDF and meshes were taken from the
[unitree_ros](https://github.com/unitreerobotics/unitree_ros) package, and modified for use in ros2.

![Go1 rviz visualization](images/go1_rviz.png?raw=true "Go1 rviz visualization")

## To Run:
Run `ros2 launch go1_description load_go1.launch.py` to load the robot model.

### Launch Arguments:
  * `use_jsp`: Choose if joint_state_publisher is launched. Valid choices are: ['gui', 'jsp', 'none']. Default is 'gui'.
  * `use_rviz`: Choose if rviz is launched. Valid choices are: ['true', 'false']. Default is 'true'.
  * `use_nav2_links`: Use Nav2 frames in URDF. Valid choices are: ['true', 'false']. Default is 'false'.
  * `fixed_frame`: Fixed frame for RVIZ. Default is 'base'.
  * `namespace`: Choose a namespace for the launched topics. Default is ''.



