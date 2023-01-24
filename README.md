# Go1 Description

**Author**: Katie Hughes

This is a ros2 package to visualize the Unitree Go1 in rviz. 

The URDF and meshes were taken from the
[unitree_ros](https://github.com/unitreerobotics/unitree_ros) package, and modified for use in ros2.

![Go1 rviz visualization](go1_description/images/go1_rviz.png?raw=true "Go1 rviz visualization")

## To Run:
Run `ros2 launch go1_description visualize.launch.py` to see the Go1 in rviz with the jsp_gui.

### Launch Arguments:
  * `use_jsp`: Choose if joint_state_publisher is launched. Valid choices are: ['gui', 'jsp', 'none']. Default is 'gui'.
  * `use_rviz`: Choose if rviz is launched. Valid choices are: ['true', 'false']. Default is 'true'.
  * `use_gz`: Choose if gazebo is launched. Valid choices are: ['true', 'false']. Default is 'false'.
    * Gazebo currently is not functional with the meshes.