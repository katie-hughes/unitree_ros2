
"""
Launches rviz with the go1 urdf file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='use_jsp', default_value='gui',
                              choices=['gui', 'jsp', 'none'],
                              description='Choose if joint_state_publisher is launched'),
        DeclareLaunchArgument(name='use_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Choose if rviz is launched'),

        SetLaunchConfiguration(name='config_file',
                               value='go1.rviz'),
        SetLaunchConfiguration(name='model',
                              value=PathJoinSubstitution([FindPackageShare('go1_description'),
                                                          'urdf',
                                                          'go1.urdf.xacro'])),
        SetLaunchConfiguration(name='rvizconfig',
                               value=PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'config',
                                                           LaunchConfiguration('config_file')])),

        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             condition=LaunchConfigurationEquals('use_jsp', 'jsp')),

       Node(
          package='joint_state_publisher_gui',
          executable='joint_state_publisher_gui',
          condition=LaunchConfigurationEquals('use_jsp', 'gui')),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description':
                          ParameterValue(Command(['xacro ',
                                                  LaunchConfiguration('model')]),
                                         value_type=str)}]),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', LaunchConfiguration('rvizconfig')],
             condition=LaunchConfigurationEquals('use_rviz', 'true'),
             on_exit = Shutdown())
    ])