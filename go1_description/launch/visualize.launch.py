
"""
Launches rviz with the go1 urdf file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='use_jsp', default_value='gui',
                              choices=['gui', 'jsp', 'none'],
                              description='Choose if joint_state_publisher is launched'),
        DeclareLaunchArgument(name='use_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Choose if rviz is launched'),
       DeclareLaunchArgument(name='use_gz', default_value='true',
                              choices=['true', 'false'],
                              description='Choose if gazebo is launched'),

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
             on_exit = Shutdown()),

     IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                                                           'launch',
                                                           'ign_gazebo.launch.py'])),
          condition=LaunchConfigurationEquals('use_gz', 'true')),
          # launch_arguments={'ign_args': '-r '+str(world_path)}.items()),

     Node(package='ros_ign_gazebo',
          executable='create',
          arguments=['-name', 'go1',
                    '-topic', 'robot_description',
                    '-z', '5.0'],
          output='screen',
          condition=LaunchConfigurationEquals('use_gz', 'true'))
    ])