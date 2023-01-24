
"""
Launches low level go controls for the go1
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

          Node(package='unitree_legged_real',
               executable='custom_udp',
               output='screen'),

          Node(package='unitree_legged_real',
               executable='custom_gait',
               output='screen')
    ])