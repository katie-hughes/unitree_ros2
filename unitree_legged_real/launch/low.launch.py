
"""
Launches low level go controls for the go1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
     return LaunchDescription([

          DeclareLaunchArgument(
            name='js_source',
            default_value='state',
            choices=['cmd','state'],
            description='Determines whether sent commands (cmd) or current state (src) '+
                        'are published to the /joint_state topic.'),

          Node(package='unitree_legged_real',
               executable='udp_low',
               output='screen'),

          Node(package='unitree_legged_real',
               executable='jsp_low',
               parameters=[{"js_source": LaunchConfiguration('js_source')}],
               namespace=LaunchConfiguration('js_source'),
               output='screen'),
               
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'launch',
                                                           'visualize.launch.py'])),
                    launch_arguments=[
                         ('use_jsp', 'none'),
                         ('fixed_frame', 'world'),
                         ('namespace', LaunchConfiguration('js_source'))
                    ])
    ])