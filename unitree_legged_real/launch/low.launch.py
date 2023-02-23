
"""
Launches low level go controls for the go1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
     return LaunchDescription([

          DeclareLaunchArgument(
            name='enable_cmd',
            default_value='false',
            choices=['true','false'],
            description='Determines whether to start a robot model '+
                        'corresponding to the /low_cmd topic (i.e., desired positions)'),

          DeclareLaunchArgument(name='use_rviz', default_value='true',
                                choices=['true', 'false'],
                                description='Choose if rviz is launched'),

          SetLaunchConfiguration(name='rviz_state_only',
                                 value=PathJoinSubstitution(
                                   [FindPackageShare('unitree_legged_real'),
                                                     'config',
                                                     'state_only.rviz'])),

          # Launch rviz
          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    '-d', LaunchConfiguration('rviz_state_only'),
               ],
               condition=IfCondition(LaunchConfiguration('use_rviz'))
          ),

          # Always run the udp!
          Node(package='unitree_legged_real',
               executable='udp_low',
               output='screen'),

          # Conditionally run the jsp's based on launch arguments
          Node(package='unitree_legged_real',
               executable='jsp_low',
               parameters=[{"js_source": "cmd"}],
               namespace="cmd",
               condition=IfCondition(LaunchConfiguration('enable_cmd')),
               output='screen'),

          Node(package='unitree_legged_real',
               executable='jsp_low',
               parameters=[{"js_source": "state"}],
               namespace="state",
               output='screen'),

          # Load robot models depending on enable_cmd
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'launch',
                                                           'load_go1.launch.py'])),
                    launch_arguments=[
                         ('use_jsp', 'none'),
                         ('namespace', 'cmd'),
                         ('use_rviz', 'false')
                    ],
               condition=IfCondition(LaunchConfiguration('enable_cmd'))
          ),

          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'launch',
                                                           'load_go1.launch.py'])),
                    launch_arguments=[
                         ('use_jsp', 'none'),
                         ('namespace', 'state'),
                         ('use_rviz', 'false')
                    ],
          ),

          
    ])