
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
            name='enable_cmd',
            default_value='true',
            choices=['true','false'],
            description='Determines whether to start a robot model '+
                        'corresponding to the /low_cmd topic (i.e., desired positions)'),

          DeclareLaunchArgument(
            name='enable_state',
            default_value='true',
            choices=['true','false'],
            description='Determines whether to start a robot model '+
                        'corresponding to the /low_state topic (i.e., real positions)'),

          DeclareLaunchArgument(name='use_rviz', default_value='true',
                                choices=['true', 'false'],
                                description='Choose if rviz is launched'),

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
               condition=IfCondition(LaunchConfiguration('enable_state')),
               output='screen'),

          # Load robot models depending on state
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'launch',
                                                           'load_go1.launch.py'])),
                    launch_arguments=[
                         ('use_jsp', 'none'),
                         ('namespace', 'cmd'),
                         ('use_rviz', LaunchConfiguration('use_rviz'))
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
                         ('use_rviz', LaunchConfiguration('use_rviz'))
                    ],
               condition=IfCondition(LaunchConfiguration('enable_state'))
          ),

          # Generic model that gets loaded if both enable_cmd and enable_state are false
          # With no namespaces
          # IncludeLaunchDescription(
          #      PythonLaunchDescriptionSource(
          #           PathJoinSubstitution([FindPackageShare('go1_description'),
          #                                                  'launch',
          #                                                  'load_go1.launch.py'])),
          #           launch_arguments=[
          #                ('use_jsp', 'none'),
          #                ('use_rviz', LaunchConfiguration('use_rviz'))
          #           ],
          #      condition=IfCondition(LaunchConfiguration('enable_cmd'))
          # ),
    ])