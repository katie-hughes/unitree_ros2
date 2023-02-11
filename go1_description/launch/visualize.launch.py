
"""
Launches rviz with the go1 urdf file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration, \
     IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, \
     TextSubstitution
from launch.conditions import LaunchConfigurationEquals, IfCondition
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
          DeclareLaunchArgument(name='use_gz', default_value='false',
                                choices=['true', 'false'],
                                description='Choose if gazebo is launched'),
          DeclareLaunchArgument(name='enable_base_footprint', default_value='false',
                                choices=['true', 'false'],
                                description='Enable robot base footprint link'),
          DeclareLaunchArgument(name='enable_lidar', default_value='false',
                                choices=['true', 'false'],
                                description='Enable lidar link'),
          DeclareLaunchArgument(name='publish_static_world_tf', default_value='false',
                                choices=['true', 'false'],
                                description=
                                   'Publish a static world transform to the base of the robot'),
          DeclareLaunchArgument(name='fixed_frame', default_value='base',
                                description='Fixed frame for RVIZ'),

        #   SetLaunchConfiguration(name='config_file',
        #                          value='go1.rviz'),
          SetLaunchConfiguration(name='config_file',
                                 value='go2.rviz'),
          SetLaunchConfiguration(name='model',
                                 value=PathJoinSubstitution([FindPackageShare('go1_description'),
                                                             'xacro',
                                                             'robot.xacro'])),
          SetLaunchConfiguration(name='rvizconfig',
                                 value=PathJoinSubstitution([FindPackageShare('go1_description'),
                                                             'config',
                                                             LaunchConfiguration('config_file')])),

          Node(package='joint_state_publisher',
               executable='joint_state_publisher',
               condition=LaunchConfigurationEquals('use_jsp', 'jsp')),

          Node(package='joint_state_publisher_gui',
               executable='joint_state_publisher_gui',
               condition=LaunchConfigurationEquals('use_jsp', 'gui')),

          Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{
                    'robot_description':
                         ParameterValue(
                              Command([
                                   'xacro ',
                                   LaunchConfiguration('model'),
                                   TextSubstitution(text=' enable_base_footprint:='),
                                   LaunchConfiguration('enable_base_footprint'),
                                   TextSubstitution(text=' enable_lidar:='),
                                   LaunchConfiguration('enable_lidar')
                              ]),
                              value_type=str
                         )
               }]
          ),

          Node(package="tf2_ros",
               executable="static_transform_publisher",
               arguments=['--frame-id', 'world', '--child-frame-id', 'base', '--z', '0.5'],
               condition=IfCondition(LaunchConfiguration('publish_static_world_tf'))),

          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    '-d', LaunchConfiguration('rvizconfig'),
                    '-f', LaunchConfiguration('fixed_frame')
               ],
               condition=IfCondition(LaunchConfiguration('use_rviz')),
               on_exit = Shutdown()),

          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                                                           'launch',
                                                           'ign_gazebo.launch.py'])),
               condition=IfCondition(LaunchConfiguration('use_gz'))),
               # launch_arguments={'ign_args': '-r '+str(world_path)}.items()),

          Node(package='ros_ign_gazebo',
               executable='create',
               arguments=['-name', 'go1',
                          '-topic', 'robot_description',
                          '-z', '5.0'],
               output='screen',
               condition=IfCondition(LaunchConfiguration('use_gz'))),

          Node(package='ros_gz_bridge',
               executable='parameter_bridge',
               arguments=["/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                          "/world/ddrive_scene/model/go1/joint_state@" +
                          "sensor_msgs/msg/JointState[ignition.msgs.Model"],
               remappings=[('/world/ddrive_scene/model/go1/joint_state',
                              'joint_states')],
               condition=IfCondition(LaunchConfiguration('use_gz')))
    ])