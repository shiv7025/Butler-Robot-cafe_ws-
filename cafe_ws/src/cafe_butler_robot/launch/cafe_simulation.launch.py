from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = 'cafe_butler_robot'
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), 'urdf', 'butler_robot.urdf.xacro']
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name), 'worlds', 'cafe_world.world']
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    order_manager_node = Node(
        package=pkg_name,
        executable='order_manager',
        name='order_manager',
        output='screen'
    )

    robot_controller_node = Node(
        package=pkg_name,
        executable='robot_controller',
        name='robot_controller',
        output='screen'
    )

    cafe_gui_node = Node(
        package=pkg_name,
        executable='cafe_gui',
        name='cafe_gui',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        order_manager_node,
        robot_controller_node,
        cafe_gui_node
    ])
