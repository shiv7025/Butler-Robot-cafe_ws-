import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Step 1: Configuration and Paths ---
    # Configure to use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define paths to your project files
    pkg_cafe_butler = get_package_share_directory('cafe_butler_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_cafe_butler, 'worlds', 'french_cafe.world')
    urdf_path = os.path.join(pkg_cafe_butler, 'urdf', 'butler_robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_cafe_butler, 'config', 'butler_robot.rviz')
    
    # --- Step 2: Process the Robot's URDF file ---
    # This reads the .xacro file and converts it to XML
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Step 3: Define the Nodes to Launch ---
    # Gazebo Server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Gazebo Client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher - This uses the processed robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn Robot Node - Spawns the robot into Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'butler_robot'],
        output='screen'
    )
    
    # Your project's nodes
    robot_controller_node = Node(package='cafe_butler_robot', executable='robot_controller')
    order_manager_node = Node(package='cafe_butler_robot', executable='order_manager')
    cafe_gui_node = Node(package='cafe_butler_robot', executable='cafe_gui')
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # --- Step 4: Create the Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        gzserver,
        gzclient,
        robot_state_publisher_node,
        spawn_entity_node,
        robot_controller_node,
        order_manager_node,
        cafe_gui_node,
        rviz_node
    ])