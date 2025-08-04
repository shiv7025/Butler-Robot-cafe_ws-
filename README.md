# Butler-Robot-cafe_ws

# overview
This project provides the ROS 2 package ```cafe_butler_robot```, which lays the foundation for an autonomous butler robot designed to operate in a simulated café environment. The implementation focuses on simulation, robot modeling, and launching the necessary components for autonomous navigation development.

The repository contains a simulated café environment for Gazebo, a differential drive robot model described in URDF, and the necessary launch files and configurations to bring up the simulation. This package is a self-contained unit for developing and testing the core functionalities of a robotic butler.

# Features
* Custom Gazebo World: Includes a custom-built café world (```cafe_world.world```) for realistic simulation.
* Detailed Robot Model: A complete robot model is defined using URDF and XACRO, with associated meshes (```.STL``` files) for visualization.
* Simulation Launch Files: Provides pre-configured launch files to easily start the Gazebo simulation with the robot.
* Application Logic: Contains Python scripts for the robot's control logic, order management, and a graphical user interface (```cafe.gui.py```).
* Configuration Management: A ```config``` directory holds YAML files for managing robot and environment parameters, such as the café layout.

# System Requirements
* OS: Ubuntu 22.04 LTS (or 20.04 LTS)
* ROS 2 Version: ROS 2 Humble Hawksbill (or a similar version)
* Simulation: Gazebo
* Build Tool: ```colcon```

# Technical Architecture
The system's architecture is built around standard ROS 2 practices, with a clear separation of concerns.
* Simulation (Gazebo): The ```world/``` directory contains the Gazebo world file. The ```launch/``` directory has scripts like ```gazebo_simulation.launch.py``` to start Gazebo and spawn the robot.
* Robot Model (URDF): The robot's physical structure, sensors, and properties are defined in the ```urdf/``` directory using ```.urdf.xacro files```. The ```meshes/``` directory provides the 3D models for the robot's visual representation.
* Robot Logic (Python): The core application logic resides in the ```cafe_butler_robot/``` subdirectory. This includes a ```robot_controller.py``` for movement, an ```order_manager.py``` for handling tasks, and a ```cafe.gui.py``` for user interaction.
* Configuration: The ```config/cafe_layout.yaml``` file is used to store static parameters about the environment, making it easy to adjust the layout without changing code.
* Visualization (RViz): Launch files are typically configured to start RViz for visualizing sensor data, robot state, and navigation goals.

# Installation and Setup
This section guides you through the one-time setup required to prepare the project.

* Prerequisites: Ensure you have a complete installation of ROS 2 (e.g., Humble) and Gazebo on your system.
*Create Workspace: Create a ROS 2 workspace and a ```src``` directory inside it.
```
mkdir -p ~/cafe_ws/src
cd ~/cafe_ws/src
```
* Add Package: Place the ```cafe_butler_robot``` package inside the ```src``` directory.
* Install Dependencies: Navigate to your workspace root and use ```rosdep``` to install any missing dependencies.
```
cd ~/cafe_ws
rosdep install --from-paths src -y --ignore-src
```
* Build the Workspace: Use ```colcon``` to build the packages in your workspace.
```
cd ~/cafe_ws
colcon build --symlink-install
```
# Usage
Follow these steps each time you want to run the simulation.

* Navigate to your Workspace: Open a new terminal and change to your workspace directory.
```
cd ~/cafe_ws
```
* Source the Setup File: You must source the workspace's setup file to make its packages available in the terminal.
```
source install/setup.bash
```
* Launch the Simulation: Run the main launch file to start Gazebo, load the custom café world, and spawn the robot.
```
ros2 launch cafe_butler_robot cafe_simulation.launch.py
```
* ```ros2 launch```: The command to run a ROS 2 launch file.
* ```cafe_butler_robot```: The name of the package containing the launch file.
* ```cafe_simulation.launch.py```: The specific launch file that starts the full simulation.

# Project Structure
```
cafe_butler_robot/
├── cafe_butler_robot/
│   ├── cafe.gui.py
│   ├── order_manager.py
│   ├── robot_controller.py
│   └── __init__.py
├── config/
│   └── cafe_layout.yaml
├── launch/
│   ├── cafe_simulation.launch.py
│   └── gazebo_simulation.launch.py
├── meshes/
│   ├── base_link.STL
│   ├── caster_front_link.STL
│   ├── caster_rear_link.STL
│   ├── imu_link.STL
│   ├── wheel_left_link.STL
│   └── wheel_right_link.STL
├── resource/
│   └── cafe_butler_robot
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── urdf/
│   ├── butler_robot.gazebo.xacro
│   └── butler_robot.urdf.xacro
├── world/
│   └── cafe_world.world
├── package.xml
├── setup.cfg
└── setup.py
```
