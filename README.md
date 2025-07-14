Obstacle Avoidance LiDAR — ROS 2 Autonomous Navigation Project

Welcome to the Obstacle Avoidance LiDAR ROS 2 package — a robotics project demonstrating autonomous obstacle detection and avoidance using LiDAR data within a Gazebo simulation environment.

🚀 Project Overview

This project implements real-time obstacle avoidance for a differential-drive robot by processing LiDAR sensor inputs. It publishes velocity commands to navigate the robot safely around obstacles in a simulated environment using Gazebo, leveraging ROS 2 Humble’s ecosystem and Python nodes.


🌟 Key Features

    Real-time LiDAR sensor data processing for dynamic obstacle detection

    Autonomous velocity command generation enabling smooth obstacle avoidance

    Fully integrated with Gazebo simulation for realistic robot behavior testing

    Modular and reusable ROS 2 Python nodes following best practices

    Comprehensive launch files for quick startup and testing

    Well-structured package including models, resources, and tests

📂 Package Structure
```
obstacle_avoidance_lidar/
├── obstacle_avoidance/      # Python ROS 2 nodes implementing navigation logic
├── launch/                  # Launch files to start simulation and nodes
├── models/                  # Gazebo robot models and meshes
├── resource/                # Additional package resources
├── test/                    # Unit and integration tests
├── package.xml              # ROS 2 package metadata
├── setup.py                 # Python build configuration
├── setup.cfg                # Python package setup
└── README.md                # Project documentation
```

🔧 Prerequisites

    ROS 2 Humble

    Gazebo Simulator (compatible with ROS 2 Humble)

    Python 3.x

🛠️ Installation & Usage

    Clone the repository inside your ROS 2 workspace src folder:
```bash
cd ~/ros2_ws/src
git clone https://github.com/parveezsyed28/obstacle_avoidance_lidar.git
```
    Build the workspace and source the environment:
    
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

    Launch the Gazebo simulation and the obstacle avoidance node:
    
```bash
ros2 launch obstacle_avoidance state_publisher.launch.py
```

📈 How It Works

    The LiDAR sensor data is published on a ROS 2 topic and continuously processed by a Python node.

    The node analyzes scan data to detect obstacles in the robot’s path.

    Based on obstacle proximity and position, velocity commands (/cmd_vel) are published to maneuver the robot safely.

    The Gazebo simulation reflects the robot’s real-time movements as it autonomously avoids collisions.

🎯 Learning Outcomes & Skills Demonstrated

    Mastery of ROS 2 Python programming and node communication

    Practical experience with sensor integration (LiDAR) in robotics

    Familiarity with Gazebo simulation environment for testing robotics algorithms

    Understanding of autonomous navigation and obstacle avoidance algorithms

    Efficient workspace management and launch file configuration

