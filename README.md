# Obstacle Avoidance Lidar

This ROS 2 package implements obstacle avoidance for a robot using LiDAR sensor data. It includes launch files, robot models, and Python nodes for autonomous navigation and obstacle detection in a Gazebo simulation.

## Features

- Real-time LiDAR data processing
- Velocity command publishing for obstacle avoidance
- Launch files to start simulation and nodes
- Robot description and models for Gazebo integration

## Package Structure
```
obstacle_avoidance_lidar/
├── obstacle_avoidance/ # Python ROS 2 nodes
├── launch/ # Launch files
├── models/ # Gazebo models and meshes
├── resource/ # Package resources
├── test/ # Test scripts
├── package.xml # Package metadata
├── setup.py # Python build config
├── setup.cfg # Python setup config
└── README.md # This file
```

## Getting Started

### Prerequisites

- ROS 2 Humble
- Gazebo simulator
- Python 3

### Build and Run

1. Clone the repository into your ROS 2 workspace `src`:
    ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/parveezsyed28/obstacle_avoidance_lidar.git
    ```   
   
2. Build the workspace
    ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
    ```   

3. Launch the simulation
   ```bash
   ros2 launch obstacle_avoidance state_publisher.launch.py
   ```   

