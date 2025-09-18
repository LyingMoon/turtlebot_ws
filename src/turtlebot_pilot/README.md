# TurtleBot Pilot 🤖

**Autonomous Mobile TurtleBot | ROS2, Linux, Python, Path Planning & Control, Computer Vision**

A comprehensive autonomous navigation system for TurtleBot3 featuring SLAM, path planning, and computer vision capabilities developed at Stanford University (2024 Autumn).

## 🚀 Features

- **Autonomous Navigation**: A* pathfinding with frontier exploration
- **SLAM Integration**: LIDAR-based Simultaneous Localization and Mapping
- **Advanced Control**: Gain-scheduled LQR control for trajectory tracking
- **Computer Vision**: PyTorch-based object recognition using Inception-v3
- **Real-time Processing**: Dynamic obstacle avoidance and pose estimation via ICP

## 📁 Project Structure

```
TurtleBotPilot/
├── src/
│   ├── navigation/          # Path planning & exploration
│   │   ├── navigator.py     # A* pathfinding implementation
│   │   ├── explorer.py      # Frontier exploration algorithm
│   │   ├── explorer_stop.py # Exploration termination logic
│   │   └── frontier_explore.py # Frontier detection utilities
│   ├── control/             # Control systems
│   │   └── heading_controller.py # LQR-based heading control
│   ├── perception/          # Computer vision & SLAM
│   │   ├── perception_controller.py # Object recognition pipeline
│   │   ├── icp_node.py      # Iterative Closest Point localization
│   │   └── icp_utils.py     # ICP utility functions
│   └── utils/               # Utilities and visualization
│       └── p3_plot.py       # Plotting and data visualization
├── launch/                  # ROS2 launch files
├── config/                  # Configuration files (rviz, etc.)
├── data/                    # Datasets and recorded bags
├── CMakeLists.txt          # Build configuration
└── package.xml             # Package metadata
```

## 🛠️ Dependencies

- ROS2 (Humble/Iron)
- Python 3.8+
- PyTorch (for computer vision)
- asl_tb3_lib
- asl_tb3_msgs

## 🚦 Quick Start

1. **Build the package:**
   ```bash
   colcon build --packages-select turtlebot_pilot
   source install/setup.bash
   ```

2. **Launch autonomous navigation:**
   ```bash
   ros2 launch turtlebot_pilot navigator.launch.py
   ```

3. **Launch exploration mode:**
   ```bash
   ros2 launch turtlebot_pilot explorer.launch.py
   ```

## 🧠 Key Algorithms

- **A* Path Planning**: Optimal pathfinding with heuristic search
- **Frontier Exploration**: Autonomous mapping via unexplored region detection
- **LQR Control**: Linear Quadratic Regulator for trajectory tracking
- **ICP Localization**: Iterative Closest Point for pose estimation
- **Inception-v3**: Deep learning for object recognition

## 📊 Performance

- Real-time obstacle avoidance with LIDAR integration
- Improved trajectory tracking accuracy via gain-scheduled control
- Robust pose estimation using ICP refinement
- Sliding window detection for enhanced object recognition

## 🎓 Academic Context

This project was developed as part of autonomous systems coursework at Stanford University, demonstrating integration of:
- Classical robotics algorithms (A*, LQR)
- Modern SLAM techniques
- Deep learning for perception
- Real-time system integration

---

*Developed with ROS2, Python, and a passion for autonomous robotics* 🚀