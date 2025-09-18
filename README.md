# TurtleBot Autonomous Navigation Workspace

**Autonomous Mobile TurtleBot | ROS2, Linux, Python, Path Planning & Control, Computer Vision**

A comprehensive ROS2 workspace containing the TurtleBot3 autonomous navigation system featuring SLAM, path planning, and computer vision capabilities developed at Stanford University (2024 Autumn).

## 🚀 Features

- **Autonomous Navigation**: A* pathfinding with frontier exploration
- **SLAM Integration**: LIDAR-based Simultaneous Localization and Mapping
- **Advanced Control**: Gain-scheduled LQR control for trajectory tracking
- **Computer Vision**: PyTorch-based object recognition using Inception-v3
- **Real-time Processing**: Dynamic obstacle avoidance and pose estimation via ICP

## 📁 Workspace Structure

```
turtlebot_ws/
├── src/                     # Source packages
│   └── turtlebot_pilot/     # Main autonomous navigation package
│       ├── src/
│       │   ├── navigation/  # Path planning & exploration
│       │   ├── control/     # Control systems
│       │   ├── perception/  # Computer vision & SLAM
│       │   └── utils/       # Utilities and visualization
│       ├── launch/          # ROS2 launch files
│       ├── config/          # Configuration files
│       ├── data/            # Datasets and recorded bags
│       ├── CMakeLists.txt   # Build configuration
│       └── package.xml      # Package metadata
├── build/                   # Build artifacts
├── install/                 # Installed packages
└── log/                     # Build logs
```

## 🛠️ Dependencies

- ROS2 (Humble/Iron recommended)
- Python 3.8+
- PyTorch (for computer vision)
- Open3D (for point cloud processing)
- asl_tb3_lib
- asl_tb3_msgs

## 🚦 Quick Start

### 1. Setup Workspace
```bash
cd turtlebot_ws
colcon build
source install/setup.bash
```

### 2. Launch Autonomous Navigation
```bash
# Full autonomous navigation with A* pathfinding
ros2 launch turtlebot_pilot navigator.launch.py

# Frontier-based exploration for SLAM
ros2 launch turtlebot_pilot explorer.launch.py

# Exploration with automatic termination
ros2 launch turtlebot_pilot explorer_stop.launch.py
```

### 3. Individual System Components
```bash
# ICP localization only
ros2 launch turtlebot_pilot icp.launch.py

# Heading control only
ros2 launch turtlebot_pilot heading_control.launch.py

# Computer vision perception
ros2 launch turtlebot_pilot perception_control.launch.py
```

### 4. Visualization and Analysis
```bash
# Real-time trajectory plotting
ros2 run turtlebot_pilot p3_plot.py

# Individual nodes
ros2 run turtlebot_pilot navigator.py
ros2 run turtlebot_pilot explorer.py
ros2 run turtlebot_pilot heading_controller.py
```

## 🔧 Build Commands

```bash
# Build specific package
colcon build --packages-select turtlebot_pilot

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build
rm -rf build install log
colcon build
```

## 🧠 Key Algorithms

- **A* Path Planning**: Optimal pathfinding with heuristic search
- **Frontier Exploration**: Autonomous mapping via unexplored region detection
- **LQR Control**: Linear Quadratic Regulator for trajectory tracking
- **ICP Localization**: Iterative Closest Point for pose estimation
- **Inception-v3**: Deep learning for object recognition

## 📊 Performance Metrics

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

## 🔄 Development Workflow

```bash
# After making changes to source code
cd turtlebot_ws
colcon build --packages-select turtlebot_pilot
source install/setup.bash

# Run tests (when available)
colcon test --packages-select turtlebot_pilot
```

## 📝 Package Information

- **Package Name**: `turtlebot_pilot`
- **Version**: 1.0.0
- **License**: MIT
- **Maintainer**: Stanford Robotics Team

---

*Developed with ROS2, Python, and a passion for autonomous robotics* 🚀