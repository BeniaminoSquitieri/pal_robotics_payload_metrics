# Pinocchio Demo Node for ROS 2 (Python and C++ Versions)

This repository contains ROS 2 nodes (Python and C++) that demonstrate the integration of the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library for robotic kinematics and dynamics computations. These nodes subscribe to the `/robot_description` topic to receive a URDF, build a Pinocchio model, compute forward kinematics and Jacobians, perform collision checks, and optionally publish visualization markers or save output to a file.

## Overview

Both versions of the node perform the following steps:

### URDF Handling:
- Subscribes to the `/robot_description` topic to receive a URDF.
- Uses a `transient_local` QoS to ensure URDF delivery to late subscribers (C++).
- Writes the URDF to a temporary file.

### Model Creation:
- Builds a Pinocchio model with a free-flyer base using the URDF file.

### Kinematics and Dynamics:
- Performs forward kinematics with a neutral joint configuration.
- Computes the Jacobian for a configurable frame.
- Retrieves and logs the base link transform.
- Checks the Euclidean distance between two configurable links to assess potential collisions.

### Optional Features:
- Publishes visualization markers to RViz (# Pinocchio Demo Node for ROS 2 (Python and C++ Versions)

This repository contains ROS 2 nodes (Python and C++) that demonstrate the integration of the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library for robotic kinematics and dynamics computations. These nodes subscribe to the `/robot_description` topic to receive a URDF, build a Pinocchio model, compute forward kinematics and Jacobians, perform collision checks, and optionally publish visualization markers or save output to a file.

## Overview

Both versions of the node perform the following steps:

### URDF Handling:
- Subscribes to the `/robot_description` topic to receive a URDF.
- Uses a `transient_local` QoS to ensure URDF delivery to late subscribers (C++).
- Writes the URDF to a temporary file.

### Model Creation:
- Builds a Pinocchio model with a free-flyer base using the URDF file.

### Kinematics and Dynamics:
- Performs forward kinematics with a neutral joint configuration.
- Computes the Jacobian for a configurable frame.
- Retrieves and logs the base link transform.
- Checks the Euclidean distance between two configurable links to assess potential collisions.

### Optional Features:
- Publishes visualization markers to RViz (Python).
- Saves results (transform, Jacobian, collision distance) to a file (C++).
- Supports debug mode with detailed logging (Python

## Prerequisites

### ROS 2:
Install a supported ROS 2 distribution . Follow the official installation guide for your platform.

### Pinocchio:
Install the Pinocchio library:
```bash
pip install pin  # For Python
```
Or build from source via the [Pinocchio GitHub repository](https://github.com/stack-of-tasks/pinocchio).

### Additional Dependencies:
- `std_msgs`
- `visualization_msgs` (for RViz support)
- A C++14-compatible compiler (for C++ version)

## Installation

### Clone the Repository:
```bash
git clone <repository_url>
cd <repository_directory>
```

### Add to a ROS 2 Workspace and Build:
```bash
colcon build --packages-select <package_name>
source install/setup.bash
```
Replace `<package_name>` with your actual package name.

## Running the Node

### Python Version:
```bash
ros2 run <package_name> pinocchio_demo_node.py
```
With parameter overrides:
```bash
ros2 run <package_name> pinocchio_demo_node.py --ros-args \
  -p base_link_name:=new_base_link \
  -p collision_threshold:=0.1 \
  -p publish_markers:=True
```
### C++ Version:
```bash
ros2 run <package_name> pinocchio_demo_node
```
With parameter overrides:
```bash
ros2 run <package_name> pinocchio_demo_node --ros-args \
  -p base_link_name:=base_link \
  -p jacobian_frame_name:=arm_7_link \
  -p collision_link_a:=arm_1_link \
  -p collision_link_b:=arm_4_link \
  -p collision_threshold:=0.05 \
  -p debug_mode:=true \
  -p save_output:=true
```

### URDF Publishing:
Ensure the `/robot_description` topic is published, e.g.:
```bash
ros2 topic pub /robot_description std_msgs/msg/String "data: '<robot_urdf_content>'"
```
Replace `<robot_urdf_content>` with your actual URDF string.

## Parameters

| Parameter | Description | Default |
|----------|-------------|---------|
| `base_link_name` | Base frame for transform retrieval | `base_link` |
| `jacobian_frame_name` | Frame for Jacobian computation | `arm_7_link` |
| `collision_link_a` | First link for collision check | `arm_1_link` |
| `collision_link_b` | Second link for collision check | `arm_4_link` |
| `collision_threshold` | Collision distance threshold | `0.05` |
| `publish_markers` | Enable RViz markers (Python only) | `False` |
| `debug_mode` | Enable detailed logging (C++ only) | `False` |
| `save_output` | Save results to file (C++ only) | `False` |

## Output Saving (C++ Only)

If `save_output` is enabled, the node writes results to `pinocchio_results.txt`, including:
- Base transform
- Jacobian matrix
- Collision distance

## Future Improvements

- **Dynamic Updates:** Support live joint state updates for real-time computation.
- **Enhanced Visualization:** Extend RViz integration for richer robot state displays.
- **Improved Logging and Profiling:** Add performance metrics and structured logs.
