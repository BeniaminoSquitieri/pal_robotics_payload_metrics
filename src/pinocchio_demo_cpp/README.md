# Pinocchio Demo Node (C++ Version)

This package contains a ROS 2 node written in C++ that demonstrates how to use the Pinocchio library for robot kinematics and dynamics. The node subscribes to the `/robot_description` topic to receive a URDF, builds a Pinocchio model, performs forward kinematics, computes a Jacobian for a specified frame, and checks for collisions between two specified links. To avoid duplicate processing, the URDF is processed only once. Optional output saving is also available.

## Overview

The Pinocchio Demo Node performs the following steps:

### URDF Handling:
- Subscribes to the `/robot_description` topic using a `transient_local` QoS, ensuring that late-joining subscribers receive the last published message.
- Writes the URDF received to a temporary file.

### Model Creation:
- Builds a Pinocchio model from the temporary URDF file using a free-flyer joint for the base.

### Kinematics and Jacobian:
- Performs forward kinematics using a neutral joint configuration and times the computation.
- Retrieves and logs the transform for a configurable base link.
- Computes the Jacobian for a configurable frame and prints it in a nicely formatted manner.
- Optionally, additional debugging information can be printed in debug mode.

### Collision Checking:
- Checks for potential collisions by measuring the Euclidean distance between two configurable links.

### Optional Output Saving:
- If enabled, the node saves the computed results (base transform, Jacobian, collision distance) to a text file.

## Prerequisites

### ROS 2:
Install a compatible ROS 2 distribution (e.g., Humble, Foxy, Galactic). See the ROS 2 installation guide for instructions.

### Pinocchio Library:
Ensure that Pinocchio is installed on your system. You can install it from source or via package manager if available for your distribution. For more details, visit the [Pinocchio GitHub repository](https://github.com/stack-of-tasks/pinocchio).

### C++ Compiler:
A C++14 (or later) compiler is required.

## Building the Node

### Clone the Repository:
```bash
git clone <repository_url>
cd <repository_directory>
```

### Add to Your ROS 2 Workspace:
Make sure the package is inside your ROS 2 workspace (e.g., `src/` folder).

### Build with Colcon:
From the root of your workspace, run:
```bash
colcon build --packages-select <package_name>
```
Replace `<package_name>` with the actual name of your package.

### Source the Workspace:
After building, source the workspace:
```bash
source install/setup.bash
```

## Running the Node

Run the node using the `ros2 run` command:
```bash
ros2 run pinocchio_demo pinocchio_demo_node 
```
Replace `<package_name>` with your package's name.

## Parameter Overrides

You can override the default parameters at runtime. For example:
```bash
ros2 run pinocchio_demo pinocchio_demo_node --ros-args \
  -p base_link_name:=base_link \
  -p jacobian_frame_name:=arm_7_link \
  -p collision_link_a:=arm_1_link \
  -p collision_link_b:=arm_4_link \
  -p collision_threshold:=0.05 \
  -p debug_mode:=true \
  -p save_output:=true
```

- `base_link_name`: Name of the frame for which the transform is retrieved.
- `jacobian_frame_name`: Name of the frame for which the Jacobian is computed.
- `collision_link_a` and `collision_link_b`: Names of the links between which the collision check is performed.
- `collision_threshold`: Distance threshold below which a collision is flagged.
- `debug_mode`: Enables additional detailed logging.
- `save_output`: If true, computed results will be saved to `pinocchio_results.txt`.

## Detailed Code Explanation

### URDF Processing:
The node subscribes to `/robot_description` and writes the URDF to a temporary file. A flag (`urdf_processed_`) ensures that the URDF is processed only once, preventing duplicate output.

### Model Building and Kinematics:
The node uses Pinocchio to build a free-floating model from the URDF, then computes forward kinematics on a neutral configuration. The execution time is measured using `std::chrono` and logged.

### Jacobian Computation:
After computing joint and frame placements, the node computes the local Jacobian for a specified frame. 

### Collision Check:
The node computes the Euclidean distance between two specified links and logs whether a potential collision is detected based on the defined threshold.

### Output Saving:
If enabled, the node saves the computed results (transform, Jacobian, collision distance) to a file named `pinocchio_results.txt`.

## Future Improvements

### Dynamic Updates:
Integrate dynamic updates using live joint states for real-time computation.

### Advanced Collision Detection:
Enhance collision detection using more sophisticated algorithms.
