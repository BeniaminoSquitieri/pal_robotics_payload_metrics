## Pinocchio Demo Node for ROS 2

This package contains a ROS 2 node that demonstrates the integration of the Pinocchio library for robotic computations. The node subscribes to the `/robot_description` topic to receive a robot URDF, builds a Pinocchio model, performs forward kinematics, computes a Jacobian, checks collisions, and optionally publishes markers for visualization in RViz.

### Overview

The Pinocchio Demo Node performs the following steps:

**URDF Handling:**
- Subscribes to the `/robot_description` topic.
- Saves the received URDF to a temporary file.

**Model Creation:**
- Uses the URDF file to build a Pinocchio model with a free-flyer base.

**Kinematics and Dynamics Computations:**
- Performs forward kinematics using the neutral configuration.
- Computes the Jacobian for a specified frame.
- Checks the distance between two designated links to determine potential collisions.

**Visualization (Optional):**
- Publishes markers to the `visualization_marker` topic for RViz display if enabled.

### Prerequisites

**ROS 2:**
- Follow the official ROS 2 installation guide for your distribution (e.g., Foxy, Galactic, Humble).

**Pinocchio Library:**
- Install the Pinocchio library via pip:
  ```bash
  pip install pin
  ```
- Alternatively, refer to the Pinocchio GitHub repository for building from source.

**Additional Dependencies:**
- Ensure you have the following ROS 2 packages installed:
  - `std_msgs`
  - `visualization_msgs` (if marker visualization is used)

### Installation

**Clone the Repository:**
```bash
git clone <repository_url>
cd <repository_directory>
```

**Build the Package:**
If you are using a ROS 2 workspace, add the repository to your workspace and build it:
```bash
colcon build --packages-select <package_name>
```

**Source the ROS 2 Environment:**
Open a terminal and source your ROS 2 setup file:
```bash
source /opt/ros/<your_ros2_distro>/setup.bash
```
Then, source your workspace overlay:
```bash
source install/setup.bash
```

### Running the Node

**Step 1: Launch the Node**
You can run the node directly using:
```bash
ros2 run <package_name> pinocchio_demo_node.py
ros2 run <package_name> pinocchio_demo_node.py --ros-args -p base_link_name:=new_base_link -p collision_threshold:=0.1 -p publish_markers:=True

```
Alternatively, if a launch file is provided:
```bash
ros2 launch <package_name> demo_launch.py
```
Replace `<package_name>` with the actual name of your ROS 2 package.

**Step 2: Publish the /robot_description Topic**
Make sure a publisher is providing the robot URDF. You can manually publish a test URDF with:
```bash
ros2 topic pub /robot_description std_msgs/msg/String "data: '<robot_urdf_content>'"
```
Replace `<robot_urdf_content>` with your actual URDF string.

### Configurable Parameters

The node provides several ROS 2 parameters that can be adjusted at runtime:

- `base_link_name` (default: `base_link`):
  - The frame for which the transform is retrieved.

- `jacobian_frame_name` (default: `arm_7_link`):
  - The frame for which the Jacobian is computed.

- `collision_link_a` (default: `arm_1_link`):
  - First link to check for collisions.

- `collision_link_b` (default: `arm_4_link`):
  - Second link to check for collisions.

- `collision_threshold` (default: `0.05`):
  - Distance threshold (in meters) below which a collision warning is triggered.

- `publish_markers` (default: `False`):
  - Enable marker publication for RViz visualization.

Parameters can be set via command-line arguments or within a launch file.

### Future Improvements

- **Real-time Integration:**
  - Integrate with live joint state updates for dynamic model updates.

- **Advanced Visualization:**
  - Extend RViz visualization to display more detailed robot states and dynamic changes.

- **Improved Logging:**
  - Refine logging capabilities and integrate performance profiling tools.

