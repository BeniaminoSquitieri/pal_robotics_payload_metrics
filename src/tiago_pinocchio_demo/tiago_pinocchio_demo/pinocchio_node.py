#!/usr/bin/env python3

import rclpy                                # Core library for ROS 2 Python client
from rclpy.node import Node                 # Base class for creating ROS 2 nodes
from std_msgs.msg import String            # Standard message type (string)
from rclpy.qos import QoSProfile, DurabilityPolicy  # Tools for configuring Quality of Service

import pinocchio as pin                    # Pinocchio: a library for robot kinematics/dynamics
import numpy as np                         # NumPy for numerical computations
import tempfile                            # Used to create temporary files
import os                                  # Used to remove the temporary file after usage

class PinocchioDemoNode(Node):
    """
    This node demonstrates four main steps using Pinocchio:
    1. Build a model from a URDF provided as a string.
    2. Perform forward kinematics.
    3. Compute a Jacobian for a specific frame.
    4. Check the distance (collision check) between two links.

    It subscribes to '/robot_description' and, once it receives the URDF,
    it executes these steps.
    """

    def __init__(self):
        # Initialize the ROS 2 node with the name 'pinocchio_demo_node'
        super().__init__('pinocchio_demo_node')
        
        # We create a QoS profile allowing us to receive latched messages (TRANSIENT_LOCAL).
        # This ensures we get the URDF even if it was published before our subscriber started.
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.get_logger().info("Initializing subscription to '/robot_description' with TRANSIENT_LOCAL QoS...")
        # Create a subscription to the '/robot_description' topic using our QoS
        self.subscription = self.create_subscription(
            String,
            '/robot_description',                 # Topic for the robot's URDF
            self.robot_description_callback,      # Function that will be called on every message
            qos
        )

    def robot_description_callback(self, msg):
        """
        Callback function executed when a new message is received on the '/robot_description' topic.
        We extract the URDF, build a Pinocchio model from it, perform forward kinematics,
        calculate a Jacobian, and check the distance between two links.
        """

        # STEP 1: Build a Pinocchio model from the URDF
        self.get_logger().info("Step 1: Received URDF. Creating Pinocchio Model...")
        urdf_string = msg.data  # The URDF in string form

        # We must write the URDF to a temporary file because buildModelFromUrdf requires a file path
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp_file:
                tmp_file.write(urdf_string)             # Write the URDF to the temp file
                tmp_filename = tmp_file.name            # Store the file's path
            self.get_logger().info(f"Temporary URDF file created at: {tmp_filename}")
        except Exception as e:
            self.get_logger().error(f"Error writing temporary URDF file: {e}")
            return

        try:
            # Build the model from the file. This returns a single Model object.
            # We use pin.JointModelFreeFlyer() so the base is free-floating in space.
            model = pin.buildModelFromUrdf(tmp_filename, pin.JointModelFreeFlyer())
            data = model.createData()  # Create a Data structure for computations
            self.get_logger().info("Step 1: Pinocchio Model created successfully.")
        except Exception as e:
            self.get_logger().error(f"Step 1 ERROR: Could not create Pinocchio model: {e}")
            # If an exception occurs, remove the temporary file and stop
            os.remove(tmp_filename)
            return

        # We can now remove the temporary file after building the model
        os.remove(tmp_filename)

        # STEP 2: Perform forward kinematics
        self.get_logger().info("Step 2: Performing forward kinematics using neutral configuration...")
        # q is the vector of joint positions. We use pin.neutral to get a default neutral configuration
        q = pin.neutral(model)

        try:
            # forwardKinematics calculates the positions of all joints
            pin.forwardKinematics(model, data, q)
            # updateFramePlacements updates the 'oMf' array storing frame placements in the world
            pin.updateFramePlacements(model, data)
            self.get_logger().info("Step 2: Forward kinematics computed successfully.")
        except Exception as e:
            self.get_logger().error(f"Step 2 ERROR: Forward kinematics failed: {e}")
            return

        # We retrieve the transform of a particular frame, e.g. 'base_link'
        frame_name = 'base_link'
        self.get_logger().info(f"Step 2: Retrieving transform for frame '{frame_name}'...")
        try:
            frame_id = model.getFrameId(frame_name)         # Lookup the frame index
            transform = data.oMf[frame_id]                  # The transform of that frame in the world
            self.get_logger().info(f"Step 2: Transform for '{frame_name}':\n{transform}")
        except Exception as e:
            self.get_logger().error(f"Step 2 ERROR: Unable to get transform for frame '{frame_name}': {e}")

        # STEP 3: Compute the Jacobian for a specific frame
        frame_jacobian_name = 'arm_7_link'
        self.get_logger().info(f"Step 3: Calculating Jacobian for frame '{frame_jacobian_name}' (LOCAL)...")
        try:
            frame_id_jac = model.getFrameId(frame_jacobian_name)
            # computeFrameJacobian returns a matrix representing the partial derivatives
            # of the frame's motion with respect to joint velocities
            jacobian = pin.computeFrameJacobian(model, data, q, frame_id_jac, pin.ReferenceFrame.LOCAL)
            self.get_logger().info(f"Step 3: Jacobian for '{frame_jacobian_name}':\n{jacobian}")
        except Exception as e:
            self.get_logger().error(f"Step 3 ERROR: Jacobian calculation failed for '{frame_jacobian_name}': {e}")

        # STEP 4: Check collisions by measuring the distance between two links (arm_1_link vs. arm_4_link)
        link_a = 'arm_1_link'
        link_b = 'arm_4_link'
        self.get_logger().info(f"Step 4: Checking collision between '{link_a}' and '{link_b}' by computing the distance...")

        try:
            # Retrieve the frames for link_a and link_b
            link_a_id = model.getFrameId(link_a)
            link_b_id = model.getFrameId(link_b)
            pos_a = data.oMf[link_a_id].translation  # Position of link_a in 3D
            pos_b = data.oMf[link_b_id].translation  # Position of link_b in 3D

            # Compute Euclidean distance
            distance = np.linalg.norm(pos_a - pos_b)
            self.get_logger().info(f"Step 4: Distance between '{link_a}' and '{link_b}': {distance:.3f} m")

            # If distance is below a certain threshold, we consider it a collision alert
            if distance < 0.05:
                self.get_logger().warn(f"Step 4: Warning! Possible collision detected between '{link_a}' and '{link_b}'.")
            else:
                self.get_logger().info(f"Step 4: No collision detected between '{link_a}' and '{link_b}'.")
        except Exception as e:
            self.get_logger().error(f"Step 4 ERROR: Collision check failed between '{link_a}' and '{link_b}': {e}")

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    # Create the node
    node = PinocchioDemoNode()
    # Spin the node so it can process callbacks
    rclpy.spin(node)
    # Clean up when shutting down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
