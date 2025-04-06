#!/usr/bin/env python3
# This shebang allows the script to be run as an executable

import rclpy                         # ROS 2 Python client library
from rclpy.node import Node          # Base class for creating ROS 2 nodes
from std_msgs.msg import String      # Standard ROS message type for strings
from rclpy.qos import QoSProfile, DurabilityPolicy  # Tools for setting up Quality of Service (QoS)

import pinocchio as pin              # Pinocchio library for robot kinematics and dynamics
import numpy as np                   # NumPy library for numerical computations
import tempfile                      # Module to create temporary files
import os                            # Module for operating system interface (e.g., file removal)
import traceback                     # Module to print stack traces of exceptions
import time                          # Module for timing measurements

# Define a ROS 2 node class that integrates Pinocchio for robot computations
class PinocchioDemoNode(Node):
    def __init__(self):
        # Initialize the node with the name 'pinocchio_demo_node'
        super().__init__('pinocchio_demo_node')
        
        # Flag to ensure that the URDF is processed only once.
        self.urdf_processed = False

        # Declare ROS 2 parameters to allow runtime configuration.
        # These parameters can be overridden when launching the node.
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('jacobian_frame_name', 'arm_7_link')
        self.declare_parameter('collision_link_a', 'arm_1_link')
        self.declare_parameter('collision_link_b', 'arm_4_link')
        self.declare_parameter('collision_threshold', 0.05)
        # debug_mode for detailed logging (default: False)
        self.declare_parameter('debug_mode', False)
        # save_output to save the computation results to a file (default: False)
        self.declare_parameter('save_output', False)
        
        # Retrieve the parameter values.
        # These values are used later in the node for calculations and configuration.
        self.base_link_name = self.get_parameter('base_link_name').get_parameter_value().string_value
        self.jacobian_frame_name = self.get_parameter('jacobian_frame_name').get_parameter_value().string_value
        self.collision_link_a = self.get_parameter('collision_link_a').get_parameter_value().string_value
        self.collision_link_b = self.get_parameter('collision_link_b').get_parameter_value().string_value
        self.collision_threshold = self.get_parameter('collision_threshold').get_parameter_value().double_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.save_output = self.get_parameter('save_output').get_parameter_value().bool_value

        # If debug mode is enabled, set the logger level to DEBUG to see detailed log messages.
        if self.debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Set up the QoS (Quality of Service) profile to receive latched messages.
        # TRANSIENT_LOCAL durability ensures that late-joining subscribers receive the last published message.
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Log the subscription action
        self.get_logger().info("Subscribing to '/robot_description' with TRANSIENT_LOCAL QoS")
        # Create a subscription to the '/robot_description' topic expecting messages of type String.
        # When a message is received, the robot_description_callback method is called.
        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            qos
        )
    
    def write_urdf_to_temp_file(self, urdf_string):
        """
        Writes the given URDF string to a temporary file and returns the file path.
        This is necessary because the Pinocchio model builder requires a file path.
        """
        try:
            # Create a temporary file in write mode with a '.urdf' suffix.
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp_file:
                tmp_file.write(urdf_string)  # Write the URDF content into the file
                tmp_filename = tmp_file.name  # Save the temporary file name
            self.get_logger().info(f"Temporary URDF file created: {tmp_filename}")
            return tmp_filename
        except Exception as e:
            # Log an error if writing the temporary file fails.
            self.get_logger().error(f"Error writing temporary URDF file: {e}")
            return None

    def build_pinocchio_model(self, urdf_filename):
        """
        Constructs the Pinocchio model from the URDF file.
        Returns both the model and the data structure needed for computations.
        """
        try:
            # Build the model from the URDF file. The JointModelFreeFlyer() allows the robot base to be free-floating.
            model = pin.buildModelFromUrdf(urdf_filename, pin.JointModelFreeFlyer())
            # Create the data structure associated with the model to hold computation results.
            data = model.createData()
            self.get_logger().info("Pinocchio model created successfully.")
            return model, data
        except Exception as e:
            # Log error and stack trace if model creation fails.
            self.get_logger().error(f"Error creating Pinocchio model: {e}")
            self.get_logger().debug(traceback.format_exc())
            return None, None

    def perform_forward_kinematics(self, model, data, q):
        """
        Performs forward kinematics using the given joint configuration 'q'.
        The forward kinematics computation updates the positions of each joint and frame.
        Also measures and logs the execution time.
        """
        try:
            start_time = time.perf_counter()  # Record start time
            # Compute forward kinematics, updating joint positions based on 'q'
            pin.forwardKinematics(model, data, q)
            # Update the placements of the frames in the world coordinate system.
            pin.updateFramePlacements(model, data)
            elapsed = time.perf_counter() - start_time  # Compute elapsed time
            self.get_logger().info(f"Forward kinematics computed successfully in {elapsed:.6f} seconds.")
        except Exception as e:
            # Log error details and propagate the exception if forward kinematics fails.
            self.get_logger().error(f"Error in forward kinematics: {e}")
            self.get_logger().debug(traceback.format_exc())
            raise

    def get_transform(self, model, data, frame_name):
        """
        Retrieves the transform (position and orientation) of the specified frame.
        The transform is obtained from the 'data.oMf' structure which holds frame placements.
        """
        try:
            # Get the frame ID using the frame name.
            frame_id = model.getFrameId(frame_name)
            # Retrieve the transform corresponding to that frame.
            transform = data.oMf[frame_id]
            self.get_logger().info(f"Transform for '{frame_name}':\n{transform}")
            return transform
        except Exception as e:
            # Log an error if the frame transform cannot be retrieved.
            self.get_logger().error(f"Unable to get transform for '{frame_name}': {e}")
            self.get_logger().debug(traceback.format_exc())
            return None

    def compute_jacobian(self, model, data, q, frame_name):
        """
        Computes the Jacobian for the given frame.
        The Jacobian is calculated in the LOCAL reference frame, which shows how joint velocities affect the frame's motion.
        Measures and logs the computation time and, if debug mode is enabled, logs the full Jacobian.
        """
        try:
            start_time = time.perf_counter()  # Start timing
            # Get the frame ID for the specified frame name.
            frame_id = model.getFrameId(frame_name)
            # Compute the frame Jacobian in the LOCAL reference frame.
            jacobian = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)
            elapsed = time.perf_counter() - start_time  # Compute elapsed time
            self.get_logger().info(f"Jacobian for '{frame_name}' computed in {elapsed:.6f} seconds. Shape: {jacobian.shape}")
            if self.debug_mode:
                self.get_logger().debug(f"Jacobian for '{frame_name}':\n{jacobian}")
            return jacobian
        except Exception as e:
            # Log an error if Jacobian computation fails.
            self.get_logger().error(f"Error computing Jacobian for '{frame_name}': {e}")
            self.get_logger().debug(traceback.format_exc())
            return None

    def check_collision(self, model, data, link_a, link_b):
        """
        Checks for potential collisions between two links by measuring the Euclidean distance between their frames.
        If the distance is below the collision threshold, a collision warning is logged.
        Also measures and logs the execution time of the collision check.
        """
        try:
            start_time = time.perf_counter()  # Start timing
            # Retrieve the frame IDs for the two links.
            link_a_id = model.getFrameId(link_a)
            link_b_id = model.getFrameId(link_b)
            # Get the 3D positions (translations) of both links.
            pos_a = data.oMf[link_a_id].translation
            pos_b = data.oMf[link_b_id].translation
            # Calculate the Euclidean distance between the two positions.
            distance = np.linalg.norm(pos_a - pos_b)
            elapsed = time.perf_counter() - start_time  # Compute elapsed time
            self.get_logger().info(f"Distance between '{link_a}' and '{link_b}': {distance:.3f} m (computed in {elapsed:.6f} seconds)")
            if distance < self.collision_threshold:
                self.get_logger().warn(f"Warning: potential collision between '{link_a}' and '{link_b}'.")
            else:
                self.get_logger().info(f"No collision detected between '{link_a}' and '{link_b}'.")
            return distance
        except Exception as e:
            self.get_logger().error(f"Error checking collision between '{link_a}' and '{link_b}': {e}")
            self.get_logger().debug(traceback.format_exc())
            return None

    def robot_description_callback(self, msg):
        """
        Callback function triggered when a new robot description (URDF) is received.
        This function performs the following steps:
          1. Writes the URDF to a temporary file.
          2. Builds the Pinocchio model using the URDF.
          3. Performs forward kinematics using the robot's neutral configuration.
          4. Retrieves the transform for the base link.
          5. Computes the Jacobian for the specified frame.
          6. Checks for collisions between two specified links.
          7. Optionally saves the computed results to a file.
          
        Note: To avoid processing duplicate messages, the URDF is processed only once.
        """
        # If we have already processed a URDF, do nothing.
        if self.urdf_processed:
            return

        self.get_logger().info("Received URDF on '/robot_description'.")
        
        # STEP 1: Write the URDF string to a temporary file.
        urdf_filename = self.write_urdf_to_temp_file(msg.data)
        if not urdf_filename:
            return  # Exit callback if file writing fails
        
        # STEP 2: Build the Pinocchio model from the temporary URDF file.
        model, data = self.build_pinocchio_model(urdf_filename)
        os.remove(urdf_filename)  # Clean up the temporary file
        if model is None or data is None:
            return  # Exit callback if model creation fails
        
        # STEP 3: Perform forward kinematics using the neutral joint configuration.
        q = pin.neutral(model)  # Obtain the default joint configuration (neutral pose)
        try:
            self.perform_forward_kinematics(model, data, q)
        except Exception:
            return  # Exit callback if forward kinematics fails
        
        # STEP 4: Retrieve the transform for the base link.
        base_transform = self.get_transform(model, data, self.base_link_name)
        
        # STEP 5: Compute the Jacobian for the specified frame.
        jacobian = self.compute_jacobian(model, data, q, self.jacobian_frame_name)
        
        # STEP 6: Check for potential collisions between two specified links.
        collision_distance = self.check_collision(model, data, self.collision_link_a, self.collision_link_b)
        
        # STEP 7: Optionally save the computed results to a file.
        if self.save_output:
            output_filename = "pinocchio_results.txt"
            try:
                with open(output_filename, "w") as file:
                    file.write("=== Pinocchio Computation Results ===\n")
                    file.write(f"Base Transform for '{self.base_link_name}':\n{base_transform}\n\n")
                    file.write(f"Jacobian for '{self.jacobian_frame_name}':\n{jacobian}\n\n")
                    file.write(f"Collision Distance between '{self.collision_link_a}' and '{self.collision_link_b}': {collision_distance:.3f} m\n")
                self.get_logger().info(f"Computation results saved to {output_filename}")
            except Exception as e:
                self.get_logger().error(f"Error saving computation results: {e}")
                self.get_logger().debug(traceback.format_exc())
        
        # Set the flag so that further URDF messages are ignored.
        self.urdf_processed = True

# Main function to initialize the node and start the ROS event loop.
def main(args=None):
    rclpy.init(args=args)              # Initialize the ROS 2 Python client library
    node = PinocchioDemoNode()         # Create an instance of the PinocchioDemoNode
    try:
        rclpy.spin(node)               # Keep the node running, processing callbacks until shutdown
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()            # Clean up the node
        rclpy.shutdown()               # Shutdown the ROS client library

# Standard Python entry point
if __name__ == '__main__':
    main()
