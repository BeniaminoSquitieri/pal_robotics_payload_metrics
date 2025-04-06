/*******************************************************
 * pinocchio_demo_node.cpp
 * 
 * A ROS 2 C++ node that:
 * 1) Subscribes to /robot_description
 * 2) Reads parameters for configuration (e.g., base link, Jacobian frame, collision links, thresholds)
 * 3) Builds a Pinocchio model (from a URDF file)
 * 4) Performs forward kinematics to compute frame transforms
 * 5) Computes a Jacobian for a specified frame
 * 6) Checks the distance between two links (collision check)
 * 7) Optionally saves the computed results to a file
 * 
 * This version processes the URDF only once to avoid duplicate outputs.
 * Detailed comments explain each step.
 *******************************************************/

 #include <rclcpp/rclcpp.hpp>
 #include <std_msgs/msg/string.hpp>
 
 // Pinocchio includes for URDF parsing and kinematics/dynamics computations
 #include <pinocchio/parsers/urdf.hpp>
 #include <pinocchio/algorithm/kinematics.hpp>
 #include <pinocchio/algorithm/jacobian.hpp>
 #include <pinocchio/algorithm/frames.hpp>
 #include <pinocchio/algorithm/joint-configuration.hpp>
 
 #include <fstream>
 #include <cstdio>       // for std::remove
 #include <string>
 #include <sstream>
 #include <iomanip>      // for std::setprecision
 #include <chrono>       // for high_resolution_clock timing measurements
 #include <exception>
 #include <iostream>
 
 class PinocchioDemoNode : public rclcpp::Node
 {
 public:
   // Constructor: initialize the node, declare and read parameters, and set up the subscription.
   PinocchioDemoNode()
   : Node("pinocchio_demo_node"),
     urdf_processed_(false)  // Initialize flag so that URDF is processed only once.
   {
     // Declare ROS 2 parameters for configuration with default values.
     // These parameters can be overridden at launch.
     this->declare_parameter("base_link_name", "base_link");
     this->declare_parameter("jacobian_frame_name", "arm_7_link");
     this->declare_parameter("collision_link_a", "arm_1_link");
     this->declare_parameter("collision_link_b", "arm_4_link");
     this->declare_parameter("collision_threshold", 0.05);
     this->declare_parameter("save_output", false);
 
     // Retrieve parameter values into member variables.
     this->get_parameter("base_link_name", base_link_name_);
     this->get_parameter("jacobian_frame_name", jacobian_frame_name_);
     this->get_parameter("collision_link_a", collision_link_a_);
     this->get_parameter("collision_link_b", collision_link_b_);
     this->get_parameter("collision_threshold", collision_threshold_);
     this->get_parameter("save_output", save_output_);
 
     // Create a QoS profile with transient_local durability.
     // This ensures that late-joining subscribers receive the last published message.
     rclcpp::QoS qos(rclcpp::KeepLast(10));
     qos.transient_local();
 
     RCLCPP_INFO(this->get_logger(),
       "[PinocchioDemoNode] Subscribing to /robot_description with TRANSIENT_LOCAL QoS.");
 
     // Create a subscription to the "/robot_description" topic.
     // The callback 'onURDFReceived' will process the URDF message.
     sub_ = this->create_subscription<std_msgs::msg::String>(
       "/robot_description",
       qos,
       std::bind(&PinocchioDemoNode::onURDFReceived, this, std::placeholders::_1)
     );
   }
 
 private:
   // Callback function triggered when a new URDF message is received.
   // This callback performs several steps:
   //  1) Writes the URDF to a temporary file.
   //  2) Builds the Pinocchio model from that file.
   //  3) Performs forward kinematics (timed).
   //  4) Retrieves and logs the transform for the base link.
   //  5) Computes and logs the Jacobian for the specified frame.
   //  6) Performs a collision check between two specified links.
   //  7) Optionally saves the results to an output file.
   // To avoid duplicate processing, the URDF is processed only once.
   void onURDFReceived(const std_msgs::msg::String::SharedPtr msg)
   {
     // Check if the URDF has already been processed.
     if (urdf_processed_) {
       return;
     }
 
     RCLCPP_INFO(this->get_logger(), "Received URDF on '/robot_description'.");
 
     // STEP 1: Write the URDF string to a temporary file.
     std::string urdf_string = msg->data;
     std::string temp_path = "/tmp/temp_robot.urdf";
 
     {
       std::ofstream out(temp_path);
       if (!out.is_open()) {
         RCLCPP_ERROR(this->get_logger(), "Cannot open %s for writing URDF!", temp_path.c_str());
         return;
       }
       out << urdf_string;
       out.close();
       RCLCPP_INFO(this->get_logger(), "Temporary URDF file created: %s", temp_path.c_str());
     }
 
     // STEP 2: Build the Pinocchio model from the URDF file.
     pinocchio::Model model;
     try {
       // Build the model using a free-flyer joint for the base.
       pinocchio::urdf::buildModel(temp_path, pinocchio::JointModelFreeFlyer(), model, false);
       RCLCPP_INFO(this->get_logger(), "Pinocchio Model created successfully. (nq=%d, nv=%d)",
                   model.nq, model.nv);
     }
     catch (const std::exception & e) {
       RCLCPP_ERROR(this->get_logger(), "Could not build model: %s", e.what());
       std::remove(temp_path.c_str());
       return;
     }
     // Remove the temporary file as it is no longer needed.
     std::remove(temp_path.c_str());
 
     // STEP 3: Perform forward kinematics using a neutral configuration.
     RCLCPP_INFO(this->get_logger(), "\n========== STEP 2: Forward Kinematics on a Neutral Configuration ==========");
 
     // Create a Pinocchio data structure to hold computation results.
     pinocchio::Data data(model);
     // Create a "neutral" joint configuration (all zeros).
     Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
 
     // Time the forward kinematics computation.
     auto fk_start = std::chrono::high_resolution_clock::now();
     try {
       // Compute joint placements.
       pinocchio::forwardKinematics(model, data, q);
       // Update the frame placements based on the joint configuration.
       pinocchio::framesForwardKinematics(model, data, q);
       auto fk_end = std::chrono::high_resolution_clock::now();
       std::chrono::duration<double> fk_duration = fk_end - fk_start;
       RCLCPP_INFO(this->get_logger(), "Forward kinematics completed successfully in %.6f seconds.", fk_duration.count());
     }
     catch (const std::exception & e) {
       RCLCPP_ERROR(this->get_logger(), "Forward kinematics failed: %s", e.what());
       return;
     }
 
     // STEP 4: Retrieve and log the transform for the base link.
     RCLCPP_INFO(this->get_logger(), "[FK] Retrieving transform for frame '%s'...", base_link_name_.c_str());
     try {
       pinocchio::FrameIndex fid = model.getFrameId(base_link_name_);
       auto & transform = data.oMf[fid];
       const auto & t = transform.translation();
       RCLCPP_INFO(this->get_logger(),
         "[FK] '%s' transform => translation [%.4f, %.4f, %.4f]",
         base_link_name_.c_str(), t[0], t[1], t[2]);
     }
     catch (const std::exception & e) {
       RCLCPP_ERROR(this->get_logger(), "Cannot get transform for '%s': %s", base_link_name_.c_str(), e.what());
     }
 
     // STEP 5: Calculate the Jacobian for the specified frame.
     RCLCPP_INFO(this->get_logger(), "\n========== STEP 3: Calculating Jacobian for '%s' ==========", jacobian_frame_name_.c_str());
     try {
       pinocchio::FrameIndex fid_jac = model.getFrameId(jacobian_frame_name_);
 
       // Compute the joint Jacobians for all joints.
       pinocchio::computeJointJacobians(model, data, q);
       // Update frame placements.
       pinocchio::framesForwardKinematics(model, data, q);
 
       // Compute the 6D local Jacobian for the specified frame.
       Eigen::MatrixXd J(6, model.nv);
       pinocchio::getFrameJacobian(model, data, fid_jac, pinocchio::LOCAL, J);
 
       RCLCPP_INFO(this->get_logger(),
         "Jacobian for '%s' has dimension [%ld x %ld]",
         jacobian_frame_name_.c_str(), J.rows(), J.cols());
 
       // Print the Jacobian matrix in a nicely formatted manner.
       for (int r = 0; r < J.rows(); ++r) {
         std::stringstream row;
         row << "  [ ";
         for (int c = 0; c < J.cols(); ++c) {
           row << std::fixed << std::setprecision(3) << J(r, c);
           if (c < J.cols() - 1) row << ", ";
         }
         row << " ]";
         RCLCPP_INFO(this->get_logger(), "%s", row.str().c_str());
       }
     }
     catch (const std::exception & e) {
       RCLCPP_ERROR(this->get_logger(), "Jacobian calculation failed for '%s': %s", jacobian_frame_name_.c_str(), e.what());
     }
 
     // STEP 6: Check for potential collisions by measuring the distance between two specified links.
     RCLCPP_INFO(this->get_logger(), "\n========== STEP 4: Checking distance between two frames (collision check) ==========");
     try {
       pinocchio::FrameIndex fidA = model.getFrameId(collision_link_a_);
       pinocchio::FrameIndex fidB = model.getFrameId(collision_link_b_);
 
       const auto & posA = data.oMf[fidA].translation();
       const auto & posB = data.oMf[fidB].translation();
 
       double dist = (posA - posB).norm();
       RCLCPP_INFO(this->get_logger(),
         "[Collision] Distance between '%s' and '%s' => %.4f m",
         collision_link_a_.c_str(), collision_link_b_.c_str(), dist);
 
       if (dist < collision_threshold_) {
         RCLCPP_WARN(this->get_logger(),
           "[Collision] Potential collision! Distance < %.2f m between '%s' and '%s'.",
           collision_threshold_, collision_link_a_.c_str(), collision_link_b_.c_str());
       } else {
         RCLCPP_INFO(this->get_logger(),
           "[Collision] No collision detected between '%s' and '%s'.",
           collision_link_a_.c_str(), collision_link_b_.c_str());
       }
     }
     catch (const std::exception & e) {
       RCLCPP_ERROR(this->get_logger(), "Collision check failed between '%s' and '%s': %s",
                    collision_link_a_.c_str(), collision_link_b_.c_str(), e.what());
     }
 
     // STEP 7: Optionally save the computed results to an output file.
     if (save_output_) {
       std::string output_filename = "pinocchio_results.txt";
       try {
         std::ofstream file(output_filename);
         if (!file.is_open()) {
           RCLCPP_ERROR(this->get_logger(), "Could not open file %s to save results!", output_filename.c_str());
         } else {
           file << "=== Pinocchio Computation Results ===\n";
           // Save the base transform.
           try {
             pinocchio::FrameIndex fid = model.getFrameId(base_link_name_);
             auto & transform = data.oMf[fid];
             file << "Base Transform for '" << base_link_name_ << "':\n" << transform << "\n\n";
           } catch (...) {
             file << "Base Transform for '" << base_link_name_ << "': not available\n\n";
           }
           // Save the Jacobian.
           try {
             pinocchio::FrameIndex fid_jac = model.getFrameId(jacobian_frame_name_);
             Eigen::MatrixXd J(6, model.nv);
             pinocchio::getFrameJacobian(model, data, fid_jac, pinocchio::LOCAL, J);
             file << "Jacobian for '" << jacobian_frame_name_ << "':\n" << J << "\n\n";
           } catch (...) {
             file << "Jacobian for '" << jacobian_frame_name_ << "': not available\n\n";
           }
           // Save the collision distance.
           try {
             pinocchio::FrameIndex fidA = model.getFrameId(collision_link_a_);
             pinocchio::FrameIndex fidB = model.getFrameId(collision_link_b_);
             double dist = (data.oMf[fidA].translation() - data.oMf[fidB].translation()).norm();
             file << "Collision Distance between '" << collision_link_a_ << "' and '" << collision_link_b_ << "': " << dist << " m\n";
           } catch (...) {
             file << "Collision Distance between '" << collision_link_a_ << "' and '" << collision_link_b_ << "': not available\n";
           }
           file.close();
           RCLCPP_INFO(this->get_logger(), "Computation results saved to %s", output_filename.c_str());
         }
       }
       catch (const std::exception & e) {
         RCLCPP_ERROR(this->get_logger(), "Error saving computation results: %s", e.what());
       }
     }
 
     // Set the flag so that further URDF messages are ignored.
     urdf_processed_ = true;
   }
 
   // Private member variables.
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
   bool urdf_processed_;             // Flag to ensure URDF is processed only once.
   // Parameters for configuring the robot model and computations.
   std::string base_link_name_;
   std::string jacobian_frame_name_;
   std::string collision_link_a_;
   std::string collision_link_b_;
   double collision_threshold_;
   bool save_output_;
 };
 
 int main(int argc, char** argv)
 {
   // Initialize the ROS 2 client library.
   rclcpp::init(argc, argv);
 
   // Create an instance of the PinocchioDemoNode.
   auto node = std::make_shared<PinocchioDemoNode>();
 
   // Spin the node to process callbacks.
   rclcpp::spin(node);
 
   // Shutdown and clean up.
   rclcpp::shutdown();
   return 0;
 }
 