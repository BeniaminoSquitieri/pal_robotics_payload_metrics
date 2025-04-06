/*******************************************************
 * pinocchio_demo_node.cpp
 * 
 * A ROS 2 C++ node that:
 * 1) Subscribes to /robot_description
 * 2) Builds a Pinocchio model (from a URDF file)
 * 3) Performs forward kinematics to get a frame transform
 * 4) Computes a Jacobian for a specified link
 * 5) Checks the distance between two links (collision check)
 *
 * The logs are formatted to provide clearer numeric output.
 *******************************************************/

 #include <rclcpp/rclcpp.hpp>
 #include <std_msgs/msg/string.hpp>
 
 // Pinocchio includes for URDF parsing and kinematics
 #include <pinocchio/parsers/urdf.hpp>
 #include <pinocchio/algorithm/kinematics.hpp>
 #include <pinocchio/algorithm/jacobian.hpp>
 #include <pinocchio/algorithm/frames.hpp>
 #include <pinocchio/algorithm/joint-configuration.hpp>
 
 #include <fstream>
 #include <cstdio>    // for std::remove
 #include <string>
 #include <sstream>
 #include <iomanip>   // for std::setprecision
 
 class PinocchioDemoNode : public rclcpp::Node
 {
 public:
   PinocchioDemoNode()
   : Node("pinocchio_demo_node")
   {
     // Create a QoS profile with transient_local durability so we can latch the URDF
     rclcpp::QoS qos(rclcpp::KeepLast(10));
     qos.transient_local();
 
     RCLCPP_INFO(this->get_logger(),
       "[PinocchioDemoNode] Subscribing to /robot_description with TRANSIENT_LOCAL QoS."
     );
 
     // Subscription to "/robot_description" 
     sub_ = this->create_subscription<std_msgs::msg::String>(
       "/robot_description",
       qos,
       std::bind(&PinocchioDemoNode::onURDFReceived, this, std::placeholders::_1)
     );
   }
 
 private:
   void onURDFReceived(const std_msgs::msg::String::SharedPtr msg)
   {
     RCLCPP_INFO(this->get_logger(),
       "\n========== STEP 1: Building Pinocchio Model from URDF =========="
     );
 
     // 1) Write URDF to a temporary file
     std::string urdf_string = msg->data;
     std::string tempPath = "/tmp/temp_robot.urdf";
 
     {
       std::ofstream out(tempPath);
       if(!out.is_open())
       {
         RCLCPP_ERROR(this->get_logger(), "Cannot open %s for writing URDF!", tempPath.c_str());
         return;
       }
       out << urdf_string;
       out.close();
     }
 
     // Build a Pinocchio model
     pinocchio::Model model;
     try
     {
       // We can use the older signature that fills 'model' in place
       // buildModel(filename, rootJoint, model, verboseFlag)
       pinocchio::urdf::buildModel(tempPath, pinocchio::JointModelFreeFlyer(), model, false);
 
       RCLCPP_INFO(this->get_logger(), "Pinocchio Model created successfully. (nq=%d, nv=%d)",
                   model.nq, model.nv);
     }
     catch(const std::exception & e)
     {
       RCLCPP_ERROR(this->get_logger(), "Could not build model: %s", e.what());
       std::remove(tempPath.c_str());
       return;
     }
 
     // Clean up the temporary file
     std::remove(tempPath.c_str());
 
     // 2) Forward Kinematics
     RCLCPP_INFO(this->get_logger(),
       "\n========== STEP 2: Forward Kinematics on a Neutral Configuration =========="
     );
 
     pinocchio::Data data(model);
 
     // Create a "neutral" config. In older Pinocchio versions, we might do zeroes:
     Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
 
     try
     {
       // forwardKinematics => calculates joint placements
       pinocchio::forwardKinematics(model, data, q);
       // framesForwardKinematics => updates frame positions data.oMf
       pinocchio::framesForwardKinematics(model, data, q);
 
       RCLCPP_INFO(this->get_logger(), "Forward kinematics completed successfully.");
     }
     catch(const std::exception & e)
     {
       RCLCPP_ERROR(this->get_logger(), "Forward kinematics failed: %s", e.what());
       return;
     }
 
     // Retrieve transform for 'base_link' (example frame)
     std::string baseLinkName = "base_link";
     RCLCPP_INFO(this->get_logger(), "[FK] Retrieving transform for frame '%s'...", baseLinkName.c_str());
     try
     {
       pinocchio::FrameIndex fid = model.getFrameId(baseLinkName);
       auto & transform = data.oMf[fid];
       const auto & t = transform.translation();
       RCLCPP_INFO(this->get_logger(),
         "[FK] '%s' transform => translation [%.4f, %.4f, %.4f]",
         baseLinkName.c_str(),
         t[0], t[1], t[2]
       );
     }
     catch(const std::exception & e)
     {
       RCLCPP_ERROR(this->get_logger(), "Cannot get transform for '%s': %s",
                    baseLinkName.c_str(), e.what());
     }
 
     // 3) Calculate Jacobian for e.g. 'arm_7_link'
     RCLCPP_INFO(this->get_logger(),
       "\n========== STEP 3: Calculating Jacobian for 'arm_7_link' =========="
     );
     std::string jacFrameName = "arm_7_link";
 
     try
     {
       pinocchio::FrameIndex fid_jac = model.getFrameId(jacFrameName);
 
       // Must compute joint Jacobians first for the entire robot
       pinocchio::computeJointJacobians(model, data, q);
       // Then update frames
       pinocchio::framesForwardKinematics(model, data, q);
 
       // The 6D local Jacobian
       Eigen::MatrixXd J(6, model.nv);
       pinocchio::getFrameJacobian(model, data, fid_jac, pinocchio::LOCAL, J);
 
       RCLCPP_INFO(this->get_logger(),
         "Jacobian for '%s' has dimension [%ld x %ld]",
         jacFrameName.c_str(), J.rows(), J.cols()
       );
       // Print the matrix in a more readable format
       for(int r = 0; r < J.rows(); ++r)
       {
         std::stringstream row;
         row << "  [ ";
         for(int c = 0; c < J.cols(); ++c)
         {
           row << std::fixed << std::setprecision(3) << J(r,c);
           if(c < J.cols() - 1) row << ", ";
         }
         row << " ]";
         RCLCPP_INFO(this->get_logger(), "%s", row.str().c_str());
       }
     }
     catch(const std::exception & e)
     {
       RCLCPP_ERROR(this->get_logger(),
         "Jacobian calculation failed for '%s': %s",
         jacFrameName.c_str(), e.what());
     }
 
     // 4) Basic "Collision" check by measuring distance between 'arm_1_link' and 'arm_4_link'
     RCLCPP_INFO(this->get_logger(),
       "\n========== STEP 4: Checking distance between two frames (collision check) =========="
     );
     std::string linkA = "arm_1_link";
     std::string linkB = "arm_4_link";
 
     try
     {
       pinocchio::FrameIndex fidA = model.getFrameId(linkA);
       pinocchio::FrameIndex fidB = model.getFrameId(linkB);
 
       const auto & posA = data.oMf[fidA].translation();
       const auto & posB = data.oMf[fidB].translation();
 
       double dist = (posA - posB).norm();
       RCLCPP_INFO(this->get_logger(),
         "[Collision] Distance between '%s' and '%s' => %.4f m",
         linkA.c_str(), linkB.c_str(), dist);
 
       if(dist < 0.05)
       {
         RCLCPP_WARN(this->get_logger(),
           "[Collision] Potential collision! Dist < 0.05 m between '%s' and '%s'.",
           linkA.c_str(), linkB.c_str());
       }
       else
       {
         RCLCPP_INFO(this->get_logger(),
           "[Collision] No collision between '%s' and '%s'.",
           linkA.c_str(), linkB.c_str());
       }
     }
     catch(const std::exception & e)
     {
       RCLCPP_ERROR(this->get_logger(),
         "Collision check failed between '%s' and '%s': %s",
         linkA.c_str(), linkB.c_str(), e.what());
     }
   }
 
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
 };
 
 int main(int argc, char** argv)
 {
   rclcpp::init(argc, argv);
 
   auto node = std::make_shared<PinocchioDemoNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
 }
 