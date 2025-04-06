cmake_minimum_required(VERSION 3.8)
project(pinocchio_demo_cpp)

# We need C++14 or above for Pinocchio (ROS 2 default is usually fine)
set(CMAKE_CXX_STANDARD 14)

# Find the packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)  # Pinocchio library
find_package(Eigen3 REQUIRED)

# Create the executable
add_executable(pinocchio_demo_node src/pinocchio_demo_node.cpp)

# Link the libraries
ament_target_dependencies(pinocchio_demo_node
  rclcpp
  std_msgs
  pinocchio
)

target_link_libraries(pinocchio_demo_node
  Eigen3::Eigen
)


# Install the executable
install(TARGETS pinocchio_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
