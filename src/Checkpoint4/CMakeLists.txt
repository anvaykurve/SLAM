cmake_minimum_required(VERSION 3.8)
project(Checkpoint4)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the executable
add_executable(ekf_localization_node src/ekf_localization_node.cpp)

# Link the ROS2 dependencies
ament_target_dependencies(ekf_localization_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
)

# Link the Eigen3 library
target_link_libraries(ekf_localization_node Eigen3::Eigen)

# Install the executable so ros2 run can find it
install(TARGETS
  ekf_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
