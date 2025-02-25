cmake_minimum_required(VERSION 3.8)
project(tutorial_pkg)

# Set C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Added missing dependency
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)  # Added useful dependency for image handling

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# First node
add_executable(my_first_node src/my_first_node.cpp)
ament_target_dependencies(my_first_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  std_srvs
)

# Tracker node
add_executable(tracker src/tracker.cpp)
ament_target_dependencies(tracker 
  rclcpp 
  OpenCV 
  cv_bridge
  geometry_msgs 
  sensor_msgs 
  std_msgs
  image_transport
)

ament_target_dependencies(tracker
  "rclcpp"
  "cv_bridge"
)

# Install targets
install(TARGETS
  my_first_node
  tracker
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()