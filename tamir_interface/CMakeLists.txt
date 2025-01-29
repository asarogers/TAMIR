cmake_minimum_required(VERSION 3.8)
project(tamir_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # Ensure this is necessary

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Waypoints.srv"
  "msg/Behaviors.msg"
  "msg/BehaviorList.msg"
  DEPENDENCIES geometry_msgs  # Remove if geometry_msgs is not used
)

# Export a dependency on rosidl_default_runtime
ament_export_dependencies(rosidl_default_runtime)

# Install directories for generated interfaces
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package
ament_package()
