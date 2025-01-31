cmake_minimum_required(VERSION 3.8)
project(k2_wire_feed)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(action_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(urdf REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WireFeed.action"
  DEPENDENCIES action_msgs
)

# Add executable and libraries
add_executable(k2_wf src/k2_wire_feed.cpp)
#add_dependencies(k2_wf ${PROJECT_NAME}_generate_interfaces)
add_library(supervisor src/Supervisor.cpp)
add_library(axis src/Axis.cpp)

# Find external shared libraries
find_library(sF_LIBRARIES libsFoundation20.so PATHS ${PROJECT_SOURCE_DIR}/lib/)
find_library(thread_LIBRARIES libpthread.so.0 PATHS /usr/lib/x86_64-linux-gnu)

message("Found sFoundation library at ${sF_LIBRARIES}")

# Link generated action messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Ensure all libraries are linked in a single call
target_link_libraries(k2_wf
  PUBLIC supervisor axis
  ${sF_LIBRARIES} ${thread_LIBRARIES}
  ${cpp_typesupport_target}
)

# Link dependencies
ament_target_dependencies(k2_wf PUBLIC 
  rclcpp 
  rclcpp_action 
  rclpy 
  action_msgs 
  pluginlib 
  rcpputils 
  urdf 
  builtin_interfaces 
  std_msgs 
  realtime_tools
)

# Set compilation features
target_compile_features(k2_wf PUBLIC cxx_std_17)

target_include_directories(k2_wf PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# INSTALLATION
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY action/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS k2_wf
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS supervisor axis
  EXPORT export_k2_wf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install generated headers
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_export_targets(export_k2_wf HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  rclcpp_action 
  rclpy 
  action_msgs 
  pluginlib 
  rcpputils 
  urdf 
  builtin_interfaces 
  std_msgs 
  realtime_tools 
  rosidl_default_runtime
)

ament_package()
