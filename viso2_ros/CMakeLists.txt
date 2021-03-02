cmake_minimum_required(VERSION 3.5)
project(viso2_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(libviso2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisoInfo.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

add_library(stereo_odometer SHARED
  #src/covariance.cpp
  #src/reference_frame.cpp
  src/stereo_odometer_node.cpp)
ament_target_dependencies(stereo_odometer
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "std_msgs"
  "diagnostic_msgs"
  "sensor_msgs"
  "nav_msgs"
  "image_transport"
  "libviso2")

target_include_directories(stereo_odometer PUBLIC include)
rosidl_target_interfaces(stereo_odometer ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(stereo_odometer PLUGIN "viso2_ros::StereoOdometerNode" EXECUTABLE stereo_odometer_node)

add_library(mono_odometer SHARED
  src/mono_odometer_node.cpp)
ament_target_dependencies(mono_odometer
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "std_msgs"
  "diagnostic_msgs"
  "sensor_msgs"
  "nav_msgs"
  "image_transport"
  "libviso2")

target_include_directories(mono_odometer PUBLIC include)
rosidl_target_interfaces(mono_odometer ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(mono_odometer PLUGIN "viso2_ros::MonoOdometerNode" EXECUTABLE mono_odometer_node)

add_library(mono_odometer_omnidirectional SHARED
  src/mono_odometer_omnidirectional_node.cpp)
ament_target_dependencies(mono_odometer_omnidirectional
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "std_msgs"
  "diagnostic_msgs"
  "sensor_msgs"
  "nav_msgs"
  "image_transport"
  "libviso2")

target_include_directories(mono_odometer_omnidirectional PUBLIC include)
rosidl_target_interfaces(mono_odometer_omnidirectional ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(mono_odometer_omnidirectional PLUGIN "viso2_ros::MonoOdometerOmnidirectionalNode" EXECUTABLE mono_odometer_omnidirectional_node)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS stereo_odometer mono_odometer mono_odometer_omnidirectional
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
