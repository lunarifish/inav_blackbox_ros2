cmake_minimum_required(VERSION 3.8)
project(inav_blackbox_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(inav_blackbox_msgs REQUIRED)

add_executable(inav_blackbox_to_bag
  src/main.cpp
  src/bag_writer.cpp
  src/blackbox_decoder.cpp
  src/csv_table.cpp
  src/inav_frames.cpp
  src/inav_sample.cpp
  src/topic_writer.cpp
)
target_include_directories(inav_blackbox_to_bag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(inav_blackbox_to_bag
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rcutils
  rosidl_runtime_cpp
  ament_index_cpp
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
  inav_blackbox_msgs
)

install(TARGETS inav_blackbox_to_bag DESTINATION lib/${PROJECT_NAME})

# The vendored iNavFlight/blackbox-tools decoder; the node looks for it here
# before falling back to $PATH.
install(PROGRAMS bin/blackbox_decode DESTINATION share/${PROJECT_NAME}/bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # Copyright headers live in the files themselves, cpplint's whitelist fights
  # the C++17 code, and the tree is formatted with .clang-format, which
  # uncrustify's ROS style contradicts.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(bag_contents test/test_bag_contents.py)
endif()

ament_export_dependencies(
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
  inav_blackbox_msgs
)
ament_package()
