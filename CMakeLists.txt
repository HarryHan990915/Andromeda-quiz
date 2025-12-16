cmake_minimum_required(VERSION 3.8)
project(andromeda_proximity_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include headers
include_directories(include)

# -----------------------------
# Build library
# -----------------------------
add_library(proximity_filter_lib src/proximity_filter_lib.cpp)
target_include_directories(proximity_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(proximity_filter_lib
  rclcpp
  sensor_msgs
  geometry_msgs
)

# -----------------------------
# Build node executable
# -----------------------------
add_executable(proximity_filter_node
  src/proximity_filter_node.cpp
  src/main.cpp
)
target_link_libraries(proximity_filter_node
  proximity_filter_lib
)
ament_target_dependencies(proximity_filter_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS
  proximity_filter_node
  proximity_filter_lib
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/test_pointcloud_pub.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
