cmake_minimum_required(VERSION 3.5)
project(trajectory_broadcaster)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(PCL REQUIRED COMPONENTS io)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(trajectory_broadcaster_node SHARED
  src/trajectory_broadcaster/trajectory_broadcaster_node.cpp
)
target_link_libraries(trajectory_broadcaster_node ${PCL_LIBRARIES})

rclcpp_components_register_node(trajectory_broadcaster_node
  PLUGIN "TrajectoryBroadcasterNode"
  EXECUTABLE trajectory_broadcaster
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
