cmake_minimum_required(VERSION 3.8)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(my_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(core_lib
  src/core/Robot.cpp
  src/core/Environment.cpp
  src/core/Waste.cpp
  src/core/Lidar.cpp
  src/core/canvas/Canvas.cpp
)

target_include_directories(core_lib PUBLIC include)
target_link_libraries(core_lib PUBLIC ${OpenCV_LIBS} yaml-cpp)

add_executable(robot_node
  src/wrappers/robot_node.cpp
)

target_link_libraries(robot_node core_lib)
ament_target_dependencies(robot_node rclcpp geometry_msgs robot_msgs)

add_executable(game_node
  src/wrappers/game_node.cpp
)

target_link_libraries(game_node core_lib)
ament_target_dependencies(game_node rclcpp robot_msgs)

add_executable(lidar_node
  src/wrappers/lidar_node.cpp
)

target_link_libraries(lidar_node core_lib)
ament_target_dependencies(lidar_node rclcpp robot_msgs sensor_msgs)

add_executable(teleop_node
  src/wrappers/teleop_node.cpp
)

ament_target_dependencies(teleop_node rclcpp geometry_msgs)

add_executable(visualization_node
  src/wrappers/visualization_node.cpp
)

target_link_libraries(visualization_node core_lib)
ament_target_dependencies(visualization_node rclcpp robot_msgs)

add_executable(monitor_node
  src/wrappers/monitor_node.cpp
)

target_link_libraries(monitor_node)
ament_target_dependencies(monitor_node rclcpp robot_msgs)

add_executable(interactive_teleop_node
  src/wrappers/interactive_teleop_node.cpp
)

target_link_libraries(interactive_teleop_node core_lib)
ament_target_dependencies(interactive_teleop_node rclcpp geometry_msgs robot_msgs sensor_msgs)

install(TARGETS
  robot_node
  game_node
  lidar_node
  teleop_node
  visualization_node
  monitor_node
  interactive_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
