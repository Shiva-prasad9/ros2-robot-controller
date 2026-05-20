cmake_minimum_required(VERSION 3.8)
project(my_robot_controllers)

# C++17 standard — required for modern ROS2 C++
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ──────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ── Include directories ────────────────────────────────────
include_directories(include)

# ── Build the PID controller executable ───────────────────
add_executable(pid_controller
  src/pid_controller.cpp
)

ament_target_dependencies(pid_controller
  rclcpp
  geometry_msgs
  nav_msgs
)

# ── Install executable ─────────────────────────────────────
install(TARGETS
  pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install header files ───────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

ament_package()