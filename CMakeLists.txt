cmake_minimum_required(VERSION 3.16)
project(linear_mpc_controller VERSION 0.3.0 LANGUAGES CXX)

# ROS-free core library.  Buildable standalone (WSL2 or any Linux) with only
# Eigen:  cmake -S . -B build && cmake --build build
# Under ROS2: colcon build --packages-select linear_mpc_controller
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# OSQP is optional for the core (model/fallback tests do not need it).
# The realtime solver gate requires OSQP (plan KTD12, frozen in deps.repos).
option(LINEAR_MPC_WITH_OSQP "Enable OSQP-backed QP solver" OFF)
if(LINEAR_MPC_WITH_OSQP)
  find_package(osqp REQUIRED)
  add_definitions(-DLINEAR_MPC_HAVE_OSQP=1)
endif()

add_library(linear_mpc_core
  src/model/differential_drive_model.cpp
  src/mpc/fallback_policy.cpp
  src/mpc/qp_problem.cpp
  src/mpc/linear_mpc.cpp
)
target_include_directories(linear_mpc_core PUBLIC include)
target_link_libraries(linear_mpc_core PUBLIC Eigen3::Eigen)
if(LINEAR_MPC_WITH_OSQP)
  target_link_libraries(linear_mpc_core PUBLIC osqp::osqp)
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_linear_mpc_core test/test_core.cpp)
  target_link_libraries(test_linear_mpc_core PRIVATE linear_mpc_core)
  add_test(NAME core COMMAND test_linear_mpc_core)
endif()

# ---- ROS2 nodes (only under colcon/ament) ----
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  add_executable(linear_mpc_node ros2/linear_mpc_node.cpp ros2/trajectory_adapter.cpp)
  target_include_directories(linear_mpc_node PRIVATE include ros2)
  ament_target_dependencies(linear_mpc_node rclcpp rclcpp_lifecycle std_msgs nav_msgs geometry_msgs)
  target_link_libraries(linear_mpc_node linear_mpc_core)
  install(TARGETS linear_mpc_node DESTINATION lib/${PROJECT_NAME})

  add_executable(velocity_arbiter_node ros2/velocity_arbiter.cpp)
  target_include_directories(velocity_arbiter_node PRIVATE include ros2)
  ament_target_dependencies(velocity_arbiter_node rclcpp geometry_msgs)
  install(TARGETS velocity_arbiter_node DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch config worlds maps
    DESTINATION share/${PROJECT_NAME})
  ament_package()
endif()
