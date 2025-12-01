cmake_minimum_required(VERSION 3.8)
project(user_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(jaka_msgs REQUIRED)

add_executable(moveit_client src/moveit_client.cpp)
ament_target_dependencies(
  moveit_client
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  jaka_msgs
)

# SDK 头文件和库，只给 rob_shutdown 用
add_executable(rob_shutdown src/rob_shutdown.cpp)
target_include_directories(rob_shutdown
  PUBLIC
    include
)
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
target_link_directories(rob_shutdown PUBLIC ${LIBRARY_DIR})
target_link_libraries(rob_shutdown jakaAPI)

install(
  PROGRAMS
    script/main_window.py 
  DESTINATION lib/${PROJECT_NAME}
  RENAME main_window 
)

install(
  PROGRAMS
    script/linear_move_ui.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    script/joint_move_ui.py 
  DESTINATION lib/${PROJECT_NAME}
)


install(
  PROGRAMS
    script/new_main_window.py 
  DESTINATION lib/${PROJECT_NAME}
  RENAME new_main_window 
)

install(
  PROGRAMS
    script/new_linear_move.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    script/new_joint_move.py 
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS 
  moveit_client rob_shutdown
  DESTINATION
  lib/${PROJECT_NAME})



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
