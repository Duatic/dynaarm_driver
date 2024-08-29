cmake_minimum_required(VERSION 3.8)
project(dynaarm_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME} SHARED   
  src/gravity_compensation_controller.cpp)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
  realtime_tools
  pinocchio
)
pluginlib_export_plugin_description_file(controller_interface controller_interface_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)
install(TARGETS ${PROJECT_NAME}  
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        moveit_core
)
ament_package()

