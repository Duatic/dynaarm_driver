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

find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)
find_package(dynaarm_msgs REQUIRED)

find_package(generate_parameter_library REQUIRED)
generate_parameter_library(
    dynaarm_status_broadcaster_parameters
    src/dynaarm_status_broadcaster_parameters.yaml
)

generate_parameter_library(
  gravity_compensation_controller_parameters
  src/gravity_compensation_controller_parameters.yaml
)

generate_parameter_library(
  freedrive_controller_parameters
  src/freedrive_controller_parameters.yaml
)

generate_parameter_library(
  dynaarm_pid_tuner_parameters
  src/dynaarm_pid_tuner_parameters.yaml
)

generate_parameter_library(
  freeze_controller_parameters
  src/freeze_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/gravity_compensation_controller.cpp
  src/dynaarm_status_broadcaster.cpp
  src/freedrive_controller.cpp
  src/dynaarm_pid_tuner.cpp
  src/freeze_controller.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  dynaarm_status_broadcaster_parameters
  gravity_compensation_controller_parameters
  freedrive_controller_parameters
  dynaarm_pid_tuner_parameters
  freeze_controller_parameters
  pinocchio::pinocchio
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
  dynaarm_msgs
  Eigen3
  eigen3_cmake_module
)
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_interface_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  TARGETS  ${PROJECT_NAME}
  DESTINATION lib/
)

if(BUILD_TESTING)

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
        dynaarm_msgs
        moveit_core
)
ament_package()
