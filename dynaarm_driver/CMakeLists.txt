cmake_minimum_required(VERSION 3.8)
project(dynaarm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)
find_package(rsl_drive_sdk REQUIRED)
find_package(ethercat_sdk_master REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(dynaarm_driver SHARED
  src/dynaarm_hardware_interface.cpp  
)
target_include_directories(dynaarm_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(dynaarm_driver PUBLIC -fPIC)
target_compile_features(dynaarm_driver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dynaarm_driver
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  urdf
  rsl_drive_sdk
  ethercat_sdk_master
  Eigen3

)
# For some reason we need to directly link to yaml-cpp otherwise it will not declare it as a direct dependency
# NOTE: Should actually be covered by the yaml_cpp_vendor
target_link_libraries(dynaarm_driver yaml-cpp)
install(TARGETS
  dynaarm_driver
  DESTINATION lib/${PROJECT_NAME})

include_directories(include)

ament_export_include_directories(include)

ament_export_libraries(
  dynaarm_driver
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  urdf
  rsl_drive_sdk
  ethercat_sdk_master
  
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(TARGETS dynaarm_driver DESTINATION lib/)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch  config
  DESTINATION share/${PROJECT_NAME}
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
