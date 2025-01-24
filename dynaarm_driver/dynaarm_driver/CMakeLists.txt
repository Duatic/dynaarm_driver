cmake_minimum_required(VERSION 3.8)
project(dynaarm_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-c++20-extensions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PACKAGE_DEPENDENCIES
  rclcpp
  pluginlib
  hardware_interface
  urdf
  dynaarm_hardware_interface_base
  ethercat_sdk_master
  rsl_drive_sdk
)

foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/dynaarm_hardware_interface.cpp
  src/dynaarm_mock_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})


ament_export_dependencies(${PACKAGE_DEPENDENCIES} yaml-cpp)
ament_export_libraries(
  ${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)

endif()

ament_package()
