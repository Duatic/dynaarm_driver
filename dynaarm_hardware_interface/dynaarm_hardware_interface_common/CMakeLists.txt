cmake_minimum_required(VERSION 3.8)
project(dynaarm_hardware_interface_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_DEPENDS
  rclcpp
)

include_directories(include)

ament_export_dependencies(${THIS_PACKAGE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

ament_package()