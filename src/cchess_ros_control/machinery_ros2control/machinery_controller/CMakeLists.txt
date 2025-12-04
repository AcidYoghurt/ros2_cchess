cmake_minimum_required(VERSION 3.8)
project(machinery_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add machinery_controller library related compile commands
add_library(
  machinery_controller
  SHARED
  src/machinery_controller.cpp
)
ament_target_dependencies(machinery_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(machinery_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_definitions(machinery_controller PRIVATE "MACHINERY_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface machinery_controller.xml)

install(
  TARGETS
  machinery_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  machinery_controller
)

ament_package()
