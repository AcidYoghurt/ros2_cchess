cmake_minimum_required(VERSION 3.8)
project(gripper_suction_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================
# ament_cmake_auto
# =====================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Extra dependencies not auto-found
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

# =====================================
# Parameter library
# =====================================
generate_parameter_library(gripper_suction_controller_parameters
        src/gripper_suction_controller.yaml
        include/gripper_suction_controller/validate_gripper_suction_controller_parameters.hpp
)

# =====================================
# Controller library
# =====================================
ament_auto_add_library(
        gripper_suction_controller
        SHARED
        src/gripper_suction_controller.cpp
)

target_include_directories(gripper_suction_controller PUBLIC
        "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(
        gripper_suction_controller
        gripper_suction_controller_parameters
)

target_compile_definitions(
        gripper_suction_controller
        PRIVATE GRIPPER_SUCTION_CONTROLLER_BUILDING_DLL
)

# =====================================
# pluginlib
# =====================================
pluginlib_export_plugin_description_file(
        controller_interface
        gripper_suction_controller.xml
)

# =====================================
# Package
# =====================================
ament_auto_package()
