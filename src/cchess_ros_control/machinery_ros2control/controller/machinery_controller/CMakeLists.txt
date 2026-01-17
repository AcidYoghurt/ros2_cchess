cmake_minimum_required(VERSION 3.8)
project(machinery_controller)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========= ament_cmake_auto =========
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ========= Library =========
ament_auto_add_library(
        machinery_controller
        SHARED
        src/machinery_controller.cpp
)

target_include_directories(machinery_controller PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_definitions(
        machinery_controller
        PRIVATE MACHINERY_CONTROLLER_BUILDING_DLL
)

# ========= pluginlib =========
pluginlib_export_plugin_description_file(
        controller_interface
        machinery_controller.xml
)

# ========= Install =========
ament_auto_package(
        INSTALL_TO_SHARE
)
