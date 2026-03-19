cmake_minimum_required(VERSION 3.8)
project(cartesian_move_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# ament_cmake_auto
# =========================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# =========================
# parameter library
# =========================
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(cartesian_move_controller_parameters
        src/cartesian_move_controller.yaml
        include/cartesian_move_controller/validate_cartesian_move_controller_parameters.hpp
)

# =========================
# controller library
# =========================
ament_auto_add_library(
        cartesian_move_controller
        SHARED
        src/cartesian_move_controller.cpp
)

target_include_directories(cartesian_move_controller PUBLIC
        "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(
        cartesian_move_controller
        cartesian_move_controller_parameters
)

target_compile_definitions(
        cartesian_move_controller
        PRIVATE CARTESIAN_MOVE_CONTROLLER_BUILDING_DLL
)

# =========================
# pluginlib export
# =========================
pluginlib_export_plugin_description_file(
        controller_interface
        cartesian_move_controller.xml
)

# =========================
# install
# =========================
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

ament_auto_package()
