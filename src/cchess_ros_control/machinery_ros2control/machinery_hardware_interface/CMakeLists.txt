cmake_minimum_required(VERSION 3.8)

project(machinery_hardware_interface)

# 使用 ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 设置编译选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 LibSerial 路径
set(LIBSERIAL_INCLUDE_DIR "/usr/include/libserial")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    # arm架构
    set(LIBSERIAL_LIBRARY "/usr/lib/aarch64-linux-gnu/libserial.so")
else()
    # x86架构
    set(LIBSERIAL_LIBRARY "/usr/lib/x86_64-linux-gnu/libserial.so")
endif()

# 创建库并自动处理 include 和依赖
ament_auto_add_library(machinery_hardware_interface SHARED
        src/machinery_hardware_interface.cpp
)

# 手动添加 LibSerial
target_include_directories(machinery_hardware_interface PUBLIC ${LIBSERIAL_INCLUDE_DIR})
target_link_libraries(machinery_hardware_interface ${LIBSERIAL_LIBRARY})

# 禁用 pluginlib 使用 boost
target_compile_definitions(machinery_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# 导出 plugin 描述文件
pluginlib_export_plugin_description_file(hardware_interface machinery_hardware_interface.xml)

# 安装头文件（ament_auto 会自动处理库的安装）
install(DIRECTORY include/ DESTINATION include)

# 测试
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gmock REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)

    ament_auto_add_gmock(test_machinery_hardware_interface test/test_machinery_hardware_interface.cpp)
    target_include_directories(test_machinery_hardware_interface PRIVATE include)
endif()

# 导出
ament_auto_package()
