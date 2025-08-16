cmake_minimum_required(VERSION 3.10)
project(sunray_tui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Linus风格：简单的DEBUG编译配置 - 默认关闭，按需启用
option(SUNRAY_DEBUG_ENABLED "Enable comprehensive debug logging and panel" ON)

if(SUNRAY_DEBUG_ENABLED)
    message(STATUS "Sunray TUI: DEBUG mode enabled - comprehensive logging active")
    add_compile_definitions(SUNRAY_DEBUG_ENABLED)
    # 调试模式下启用详细符号信息
    set(CMAKE_BUILD_TYPE Debug)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        add_compile_options(-g -O0)
    endif()
else()
    message(STATUS "Sunray TUI: RELEASE mode - debug logging disabled")
    # 生产模式下优化编译
    if(CMAKE_BUILD_TYPE STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

# Add third-party libraries
add_subdirectory(third_party/yaml-cpp)
add_subdirectory(third_party/FTXUI)

# Add include directories
include_directories(include)

# Create executable with clean structure
add_executable(sunray_tui 
    src/main/ui_main.cpp
    src/core/ui_logic.cpp
    src/render/ui_renderer_new.cpp  # 使用新版本
    src/core/build_output_manager.cpp  # 新增编译输出管理器
    src/ui_core.cpp
    src/screen_coordinate_mapper.cpp
    src/terminal_guard.cpp
)

# Link libraries
target_link_libraries(sunray_tui
    yaml-cpp
    ftxui::screen
    ftxui::dom
    ftxui::component
)
target_compile_definitions(sunray_tui PRIVATE HAVE_YAML_CPP HAVE_FTXUI)

# Set output directory
set_target_properties(sunray_tui PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
)