cmake_minimum_required(VERSION 3.10)
project(sunray_tui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add third-party libraries
add_subdirectory(third_party/yaml-cpp)
add_subdirectory(third_party/ftxui)

# Add include directories
include_directories(include)

# Create executable with clean structure
add_executable(sunray_tui 
    src/tui_main.cpp
    src/tui_logic.cpp
    src/tui_interaction.cpp
    src/tui_render.cpp
    src/tui_render_buttons.cpp
    src/tui_render_keyboard.cpp
    src/tui_render_mouse.cpp
    src/tui_render_layout.cpp
    src/tui_render_cells.cpp
    src/tui_render_details.cpp
    src/tui_render_debug.cpp
    src/tui_render_component.cpp
    src/tui_core.cpp
    src/tui_screen.cpp
    src/tui_terminal.cpp
    src/tui_config.cpp
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