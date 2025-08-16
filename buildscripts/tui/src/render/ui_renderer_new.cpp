#include "render/ui_renderer.hpp"
#include <iostream>  // for std::cout
#include <filesystem>

// 简化版debug宏
#define DEBUG_KEY(logger, event) do { \
    std::cout << "[KEY] " << event << std::endl; \
} while(0)

#ifdef HAVE_FTXUI
using namespace ftxui;
#endif

namespace sunray_tui {

UIRenderer::UIRenderer(UIState& state) 
    : state_(state) {}

int UIRenderer::run_with_build_callback(std::function<void()> build_callback) {
    build_callback_ = build_callback;
    
#ifndef HAVE_FTXUI
    return 1;  // 非FTXUI版本不支持高级渲染
#else
    // 新架构：不再需要复杂的作用域管理和终端切换
    TerminalGuard guard;
    
    auto component = create_component();
    auto screen = ScreenInteractive::Fullscreen();
    
    // Linus风格：简化事件处理，统一在一个地方处理所有状态变化
    auto wrapped_component = CatchEvent(component, [&](Event event) -> bool {
        // 更新编译状态（每次事件都检查）
        update_build_status();
        
        // 检查退出条件
        if (event == Event::Character('q') || event == Event::Escape) {
            if (state_.build_running) {
                // 如果编译正在进行，先终止编译
                state_.stop_build();
            }
            screen.ExitLoopClosure()();
            return true;
        }
        
        return false;  // 让其他事件正常处理
    });
    
    screen.Loop(wrapped_component);
    return 0;
#endif
}

#ifdef HAVE_FTXUI

void UIRenderer::start_build() {
    if (state_.build_running) {
        TRACE_CRITICAL("BUILD", "Build already running, ignoring request");
        return;
    }
    
    auto selected_modules = state_.selection_manager.get_selected_modules();
    if (selected_modules.empty()) {
        TRACE_CRITICAL("BUILD", "No modules selected for build");
        return;
    }
    
    // 构建命令
    std::string project_root = std::filesystem::current_path().string(); // 简化版本
    std::string build_script = project_root + "/build.sh";
    
    // 格式化模块参数
    std::string module_args;
    for (size_t i = 0; i < selected_modules.size(); ++i) {
        if (i > 0) module_args += " ";
        module_args += selected_modules[i];
    }
    
    std::string full_cmd = build_script + " --cli --from-tui " + module_args;
    
    TRACE_CRITICAL("BUILD_START", "Starting build with command: " + full_cmd);
    
    state_.start_build(full_cmd);
}

void UIRenderer::update_build_status() {
    if (!state_.build_running) {
        return;
    }
    
    // 更新编译状态 - 使用UIState统一管理
    state_.update_build_status();
}

Element UIRenderer::render_build_output() {
    if (!state_.build_output_visible) {
        return text("");
    }
    
    Elements output_elements;
    
    // 标题
    if (state_.build_running) {
        output_elements.push_back(text("=== 编译进行中... ===") | bold | color(Color::Yellow));
    } else if (state_.build_manager && state_.build_manager->get_state() != BuildOutputManager::BuildState::IDLE) {
        int exit_code = state_.build_manager->get_exit_code();
        if (exit_code == 0) {
            output_elements.push_back(text("=== 编译成功 ===") | bold | color(Color::Green));
        } else {
            output_elements.push_back(text("=== 编译失败 ===") | bold | color(Color::Red));
        }
    }
    
    // 输出内容
    auto lines = state_.get_build_output_lines(100);
    
    // 显示最后的几行（滚动窗口）
    int max_lines = 10;  // 限制显示行数
    int start_line = std::max(0, static_cast<int>(lines.size()) - max_lines + state_.build_output_scroll);
    int end_line = std::min(static_cast<int>(lines.size()), start_line + max_lines);
    
    for (int i = start_line; i < end_line; ++i) {
        if (i >= 0 && i < static_cast<int>(lines.size())) {
            // 简单的颜色处理：错误行用红色，警告行用黄色
            std::string line = lines[i];
            Element line_element = text(line);
            
            if (line.find("error:") != std::string::npos || line.find("ERROR") != std::string::npos) {
                line_element = line_element | color(Color::Red);
            } else if (line.find("warning:") != std::string::npos || line.find("WARNING") != std::string::npos) {
                line_element = line_element | color(Color::Yellow);
            }
            
            output_elements.push_back(line_element);
        }
    }
    
    // 如果没有输出，显示等待信息
    if (output_elements.size() <= 1) {
        output_elements.push_back(text("等待编译输出...") | dim);
    }
    
    return vbox(output_elements) | border | size(HEIGHT, LESS_THAN, 12);
}

Component UIRenderer::create_component() {
    // Color interpolation utilities for breathing hover effects
    auto interpolate_color = [](const ftxui::Color& from, const ftxui::Color& to, double phase) -> ftxui::Color {
        // Simple RGB interpolation for breathing effect
        if (phase <= 0.0) return from;
        if (phase >= 1.0) return to;
        
        // For simplicity, alternate between colors based on phase
        return (phase > 0.5) ? to : from;
    };
    
    auto get_hover_color = [&](const ftxui::Color& base_color, bool is_focused) -> ftxui::Color {
        if (!is_focused || !state_.two_section.hover_animation_active) {
            return base_color;
        }
        
        // Create breathing effect between base color and gray
        ftxui::Color gray = Color::RGB(128, 128, 128);  // Medium gray
        return interpolate_color(base_color, gray, state_.two_section.hover_animation_phase);
    };
    
    auto renderer = Renderer([&] {
        Elements elements;
        elements.push_back(text("Sunray TUI - 模块管理工具") | bold | center);
        elements.push_back(separator());
        
        // Section 1: Groups (flat list, no expansion/collapse)
        elements.push_back(text("=== 模块组 ===") | bold | color(Color::Cyan));
        
        for (size_t i = 0; i < state_.groups.size(); ++i) {
            const auto& group = state_.groups[i];
            bool is_focused = (state_.two_section.focus_on_groups && 
                             static_cast<int>(i) == state_.two_section.groups_cursor);
            
            Color base_color = state_.two_section.highlighted_groups.count(group.name) ? Color::Yellow : Color::GrayLight;
            Color display_color = get_hover_color(base_color, is_focused);
            
            auto group_element = text(group.display_name()) | color(display_color);
            if (is_focused) {
                group_element = group_element | bold;
            }
            elements.push_back(group_element);
        }
        
        elements.push_back(separator());
        
        // Section 2: Modules
        elements.push_back(text("=== 所有模块 ===") | bold | color(Color::Cyan));
        
        for (size_t i = 0; i < state_.modules.size(); ++i) {
            const auto& module = state_.modules[i];
            bool is_focused = (!state_.two_section.focus_on_groups && 
                             static_cast<int>(i) == state_.two_section.modules_cursor);
            bool is_selected = state_.two_section.is_module_selected(module.name);
            
            // Determine color based on state
            Color base_color;
            if (is_selected) {
                base_color = Color::Green;
            } else if (module.has_conflicts()) {
                base_color = Color::Red;
            } else {
                base_color = Color::GrayLight;
            }
            
            Color display_color = get_hover_color(base_color, is_focused);
            
            std::string prefix = is_selected ? "[√] " : "[ ] ";
            auto module_element = text(prefix + module.display_name()) | color(display_color);
            if (is_focused) {
                module_element = module_element | bold;
            }
            elements.push_back(module_element);
        }
        
        elements.push_back(separator());
        
        // Section 3: Description and Details
        elements.push_back(text("=== 详细信息 ===") | bold | color(Color::Cyan));
        
        // Show current item description
        if (!state_.current_item_description.empty()) {
            elements.push_back(text(state_.current_item_description) | bold | color(Color::White));
        }
        
        // Show current item details (multi-line)
        if (!state_.current_item_details.empty()) {
            Elements detail_lines;
            std::string details = state_.current_item_details;
            size_t start = 0;
            
            while (start < details.length()) {
                size_t end = details.find('\n', start);
                std::string line = details.substr(start, (end == std::string::npos) ? end : end - start);
                start = (end == std::string::npos) ? details.length() : end + 1;
                
                // Skip completely empty lines to avoid layout issues
                if (line.empty()) {
                    continue;
                }
                
                // Check if this is a conflict line with actual conflicts
                if (line.find("冲突: ") == 0 && line.find("NULL") == std::string::npos) {
                    // Conflict line with content - apply highlighting and flash
                    if (state_.conflict_flash_active && (state_.conflict_flash_count % 2 == 1)) {
                        // Flash animation: alternate between yellow and red
                        detail_lines.push_back(text(line) | bgcolor(Color::Yellow) | color(Color::Black) | bold);
                    } else {
                        // Normal conflict highlighting: red background
                        detail_lines.push_back(text(line) | bgcolor(Color::Red) | color(Color::White) | bold);
                    }
                } else {
                    // Normal line
                    detail_lines.push_back(text(line) | dim);
                }
                
                if (end == std::string::npos) break;
            }
        }
        
        // Build button
        elements.push_back(separator());
        Element build_button_content;
        if (state_.current_focus == FocusTarget::BUILD_BUTTON) {
            if (state_.build_running) {
                build_button_content = text("【 编译进行中... 】") | bold | bgcolor(Color::Yellow) | color(Color::Black);
            } else {
                build_button_content = text("【 开始编译构建 】") | bold | bgcolor(Color::Blue) | color(Color::White);
            }
        } else {
            if (state_.build_running) {
                build_button_content = text("【 编译进行中... 】") | bold | color(Color::Yellow);
            } else {
                build_button_content = text("【 开始编译构建 】") | bold | color(Color::Blue);
            }
        }
        elements.push_back(hbox({
            filler(),
            build_button_content,
            filler()
        }) | center);
        
        // 新增：编译输出窗口
        if (state_.build_output_visible) {
            elements.push_back(separator());
            elements.push_back(render_build_output());
        }
        
        // Keyboard hints
        elements.push_back(separator());
        elements.push_back(hbox({
            vbox({
                text("上下导航") | color(Color::GrayLight),
                text("Tab切换") | color(Color::GrayLight),
                text("回车选择") | color(Color::GrayLight),
                text("清除选择") | color(Color::GrayLight),
                text("退出程序") | color(Color::GrayLight)
            }) | flex,
            text("  ") | color(Color::Default),
            vbox({
                text("↑↓ 方向键") | color(Color::Cyan),
                text("Tab键") | color(Color::Cyan),
                text("回车键") | color(Color::Cyan),
                text("C键") | color(Color::Cyan),
                text("q/Esc键") | color(Color::Cyan)
            }) | flex
        }));

        
        return vbox(elements) | border;
    });
    
    return CatchEvent(renderer, [this](Event event) {
        // 简化版事件记录
        std::string event_name = "Unknown";
        if (event == Event::Tab) event_name = "Tab";
        else if (event == Event::TabReverse) event_name = "Shift+Tab";
        else if (event == Event::ArrowUp) event_name = "ArrowUp";
        else if (event == Event::ArrowDown) event_name = "ArrowDown";
        else if (event == Event::Return) event_name = "Return";
        else if (event == Event::Character('C')) event_name = "C";
        else if (event == Event::Character('c')) event_name = "c";
        else if (event == Event::Character('D')) event_name = "D";
        else if (event == Event::Character('d')) event_name = "d";
        else if (event == Event::Character('q')) event_name = "q";
        else if (event == Event::Escape) event_name = "Escape";
        else if (event == Event::CtrlC) event_name = "Ctrl+C";
        else if (event.is_mouse()) event_name = "Mouse";
        
        // Tab键处理 - 新增BUILD_OUTPUT焦点
        if (event == Event::Tab) {
            switch (state_.current_focus) {
                case FocusTarget::GROUPS:
                    state_.current_focus = FocusTarget::MODULES;
                    state_.two_section.focus_on_groups = false;
                    break;
                case FocusTarget::MODULES:
                    state_.current_focus = FocusTarget::BUILD_BUTTON;
                    break;
                case FocusTarget::BUILD_BUTTON:
                    if (state_.build_output_visible) {
                        state_.current_focus = FocusTarget::BUILD_OUTPUT;
                    } else {
                        state_.current_focus = FocusTarget::GROUPS;
                        state_.two_section.focus_on_groups = true;
                    }
                    break;
                case FocusTarget::BUILD_OUTPUT:
                    state_.current_focus = FocusTarget::GROUPS;
                    state_.two_section.focus_on_groups = true;
                    break;
            }
            return true;
        }
        
        // Enter键处理
        if (event == Event::Return) {
            if (state_.current_focus == FocusTarget::BUILD_BUTTON && !state_.build_running) {
                start_build();
                return true;
            }
            // 其他Enter键处理逻辑...
        }
        
        // 编译输出窗口的滚动处理
        if (state_.current_focus == FocusTarget::BUILD_OUTPUT) {
            if (event == Event::ArrowUp) {
                state_.build_output_scroll = std::max(state_.build_output_scroll - 1, -10);
                return true;
            } else if (event == Event::ArrowDown) {
                state_.build_output_scroll = std::min(state_.build_output_scroll + 1, 10);
                return true;
            }
        }
        
        // 其他事件处理...
        return false;
    });
}

#endif

} // namespace sunray_tui