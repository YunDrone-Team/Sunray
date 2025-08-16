#include "render/ui_renderer.hpp"
#include "debug/dual_logger.hpp"
#include <iostream>  // for std::cout
#include <unistd.h>  // for execl
// #include "../include/terminal_guard.hpp" // 移除直接包含，让头文件处理

#ifdef HAVE_FTXUI
using namespace ftxui;
#endif

namespace sunray_tui {

UIRenderer::UIRenderer(UIState& state) : state_(state) {}

int UIRenderer::run_with_build_callback(std::function<void()> build_callback) {
    build_callback_ = build_callback;
    
#ifndef HAVE_FTXUI
    return 1;  // 非FTXUI版本不支持高级渲染
#else
    // 视觉优先：确保完整的作用域管理
    bool should_build = false;
    
    {
        // TerminalGuard作用域：确保TUI完全退出后再执行构建
        TerminalGuard guard;
        
        auto component = create_component();
        auto screen = ScreenInteractive::Fullscreen();
        
        // Linus风格：直接事件响应，消除polling延迟
        // 将screen对象传递给组件，允许直接退出
        auto wrapped_component = CatchEvent(component, [&](Event event) -> bool {
            // 检查是否需要退出TUI（构建请求）
            if (state_.build_requested) {
                DEBUG_STATE(state_.debug_logger, "TUILoop", "Build requested, exiting TUI loop");
                state_.build_requested = false;
                should_build = true;
                screen.ExitLoopClosure()();  // 优雅退出TUI循环
                return true;
            }
            return false;  // 让其他事件正常处理
        });
        
        // 将screen引用传递给state，允许从事件处理中直接退出
        state_.screen_exit_callback = [&screen]() {
            screen.ExitLoopClosure()();
        };
        
        screen.Loop(wrapped_component);
        
        // TUI循环结束，显示退出消息
        if (should_build) {
            std::cout << "\n正在退出TUI界面...\n" << std::flush;
        }
    } // TerminalGuard在这里析构，确保终端状态完全恢复
    
    // 现在TerminalGuard已完全析构，可以安全执行构建
    if (should_build && build_callback_) {
        build_callback_();  // 调用UILogic::execute_build()
    }
    
    return 0;
#endif
}

#ifdef HAVE_FTXUI
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
            bool is_highlighted = state_.two_section.is_group_highlighted(group.name);
            
            Element group_content = text("  " + group.name);
            
            if (is_highlighted) {
                // Group is highlighted (all modules selected) - yellow background with hover effect
                ftxui::Color hover_bg = get_hover_color(Color::Yellow, is_focused);
                group_content = group_content | bgcolor(hover_bg) | color(Color::Black) | bold;
            } else if (is_focused) {
                // Group has focus - blue indicator with hover background effect
                ftxui::Color hover_bg = get_hover_color(Color::RGB(30, 30, 30), true);  // Dark background
                group_content = hbox({
                    text("→") | color(Color::Blue) | bold,
                    text(" "),
                    group_content | flex
                }) | bgcolor(hover_bg);
            } else {
                // Normal group
                group_content = hbox({
                    text("  "),
                    group_content | flex
                });
            }
            
            elements.push_back(group_content);
        }
        
        elements.push_back(separator());
        
        // Section 2: Modules (fully expanded, flat list)
        elements.push_back(text("=== 所有模块 ===") | bold | color(Color::Cyan));
        
        for (size_t i = 0; i < state_.modules.size(); ++i) {
            const auto& module = state_.modules[i];
            bool is_focused = (!state_.two_section.focus_on_groups && 
                             static_cast<int>(i) == state_.two_section.modules_cursor);
            bool is_selected = state_.two_section.is_module_selected(module.name);
            bool is_disabled = state_.is_module_disabled(module.name);
            
            Element module_content = text("  " + module.name);
            
            if (is_disabled) {
                // Module is disabled due to conflicts - gray and dim
                module_content = hbox({
                    text("✗") | color(Color::GrayDark) | dim,
                    text(" "),
                    module_content | flex
                }) | color(Color::GrayDark) | dim;
                
                // Add hover effect even for disabled modules
                if (is_focused) {
                    ftxui::Color hover_bg = get_hover_color(Color::RGB(60, 60, 60), true);
                    module_content = module_content | bgcolor(hover_bg);
                }
            } else if (is_selected) {
                // Module is selected - green background with hover effect
                ftxui::Color hover_bg = get_hover_color(Color::Green, is_focused);
                module_content = hbox({
                    text("✓") | color(Color::White) | bold,
                    text(" "),
                    module_content | flex
                }) | bgcolor(hover_bg) | color(Color::White);
            } else if (is_focused) {
                // Module has focus - blue indicator with hover background effect
                ftxui::Color hover_bg = get_hover_color(Color::RGB(30, 30, 30), true);  // Dark background
                module_content = hbox({
                    text("→") | color(Color::Blue) | bold,
                    text(" "),
                    module_content | flex
                }) | bgcolor(hover_bg);
            } else {
                // Normal module
                module_content = hbox({
                    text("  "),
                    module_content | flex
                });
            }
            
            elements.push_back(module_content);
        }
        
        // Enhanced Description section - Fixed 4-line format
        elements.push_back(separator());
        elements.push_back(text("描述: " + state_.current_item_description) | bold);
        
        // Process details lines with conflict highlighting
        std::vector<Element> detail_lines;
        if (!state_.current_item_details.empty()) {
            std::string details = state_.current_item_details;
            size_t pos = 0;
            while (pos < details.length()) {
                size_t end = details.find('\n', pos);
                std::string line;
                if (end == std::string::npos) {
                    line = details.substr(pos);
                    pos = details.length();
                } else {
                    line = details.substr(pos, end - pos);
                    pos = end + 1;
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
        
        // Ensure exactly 3 detail lines (description + 3 = 4 total lines)
        while (detail_lines.size() < 3) {
            detail_lines.push_back(text("") | dim);  // Add empty lines
        }
        
        // Display only first 3 detail lines
        for (size_t i = 0; i < std::min(detail_lines.size(), size_t(3)); ++i) {
            elements.push_back(detail_lines[i]);
        }
        
        // Build button
        elements.push_back(separator());
        Element build_button_content;
        if (state_.build_button_focused) {
            build_button_content = text("【 开始编译构建 】") | bold | bgcolor(Color::Blue) | color(Color::White);
        } else {
            build_button_content = text("【 开始编译构建 】") | bold | color(Color::Blue);
        }
        elements.push_back(hbox({
            filler(),
            build_button_content,
            filler()
        }) | center);
        
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
#ifdef SUNRAY_DEBUG_ENABLED
                , text("D键调试") | color(Color::Cyan)
#endif
            }) | flex
        }));

        // Linus风格DEBUG面板 - 解耦设计，条件渲染
#ifdef SUNRAY_DEBUG_ENABLED
        if (state_.debug_panel_enabled) {
            elements.push_back(separator());
            elements.push_back(render_debug_panel());
        }
#endif
        
        return vbox(elements) | border;
    });
    
    return CatchEvent(renderer, [this](Event event) {
        // Linus风格：记录每个事件 - 追踪"按Enter无响应"问题
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
        
        DEBUG_KEY(state_.debug_logger, event_name);
        
        // Tab键处理 - 追踪焦点变化
        if (event == Event::Tab) {
            FocusTarget old_focus = state_.current_focus;
            state_.cycle_focus_forward();
            FocusTarget new_focus = state_.current_focus;
            
            std::string old_focus_str = debug::focus_target_to_string(static_cast<int>(old_focus));
            std::string new_focus_str = debug::focus_target_to_string(static_cast<int>(new_focus));
            
            TRACE_CRITICAL("FOCUS_CHANGE", old_focus_str + " -> " + new_focus_str + " (Tab key)");
            DEBUG_FOCUS(state_.debug_logger, old_focus_str, new_focus_str);
            return true;
        }
        
        // Shift+Tab反向循环支持
        if (event == Event::TabReverse) {
            FocusTarget old_focus = state_.current_focus;
            state_.cycle_focus_backward();
            FocusTarget new_focus = state_.current_focus;
            DEBUG_FOCUS(state_.debug_logger,
                        debug::focus_target_to_string(static_cast<int>(old_focus)),
                        debug::focus_target_to_string(static_cast<int>(new_focus)));
            return true;
        }
        
        // Arrow keys - navigation within current section
        if (event == Event::ArrowUp) {
            DEBUG_STATE(state_.debug_logger, "Navigation", "ArrowUp - focus=" + 
                        debug::focus_target_to_string(static_cast<int>(state_.current_focus)));
            if (!state_.is_build_button_focused()) {
                if (state_.is_groups_focused()) {
                    if (state_.two_section.groups_cursor > 0) {
                        state_.two_section.groups_cursor--;
                        state_.update_current_item_info();
                        DEBUG_STATE(state_.debug_logger, "Cursor", "groups_cursor=" + std::to_string(state_.two_section.groups_cursor));
                    }
                } else if (state_.is_modules_focused()) {
                    if (state_.two_section.modules_cursor > 0) {
                        state_.two_section.modules_cursor--;
                        state_.update_current_item_info();
                        DEBUG_STATE(state_.debug_logger, "Cursor", "modules_cursor=" + std::to_string(state_.two_section.modules_cursor));
                    }
                }
            }
            return true;
        }
        
        if (event == Event::ArrowDown) {
            DEBUG_STATE(state_.debug_logger, "Navigation", "ArrowDown - focus=" + 
                        debug::focus_target_to_string(static_cast<int>(state_.current_focus)));
            if (!state_.is_build_button_focused()) {
                if (state_.is_groups_focused()) {
                    if (state_.two_section.groups_cursor < static_cast<int>(state_.groups.size()) - 1) {
                        state_.two_section.groups_cursor++;
                        state_.update_current_item_info();
                        DEBUG_STATE(state_.debug_logger, "Cursor", "groups_cursor=" + std::to_string(state_.two_section.groups_cursor));
                    }
                } else if (state_.is_modules_focused()) {
                    if (state_.two_section.modules_cursor < static_cast<int>(state_.modules.size()) - 1) {
                        state_.two_section.modules_cursor++;
                        state_.update_current_item_info();
                        DEBUG_STATE(state_.debug_logger, "Cursor", "modules_cursor=" + std::to_string(state_.two_section.modules_cursor));
                    }
                }
            }
            return true;
        }
        
        // Enter key - selection using unified focus API
        if (event == Event::Return) {
            std::string current_focus_str = debug::focus_target_to_string(static_cast<int>(state_.current_focus));
            
            TRACE_CRITICAL("KEY_PRESS", "Return key pressed, focus=" + current_focus_str);
            DEBUG_EVENT(state_.debug_logger, "ENTER", "Processing Return key, focus=" + current_focus_str);
            
            if (state_.is_build_button_focused()) {
                // Linus风格：直接事件响应，立即退出TUI
                TRACE_CRITICAL("BUILD_BUTTON_PRESSED", "Starting build process");
                DEBUG_BUILD(state_.debug_logger);
                DEBUG_STATE(state_.debug_logger, "BuildRequest", "Setting build_requested=true");
                state_.build_requested = true;
                
                // 立即退出TUI循环 - 无需等待下一个事件
                DEBUG_STATE(state_.debug_logger, "TUIExit", "Calling immediate screen exit");
                if (state_.screen_exit_callback) {
                    state_.screen_exit_callback();
                }
                return true;
            } else if (state_.is_groups_focused()) {
                // Select/deselect current group
                if (state_.two_section.groups_cursor < static_cast<int>(state_.groups.size())) {
                    const std::string& group_name = state_.groups[state_.two_section.groups_cursor].name;
                    DEBUG_STATE(state_.debug_logger, "GroupToggle", "Toggling group: " + group_name);
                    state_.selection_manager.toggle_group_selection(group_name);
                    state_.update_current_item_info();
                    DEBUG_STATE(state_.debug_logger, "GroupToggle", "Group toggle completed");
                }
            } else if (state_.is_modules_focused()) {
                // Select/deselect current module with conflict detection
                if (state_.two_section.modules_cursor < static_cast<int>(state_.modules.size())) {
                    const std::string& module_name = state_.modules[state_.two_section.modules_cursor].name;
                    DEBUG_STATE(state_.debug_logger, "ModuleToggle", "Toggling module: " + module_name);
                    state_.toggle_module_selection_with_conflicts(module_name);
                    state_.update_current_item_info();
                    DEBUG_STATE(state_.debug_logger, "ModuleToggle", "Module toggle completed");
                }
            }
            return true;
        }
        
        // Clear selection
        if (event == Event::Character('C') || event == Event::Character('c')) {
            state_.selection_manager.clear_all_selections();
            return true;
        }
        
        // Exit
        if (event == Event::Character('q') || event == Event::Escape || event == Event::CtrlC) {
            throw std::runtime_error("User requested exit");
        }
        
        // Linus风格DEBUG面板运行时控制
#ifdef SUNRAY_DEBUG_ENABLED
        if (event == Event::Character('D') || event == Event::Character('d')) {
            state_.debug_panel_enabled = !state_.debug_panel_enabled;
            DEBUG_STATE(state_.debug_logger, "DebugPanel", 
                        state_.debug_panel_enabled ? "enabled" : "disabled");
            return true;
        }
#endif
        
        // Mouse support - integrate with unified focus
        if (event.is_mouse()) {
            // Linus风格：记录鼠标事件用于调试
            const auto& mouse = event.mouse();
            if (mouse.motion == ftxui::Mouse::Moved) {
                DEBUG_EVENT(state_.debug_logger, "MOUSE", "Move to (" + std::to_string(mouse.x) + "," + std::to_string(mouse.y) + ")");
            } else if (mouse.motion == ftxui::Mouse::Pressed) {
                DEBUG_EVENT(state_.debug_logger, "MOUSE", "Press " + std::to_string(static_cast<int>(mouse.button)) + " at (" + std::to_string(mouse.x) + "," + std::to_string(mouse.y) + ")");
            } else if (mouse.motion == ftxui::Mouse::Released) {
                DEBUG_EVENT(state_.debug_logger, "MOUSE", "Release " + std::to_string(static_cast<int>(mouse.button)) + " at (" + std::to_string(mouse.x) + "," + std::to_string(mouse.y) + ")");
            }
            
            return handle_mouse_event(event.mouse());
        }
        
        return false;
    });
}

bool UIRenderer::handle_mouse_event(const ftxui::Mouse& mouse) {
    // Linus风格：明确区分鼠标事件类型，避免误判
    if (mouse.motion == ftxui::Mouse::Moved) {
        return handle_mouse_move(mouse);
    } else if (mouse.motion == ftxui::Mouse::Pressed && mouse.button == ftxui::Mouse::Left) {
        return handle_mouse_click(mouse);
    } else if (mouse.motion == ftxui::Mouse::Released) {
        // 鼠标释放事件 - 不处理但返回true表示已消耗
        return true;
    }
    
    // 其他鼠标事件（右键、中键等）- 不处理
    return false;
}

bool UIRenderer::handle_mouse_move(const ftxui::Mouse& mouse) {
    // Linus风格：鼠标移动不应该影响键盘焦点，也不应该阻塞事件循环
    // 返回true表示事件已处理，避免事件循环卡顿
    return true;
}

bool UIRenderer::handle_mouse_click(const ftxui::Mouse& mouse) {
    // Linus风格鼠标点击：设置统一焦点状态
    // 这里需要根据点击位置确定焦点目标
    
    // 简化实现：根据mouse.y位置推断焦点区域
    // 实际项目中应该使用coordinate_mapper
    int click_y = mouse.y;
    
    // 估算区域（基于典型TUI布局）
    if (click_y >= 3 && click_y <= 10) {
        // Groups区域
        state_.set_focus_target(FocusTarget::GROUPS);
        return true;
    } else if (click_y >= 12 && click_y <= 25) {
        // Modules区域  
        state_.set_focus_target(FocusTarget::MODULES);
        return true;
    } else if (click_y >= 30 && click_y <= 32) {
        // Build Button区域
        state_.set_focus_target(FocusTarget::BUILD_BUTTON);
        // 如果点击Build Button，设置构建请求标志
        state_.build_requested = true;
        return true;
    }
    
    return false;
}

#ifdef SUNRAY_DEBUG_ENABLED
Element UIRenderer::render_debug_panel() {
    auto recent = state_.debug_log.get_recent(8);  // 显示最近8条消息
    
    Elements debug_elements;
    debug_elements.push_back(text("=== DEBUG PANEL ===") | bold | color(Color::Red));
    
    if (recent.empty()) {
        debug_elements.push_back(text("(无调试消息)") | color(Color::GrayDark) | dim);
    } else {
        for (const auto& msg : recent) {
            debug_elements.push_back(text(msg) | color(Color::GrayLight) | dim);
        }
    }
    
    return vbox(debug_elements) | border;
}
#endif

#endif

} // namespace sunray_tui