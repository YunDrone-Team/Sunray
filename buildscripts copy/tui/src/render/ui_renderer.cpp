#include "render/ui_renderer.hpp"
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
    TerminalGuard guard;
    
    auto component = create_component();
    
    auto screen = ScreenInteractive::Fullscreen();
    
    // Linus风格：当需要执行构建时，先退出TUI循环，再执行构建
    bool should_build = false;
    
    // 包装组件，定期检查构建请求（不在每个事件中检查）
    auto wrapped_component = CatchEvent(component, [&](Event event) -> bool {
        // 只在动画帧时检查构建请求，减少检查频率
        static int check_counter = 0;
        if (++check_counter % 10 == 0 && state_.build_requested) {
            state_.build_requested = false;
            should_build = true;
            screen.ExitLoopClosure()();  // 优雅退出TUI循环
            return true;
        }
        return false;
    });
    
    screen.Loop(wrapped_component);
    
    // FTXUI循环退出后，检查是否需要执行构建
    if (should_build && build_callback_) {
        build_callback_();  // TUI已退出，现在执行构建
    }
    
    return 0;
#endif
}

#ifdef HAVE_FTXUI
Component UIRenderer::create_component() {
    // 初始化渲染项  
    state_.update_render_items();
    
    auto renderer = Renderer([&] {
        Elements elements;
        elements.push_back(text("Sunray TUI - 模块管理工具") | bold | center);
        elements.push_back(separator());
        
        for (size_t i = 0; i < state_.render_items.size(); ++i) {
            const auto& item = state_.render_items[i];
            bool is_selected = (i == static_cast<size_t>(state_.current_selection));
            bool is_hovered = (i == static_cast<size_t>(state_.mouse_hover_index));
            
            Element content;
            switch (item.type) {
                case RenderItem::GROUP_HEADER: {
                    // 使用分离的组名和计数器创建两列布局
                    Element left_content;
                    if (item.identifier == "ungrouped") {
                        // ungrouped组使用斜体样式
                        left_content = text(item.text) | italic | dim;
                    } else {
                        left_content = text(item.text) | bold;
                    }
                    
                    Element right_content;
                    
                    // 计数器部分，如果有选中项则添加黄色背景
                    if (item.counter_text.find("选中") != std::string::npos) {
                        right_content = text(" " + item.counter_text + " ") | bgcolor(Color::Yellow) | color(Color::Black) | bold;
                    } else {
                        right_content = text(" " + item.counter_text + " ") | color(Color::GrayLight);
                    }
                    
                    // 组合左右两部分，左对齐组名，右对齐计数器
                    content = hbox({
                        left_content | flex,
                        right_content
                    });
                    
                    // 添加选择指示器或hover效果，但是让指示器覆盖整行
                    if (is_selected && !is_hovered) {
                        // 选中状态：蓝色背景指示器覆盖整行
                        content = hbox({
                            text("→") | color(Color::Blue) | bold,
                            text(" "),
                            content | flex
                        }) | bgcolor(Color::RGB(30, 30, 30));  // 深色背景
                    } else if (is_hovered) {
                        // Hover状态：灰色背景，蓝色指示器
                        content = hbox({
                            text("→") | color(Color::Blue) | bold,
                            text(" "),
                            content | flex
                        }) | bgcolor(Color::RGB(80, 80, 80));  // 更亮的灰色背景
                    } else {
                        // 正常状态：添加空格对齐
                        content = hbox({
                            text("  "),  // 两个空格对齐
                            content | flex
                        });
                    }
                    break;
                }
                case RenderItem::MODULE_ITEM: {
                    // 为模块项添加缩进
                    std::string indent_str(item.indent_level * 2, ' ');
                    Element text_content = text(indent_str + item.text);
                    
                    // 检查模块是否被选中
                    bool module_selected = state_.view.is_module_selected(item.identifier);
                    
                    // 根据模块状态设置颜色
                    if (item.is_disabled) {
                        text_content = text_content | color(Color::GrayDark) | dim;
                    } else if (module_selected) {
                        text_content = text_content | color(Color::White) | bold;
                    }
                    
                    content = text_content;
                    
                    // 添加选择指示器或hover效果，并为选中模块添加绿色背景
                    if (module_selected) {
                        // 选中的模块：绿色背景
                        content = hbox({
                            text("✓") | color(Color::White) | bold,
                            text(" "),
                            content | flex
                        }) | bgcolor(Color::Green) | color(Color::White);
                    } else if (is_selected && !is_hovered) {
                        // 当前选中但未选择的模块：蓝色指示器
                        content = hbox({
                            text("→") | color(Color::Blue) | bold,
                            text(" "),
                            content | flex
                        }) | bgcolor(Color::RGB(30, 30, 30));
                    } else if (is_hovered) {
                        // Hover状态：灰色背景，蓝色指示器
                        content = hbox({
                            text("→") | color(Color::Blue) | bold,
                            text(" "),
                            content | flex
                        }) | bgcolor(Color::RGB(80, 80, 80));
                    } else {
                        // 正常状态：添加空格对齐
                        content = hbox({
                            text("  "),  // 两个空格对齐
                            content | flex
                        });
                    }
                    break;
                }
                case RenderItem::SEPARATOR:
                    content = separator();
                    break;
                case RenderItem::INFO_TEXT:
                    content = text(item.text) | dim;
                    break;
                default:
                    content = text(item.text);
                    break;
            }
            
            elements.push_back(content);
        }
        
        // 显示当前选中项的信息 - 固定4行高度
        elements.push_back(separator());
        elements.push_back(text("描述: " + (state_.current_item_description.empty() ? "NULL" : state_.current_item_description)) | bold);
        
        // 收集详细信息行，确保总共4行
        std::vector<Element> detail_lines;
        if (!state_.current_item_details.empty()) {
            // 按行分割详细信息，特殊处理冲突行
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
                
                // 检查是否是冲突行且包含非NULL内容
                if (line.find("冲突: ") == 0 && line.find("NULL") == std::string::npos) {
                    // 冲突闪烁效果：奇数次闪烁时使用不同颜色
                    if (state_.conflict_flash_active && (state_.conflict_flash_count % 2 == 1)) {
                        detail_lines.push_back(text(line) | bgcolor(Color::Yellow) | color(Color::Black) | bold | blink);
                    } else {
                        detail_lines.push_back(text(line) | bgcolor(Color::Red) | color(Color::White) | bold);
                    }
                } else {
                    detail_lines.push_back(text(line) | dim);
                }
                
                if (end == std::string::npos) break;
            }
        }
        
        // 确保总共有4行（包括描述行，还需要3行详细信息）
        while (detail_lines.size() < 3) {
            detail_lines.push_back(text("") | dim);  // 添加空行
        }
        
        // 只显示前3行详细信息（加上描述行总共4行）
        for (size_t i = 0; i < std::min(detail_lines.size(), size_t(3)); ++i) {
            elements.push_back(detail_lines[i]);
        }
        
        elements.push_back(separator());
        
        // 记录编译按钮的屏幕位置（精确计算）
        state_.build_button_screen_y = 1 + static_cast<int>(elements.size());
        
        // 添加编译按钮区域 - 变大并居中，支持hover效果
        Element build_button_content;
        if (state_.build_button_focused) {
            build_button_content = text("【 开始编译构建 】") | bold | bgcolor(Color::Blue) | color(Color::White);
        } else if (state_.build_button_hovered) {
            build_button_content = text("【 开始编译构建 】") | bold | color(Color::Blue) | bgcolor(Color::RGB(80, 80, 80));
        } else {
            build_button_content = text("【 开始编译构建 】") | bold | color(Color::Blue);
        }
        elements.push_back(hbox({
            filler(),
            build_button_content,
            filler()
        }) | center);
        
        elements.push_back(separator());
        
        // 按键提示 - 表格布局
        elements.push_back(hbox({
            vbox({
                text("上下导航") | color(Color::GrayLight),
                text("展开/折叠") | color(Color::GrayLight),
                text("模块选择") | color(Color::GrayLight),
                text("焦点切换") | color(Color::GrayLight),
                text("清除选择") | color(Color::GrayLight),
                text("退出程序") | color(Color::GrayLight),
                text("鼠标交互") | color(Color::GrayLight)
            }) | flex,
            text("  ") | color(Color::Default),  // 分隔列
            vbox({
                text("↑↓ 方向键") | color(Color::Cyan),
                text("空格键") | color(Color::Cyan),
                text("回车键") | color(Color::Cyan),
                text("Tab键") | color(Color::Cyan),
                text("C键") | color(Color::Cyan),
                text("q/Esc键") | color(Color::Cyan),
                text("悬停+点击") | color(Color::Cyan)
            }) | flex
        }));
        
        // 渲染结束后重置动画状态
        if (state_.animation_in_progress) {
            state_.animation_in_progress = false;
            state_.previous_selection = -1;
        }
        
        // 更新冲突闪烁状态
        if (state_.conflict_flash_active) {
            state_.update_conflict_flash();
            animation::RequestAnimationFrame();
        }
        
        // 主界面内容
        Element main_content = vbox(elements) | border;
        
        // 重建统一的坐标映射
        int dialog_ok_y = state_.show_build_dialog ? 15 : -1;
        state_.coordinate_mapper.rebuild_mapping(
            state_.render_items,
            state_.build_button_screen_y,
            state_.show_build_dialog,
            dialog_ok_y
        );
        
        // 验证坐标映射的准确性
        if (!state_.coordinate_mapper.validate_mapping()) {
            state_.coordinate_mapper.set_base_offset(0);
            state_.coordinate_mapper.rebuild_mapping(
                state_.render_items,
                state_.build_button_screen_y,
                state_.show_build_dialog,
                dialog_ok_y
            );
        }
        
        // 如果显示构建对话框，则叠加模态对话框
        if (state_.show_build_dialog) {
            // 获取选中的模块列表
            std::vector<std::string> selected_modules;
            for (const auto& module_name : state_.view.selected_modules) {
                selected_modules.push_back(module_name);
            }
            
            // 构建模块列表显示
            Elements module_list;
            if (selected_modules.empty()) {
                module_list.push_back(text("没有选择任何模块") | color(Color::Red));
            } else {
                module_list.push_back(text("待编译模块:") | bold);
                for (const auto& module : selected_modules) {
                    module_list.push_back(text("  • " + module) | color(Color::Green));
                }
            }
            
            // 创建更大的对话框，不透视背景
            Element dialog_content = vbox({
                text("构建确认") | bold | center,
                separator(),
                vbox(module_list),
                separator(),
                text("[  确认构建  ]") | bold | center | bgcolor(Color::Blue) | color(Color::White)
            }) | border | bgcolor(Color::RGB(40, 40, 40)) | color(Color::White) | 
                size(WIDTH, GREATER_THAN, 40) | size(HEIGHT, GREATER_THAN, 10);
            
            // 完全遮盖背景，不透视
            return dbox({
                text("") | size(WIDTH, EQUAL, 200) | size(HEIGHT, EQUAL, 50) | bgcolor(Color::RGB(20, 20, 20)),  // 黑色遮罩
                dialog_content | center  // 对话框居中显示
            });
        }
        
        return main_content;
    });
    
    return CatchEvent(renderer, [this](Event event) {
        // 如果对话框显示，处理确认对话框
        if (state_.show_build_dialog) {
            if (event == Event::Return) {
                state_.close_build_dialog();
                // 设置构建请求标志，让主循环处理
                state_.build_requested = true;
                return true;
            }
            if (event == Event::Escape) {
                state_.close_build_dialog();
                return true;
            }
            return true; // 阻止其他键盘操作
        }
        
        // Tab键焦点切换
        if (event == Event::Tab) {
            state_.handle_tab_focus();
            return true;
        }
        
        if (event == Event::ArrowUp) {
            if (!state_.build_button_focused) {
                state_.previous_selection = state_.current_selection;
                state_.move_selection_up();
                state_.update_current_item_info();
                state_.animation_in_progress = true;
                animation::RequestAnimationFrame();
            }
            return true;
        }
        if (event == Event::ArrowDown) {
            if (!state_.build_button_focused) {
                state_.previous_selection = state_.current_selection;
                state_.move_selection_down();
                state_.update_current_item_info();
                state_.animation_in_progress = true;
                animation::RequestAnimationFrame();
            }
            return true;
        }
        if (event == Event::Character(' ')) {
            if (!state_.build_button_focused) {
                state_.handle_expand_toggle();
            }
            return true;
        }
        if (event == Event::Return) {
            if (state_.build_button_focused) {
                state_.handle_build_button();
            } else {
                state_.handle_selection();
            }
            return true;
        }
        if (event == Event::Character('C') || event == Event::Character('c')) {
            state_.view.clear_selection();
            state_.update_render_items();
            return true;
        }
        if (event == Event::Character('q') || event == Event::Escape || event == Event::CtrlC) {
            throw std::runtime_error("User requested exit");
        }
        if (event == Event::Character('D')) {
            // 调试模式：打印坐标映射
            state_.coordinate_mapper.debug_print_mapping();
            return true;
        }
        
        // 鼠标支持
        if (event.is_mouse()) {
            return handle_mouse_event(event.mouse());
        }
        
        return false;
    });
}

bool UIRenderer::handle_mouse_event(const Mouse& mouse) {
    if (mouse.motion == Mouse::Moved) {
        return handle_mouse_move(mouse);
    } else if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
        return handle_mouse_click(mouse);
    }
    return false;
}

bool UIRenderer::handle_mouse_move(const Mouse& mouse) {
    bool state_changed = false;
    
    // 使用统一坐标映射器获取元素信息
    ElementInfo element = state_.coordinate_mapper.get_element_at(mouse.y);
    
    // 如果精确匹配失败，尝试查找附近的可交互元素
    if (element.type == ElementType::UNKNOWN) {
        element = state_.coordinate_mapper.find_nearest_interactive(mouse.y, 0);
    }
    
    // 更新编译按钮hover状态
    bool button_hovered = (element.type == ElementType::BUILD_BUTTON);
    if (button_hovered != state_.build_button_hovered) {
        state_.build_button_hovered = button_hovered;
        state_changed = true;
    }
    
    // 更新render_items的hover状态
    int new_hover_index = element.render_item_index;
    if (new_hover_index != state_.mouse_hover_index) {
        state_.mouse_hover_index = new_hover_index;
        state_changed = true;
    }
    
    // 移动蓝色选择块到可交互元素（非按钮）
    if (element.render_item_index >= 0 && 
        element.render_item_index != state_.current_selection &&
        !button_hovered &&
        (element.type == ElementType::GROUP_HEADER || element.type == ElementType::MODULE_ITEM)) {
        
        state_.previous_selection = state_.current_selection;
        state_.current_selection = element.render_item_index;
        state_.animation_in_progress = true;
        state_.update_current_item_info();
        
        animation::RequestAnimationFrame();
        
        state_changed = true;
    }
    
    return state_changed;
}

bool UIRenderer::handle_mouse_click(const Mouse& mouse) {
    // 使用统一坐标映射器获取元素信息
    ElementInfo element = state_.coordinate_mapper.get_element_at(mouse.y);
    
    // 首先检查是否点击了禁用的模块
    if (element.type == ElementType::MODULE_ITEM && element.render_item_index >= 0) {
        const RenderItem& item = state_.render_items[element.render_item_index];
        if (item.is_disabled && item.is_selectable) {
            // 点击了禁用的模块，移动选择位置并触发冲突闪烁
            state_.current_selection = element.render_item_index;
            state_.update_current_item_info();
            state_.trigger_conflict_flash();
            return true;
        }
    }
    
    // 如果不是禁用模块且精确匹配失败，尝试查找附近的可点击元素
    if (element.type == ElementType::UNKNOWN || !element.is_clickable) {
        element = state_.coordinate_mapper.find_nearest_interactive(mouse.y, 1);
    }
    
    if (!element.is_clickable) {
        return false;
    }
    
    // 处理不同类型的点击
    switch (element.type) {
        case ElementType::BUILD_BUTTON:
            state_.handle_build_button();
            return true;
            
        case ElementType::DIALOG_OK_BUTTON:
            state_.close_build_dialog();
            // 设置构建请求标志，让主循环处理
            state_.build_requested = true;
            return true;
            
        case ElementType::GROUP_HEADER:
        case ElementType::MODULE_ITEM:
            // 移动选择位置（如果需要）
            if (element.render_item_index != state_.current_selection) {
                state_.current_selection = element.render_item_index;
                state_.update_current_item_info();
            }
            
            // 获取render_item并处理点击
            if (element.render_item_index >= 0 && 
                element.render_item_index < static_cast<int>(state_.render_items.size())) {
                const RenderItem& item = state_.render_items[element.render_item_index];
                
                // 禁用的模块不响应点击
                if (item.is_disabled && item.is_selectable) {
                    return false;
                }
                
                // 处理展开/折叠或选择
                if (item.is_collapsible) {
                    return state_.handle_expand_toggle();
                } else if (item.is_selectable) {
                    return state_.handle_selection();
                }
            }
            break;
            
        default:
            break;
    }
    
    return false;
}
#endif

} // namespace sunray_tui