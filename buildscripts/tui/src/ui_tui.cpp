#include "include/ui_core.hpp"
#include "include/config_loader.hpp"
#include "include/terminal_guard.hpp"
#include "include/screen_coordinate_mapper.hpp"

#ifdef HAVE_FTXUI
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/animation.hpp"  // 动画支持
#endif

#include <iostream>
#include <stdexcept>

using namespace sunray_tui;

void show_help() {
    std::cout << "Sunray TUI - 交互式模块管理工具\n\n";
    std::cout << "用法: sunray_tui [选项]\n\n";
    std::cout << "选项:\n";
    std::cout << "  --help, -h     显示此帮助信息\n";
    std::cout << "  --version, -v  显示版本信息\n\n";
    std::cout << "交互控制:\n";
    std::cout << "  上下键         上下导航\n";
    std::cout << "  空格           选择模块/展开组\n";
    std::cout << "  C              清除所有选择\n";
    std::cout << "  q/Esc          退出程序\n\n";
}

void show_version() {
    std::cout << "Sunray TUI v1.0\n";
    std::cout << "基础交互式模块管理工具\n";
}

#ifdef HAVE_FTXUI
using namespace ftxui;

class BasicRenderer {
public:
    explicit BasicRenderer(UIState& state) : state_(state) {}
    
    int run() {
        TerminalGuard guard;
        
        auto component = create_component();
        
        try {
            auto screen = ScreenInteractive::Fullscreen();
            screen.Loop(component);
        } catch (const std::runtime_error& e) {
            std::string msg = e.what();
            if (msg == "User requested exit") {
                return 0;  // 正常退出
            }
            std::cerr << "UI错误: " << e.what() << std::endl;
            return 1;
        } catch (const std::exception& e) {
            std::cerr << "UI错误: " << e.what() << std::endl;
            return 1;
        }
        
        return 0;
    }

private:
    UIState& state_;
    
    Component create_component() {
        // 初始化渲染项  
        state_.update_render_items();
        
        auto renderer = Renderer([this] {
            // 移除重复的update_render_items()调用 - 只在数据变化时更新
            
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
                        if (item.has_selected_items) {
                            right_content = text(item.counter_text) | bold | bgcolor(Color::Yellow) | color(Color::Black);
                        } else {
                            right_content = text(item.counter_text) | bold;
                        }
                        
                        // 选择指示器
                        Element indicator;
                        if (is_selected) {
                            indicator = text("█") | color(Color::RGB(35, 76, 138));  // 天蓝色 #234C8A
                        } else {
                            indicator = text(" ");
                        }
                        
                        // 使用hbox创建三列布局：左对齐组名 + 中间填充 + 右对齐计数器 + 指示器
                        content = hbox({
                            left_content,
                            filler(),  // 填充中间空白
                            right_content,
                            text(" "),  // 小间距
                            indicator   // 选择指示器
                        });
                        
                        // 检查组是否被选中
                        if (state_.view.is_group_selected(item.identifier)) {
                            content = content | bgcolor(Color::Blue) | color(Color::White);
                        }
                        // 整行hover效果 - 应用到完整内容上，保留指示块
                        else if (is_hovered && !is_selected) {
                            content = content | bgcolor(Color::RGB(80, 80, 80));  // 整行hover背景
                        }
                        
                        // 动画结束后重置状态
                        if (state_.animation_in_progress && i == static_cast<size_t>(state_.current_selection)) {
                            // 为当前选择项添加微妙的动画效果
                            #ifdef HAVE_FTXUI
                            ftxui::animation::RequestAnimationFrame();
                            #endif
                        }
                        break;
                    }
                    case RenderItem::MODULE_ITEM: {
                        Element main_content = text(std::string(item.indent_level * 2, ' ') + item.text);
                        
                        // 添加选择指示器
                        Element indicator;
                        if (is_selected) {
                            indicator = text("█") | color(Color::RGB(35, 76, 138));  // 天蓝色 #234C8A
                        } else {
                            indicator = text(" ");
                        }
                        
                        content = hbox({
                            main_content,
                            filler(),
                            text(" "),
                            indicator
                        });
                        
                        // 检查模块状态并应用样式
                        if (item.is_disabled) {
                            // 禁用状态：灰色文本，暗背景
                            content = content | color(Color::GrayDark) | dim;
                        }
                        else if (state_.is_module_selected_new(item.identifier)) {
                            // 选中状态：绿色背景
                            content = content | bgcolor(Color::Green) | color(Color::Black);
                        }
                        // 整行hover效果 - 应用到完整内容上，保留指示块（但不应用于禁用项和已选中项）
                        else if (is_hovered && !is_selected && !item.is_disabled) {
                            content = content | bgcolor(Color::RGB(80, 80, 80));  // 整行hover背景
                        }
                        
                        // 动画结束后重置状态
                        if (state_.animation_in_progress && i == static_cast<size_t>(state_.current_selection)) {
                            // 为当前选择项添加微妙的动画效果
                            #ifdef HAVE_FTXUI
                            ftxui::animation::RequestAnimationFrame();
                            #endif
                        }
                        break;
                    }
                    case RenderItem::SEPARATOR:
                        content = separator();
                        break;
                    case RenderItem::INFO_TEXT: {
                        Element main_content = text(item.text) | dim;
                        Element indicator;
                        if (is_selected) {
                            indicator = text("█") | color(Color::RGB(35, 76, 138));  // 天蓝色 #234C8A
                        } else {
                            indicator = text(" ");
                        }
                        
                        content = hbox({
                            main_content,
                            filler(),
                            text(" "),
                            indicator
                        });
                        break;
                    }
                }
                
                // 对于SEPARATOR类型，仍然保持原样
                // 其他类型已经在上面添加了指示器，不再需要额外的高亮效果
                
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
            // 标题(1) + 分隔符(1) + 边框(1) + 当前elements数量 = 按钮位置
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
                // 简单的动画结束逻辑：下一帧重置状态
                state_.animation_in_progress = false;
                state_.previous_selection = -1;
            }
            
            // 更新冲突闪烁状态
            if (state_.conflict_flash_active) {
                state_.update_conflict_flash();
                #ifdef HAVE_FTXUI
                ftxui::animation::RequestAnimationFrame();
                #endif
            }
            
            // 主界面内容
            Element main_content = vbox(elements) | border;
            
            // 重建统一的坐标映射 - Linus风格：在渲染完成后立即同步
            int dialog_ok_y = state_.show_build_dialog ? 15 : -1;  // 对话框OK按钮的大概位置
            state_.coordinate_mapper.rebuild_mapping(
                state_.render_items,
                state_.build_button_screen_y,
                state_.show_build_dialog,
                dialog_ok_y
            );
            
            // 验证坐标映射的准确性
            if (!state_.coordinate_mapper.validate_mapping()) {
                // 如果映射无效，尝试重置偏移量
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
                Element dialog_content = vbox({
                    text("--> 开始构建任务...") | bold | center,
                    separator(),
                    text("[  OK  ]") | bold | center | bgcolor(Color::Blue) | color(Color::White)
                }) | border | size(WIDTH, EQUAL, 50) | size(HEIGHT, EQUAL, 8);
                
                return dbox({
                    main_content | dim,  // 主界面变暗
                    dialog_content | center  // 对话框居中显示
                });
            }
            
            return main_content;
        });
        
        return CatchEvent(renderer, [this](Event event) {
            // 如果对话框显示，只处理关闭对话框的操作
            if (state_.show_build_dialog) {
                if (event == Event::Return) {
                    state_.close_build_dialog();
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
                    state_.update_current_item_info();  // 确保信息更新
                    state_.animation_in_progress = true;
                    #ifdef HAVE_FTXUI
                    ftxui::animation::RequestAnimationFrame();
                    #endif
                }
                return true;
            }
            if (event == Event::ArrowDown) {
                if (!state_.build_button_focused) {
                    state_.previous_selection = state_.current_selection;
                    state_.move_selection_down();
                    state_.update_current_item_info();  // 确保信息更新
                    state_.animation_in_progress = true;
                    #ifdef HAVE_FTXUI
                    ftxui::animation::RequestAnimationFrame();
                    #endif
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
                state_.update_render_items();  // 数据变化，需要更新
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
            
            // === 鼠标支持 - 复用现有键盘逻辑 ===
            if (event.is_mouse()) {
                return handle_mouse_event(event.mouse());
            }
            
            return false;
        });
    }

private:
    // 将屏幕y坐标映射到render_items索引
    int screen_y_to_item_index(int screen_y) const {
        // FTXUI布局分析：
        // 0: border顶部
        // 1: 标题 "Sunray TUI - 模块管理工具"  
        // 2: 分隔线
        // 3: 第一个内容项 (control组)
        // 4: 第二个内容项 (sunray_common模块)
        // ...
        
        // 计算内容区起始位置：border(1) + 标题(1) + 分隔线(1) = 3
        int content_start_y = 3;
        int item_y = screen_y - content_start_y;
        
        if (item_y >= 0 && item_y < static_cast<int>(state_.render_items.size())) {
            return item_y;
        }
        return -1; // 无效位置
    }
    
    // 统一的鼠标事件处理 - Linus风格：简单、可靠、统一
    bool handle_mouse_event(const Mouse& mouse) {
        if (mouse.motion == Mouse::Moved) {
            return handle_mouse_move(mouse);
        } else if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
            return handle_mouse_click(mouse);
        }
        return false;
    }
    
    // 处理鼠标移动事件
    bool handle_mouse_move(const Mouse& mouse) {
        bool state_changed = false;
        
        // 使用统一坐标映射器获取元素信息
        ElementInfo element = state_.coordinate_mapper.get_element_at(mouse.y);
        
        // 如果精确匹配失败，尝试查找附近的可交互元素
        if (element.type == ElementType::UNKNOWN) {
            element = state_.coordinate_mapper.find_nearest_interactive(mouse.y, 0);
        }
        
        // 调试模式：显示鼠标位置信息
        #ifdef DEBUG_MOUSE_COORDINATES
        state_.coordinate_mapper.debug_mouse_position(mouse.y);
        #endif
        
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
            
            #ifdef HAVE_FTXUI
            ftxui::animation::RequestAnimationFrame();
            #endif
            
            state_changed = true;
        }
        
        return state_changed;
    }
    
    // 处理鼠标点击事件
    bool handle_mouse_click(const Mouse& mouse) {
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
                return true;  // 消费掉这个点击事件，不再查找附近元素
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
};

#else
// 非FTXUI版本
class BasicRenderer {
public:
    explicit BasicRenderer(UIState& state) : state_(state) {}
    
    int run() {
        TerminalGuard guard;
        
        std::cout << "Sunray TUI - 模块管理工具\n";
        std::cout << "注意: 未编译FTXUI支持，使用简化界面\n\n";
        
        state_.update_render_items();
        for (const auto& item : state_.render_items) {
            switch (item.type) {
                case RenderItem::GROUP_HEADER:
                    std::cout << item.text << "\n";
                    break;
                case RenderItem::MODULE_ITEM:
                    std::cout << std::string(item.indent_level * 2, ' ') << item.text << "\n";
                    break;
                case RenderItem::SEPARATOR:
                    std::cout << "---\n";
                    break;
                case RenderItem::INFO_TEXT:
                    std::cout << item.text << "\n";
                    break;
            }
        }
        
        std::cout << "\n按任意键退出...";
        std::cin.get();
        return 0;
    }
    
private:
    UIState& state_;
};
#endif

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            show_help();
            return 0;
        }
        if (arg == "--version" || arg == "-v") {
            show_version();
            return 0;
        }
    }
    
    const std::string config_path = "buildscripts/modules.yaml";
    
    try {
        auto config = ConfigData::load_from_file(config_path);
        if (!config) {
            std::cerr << "无法加载配置文件: " << config_path << std::endl;
            return 1;
        }
        
        UIState ui_state = config->create_ui_state();
        
        BasicRenderer renderer(ui_state);
        return renderer.run();
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        TerminalGuard::restore_terminal();
        return 1;
    } catch (...) {
        std::cerr << "未知错误" << std::endl;
        TerminalGuard::restore_terminal();
        return 1;
    }
}