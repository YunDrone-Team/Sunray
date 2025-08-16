#pragma once

#include "ui_core.hpp"
#include "terminal_guard.hpp"

#ifdef HAVE_FTXUI
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/event.hpp"
#endif

namespace sunray_tui {

#ifdef HAVE_FTXUI
using namespace ftxui;

class InteractiveRenderer {
public:
    explicit InteractiveRenderer(UIState& state) : state_(state), should_exit_(false) {}
    
    // 主运行函数
    int run() {
        TerminalGuard guard;  // RAII 终端保护
        
        auto component = create_main_component();
        
        try {
            auto screen = ScreenInteractive::Fullscreen();
            screen.Loop(component);
        } catch (const std::exception& e) {
            std::cerr << "UI渲染错误: " << e.what() << std::endl;
            return 1;
        }
        
        return 0;
    }

private:
    UIState& state_;
    std::string search_input_;
    bool should_exit_;
    
    Component create_main_component() {
        // 搜索输入框
        auto search_component = Input(&search_input_, "🔍 搜索模块/组...");
        
        // 标签页容器
        auto tab_container = create_tab_container();
        
        // 状态栏
        auto status_component = create_status_component();
        
        // 主容器
        auto main_container = Container::Vertical({
            search_component,
            tab_container,
            status_component
        });
        
        // 主渲染器
        auto renderer = Renderer(main_container, [=] {
            return vbox({
                // 标题栏
                hbox({
                    text("🚀 ") | color(Color::Cyan),
                    text("Sunray TUI") | bold | color(Color::Green),
                    text(" - 模块管理器 ") | color(Color::White),
                    text("v2.0") | dim | color(Color::Yellow)
                }) | center,
                separator(),
                
                // 搜索框
                hbox({
                    text("搜索: ") | color(Color::Cyan),
                    search_component->Render() | border | flex
                }) | size(HEIGHT, EQUAL, 3),
                
                // 主内容区
                tab_container->Render() | flex,
                
                // 状态栏
                separator(),
                status_component->Render()
            });
        });
        
        // 事件处理 - 使用退出条件
        auto main_component_with_exit = Renderer(renderer, [renderer, this] {
            return renderer->Render();
        });
        
        return CatchEvent(main_component_with_exit, [this](Event event) {
            if (handle_global_events(event)) {
                return true;
            }
            return false;
        });
    }
    
    Component create_tab_container() {
        // 创建标签页标题
        std::vector<std::string> tab_titles = {"📦 模块选择", "📁 组选择", "🔧 配置构建"};
        auto tab_toggle = Toggle(&tab_titles, &state_.view.active_tab);
        
        // 创建标签页内容
        auto tab_content = Container::Tab({
            create_modules_tab_content(),
            create_groups_tab_content(), 
            create_build_config_tab_content()
        }, &state_.view.active_tab);
        
        // 组合标签页
        auto container = Container::Vertical({
            tab_toggle,
            tab_content
        });
        
        return Renderer(container, [=] {
            return vbox({
                // 标签页标题
                tab_toggle->Render() | center,
                separator(),
                
                // 标签页内容
                tab_content->Render() | flex | border
            });
        });
    }
    
    Component create_modules_tab_content() {
        return Renderer([this] {
            Elements elements;
            
            // 确保是模块标签页
            if (state_.view.active_tab == ViewState::MODULES_TAB) {
                state_.update_render_items();
                
                for (size_t i = 0; i < state_.render_items.size(); ++i) {
                    const auto& item = state_.render_items[i];
                    bool is_selected = (i == static_cast<size_t>(state_.current_selection));
                    auto element = render_item_with_colors(item, is_selected);
                    elements.push_back(element);
                }
            }
            
            return vbox(elements) | vscroll_indicator | yframe;
        });
    }
    
    Component create_groups_tab_content() {
        return Renderer([this] {
            Elements elements;
            
            if (state_.view.active_tab == ViewState::GROUPS_TAB) {
                state_.update_render_items();
                
                for (size_t i = 0; i < state_.render_items.size(); ++i) {
                    const auto& item = state_.render_items[i];
                    bool is_selected = (i == static_cast<size_t>(state_.current_selection));
                    auto element = render_item_with_colors(item, is_selected);
                    elements.push_back(element);
                }
            }
            
            return vbox(elements) | vscroll_indicator | yframe;
        });
    }
    
    Component create_build_config_tab_content() {
        return Renderer([this] {
            Elements elements;
            
            if (state_.view.active_tab == ViewState::BUILD_CONFIG_TAB) {
                state_.update_render_items();
                
                for (size_t i = 0; i < state_.render_items.size(); ++i) {
                    const auto& item = state_.render_items[i];
                    bool is_selected = (i == static_cast<size_t>(state_.current_selection));
                    auto element = render_item_with_colors(item, is_selected);
                    elements.push_back(element);
                }
            }
            
            return vbox(elements) | vscroll_indicator | yframe;
        });
    }
    
    Element render_item_with_colors(const RenderItem& item, bool is_selected) {
        Element content;
        
        // 根据类型渲染并添加颜色
        switch (item.type) {
            case RenderItem::GROUP_HEADER: {
                auto base = text(item.text) | bold;
                if (is_selected) {
                    content = base | inverted | color(Color::Green);
                } else {
                    // 检查是否包含选中标记来决定颜色
                    if (item.text.find("☑") != std::string::npos) {
                        content = base | color(Color::Green);  // 已选择的组用绿色
                    } else {
                        content = base | color(Color::Cyan);   // 未选择的组用青色
                    }
                }
                break;
            }
                
            case RenderItem::MODULE_ITEM: {
                std::string indent(item.indent_level * 2, ' ');
                auto base = text(indent + item.text);
                
                if (is_selected) {
                    content = base | inverted | color(Color::Blue);
                } else {
                    // 检查是否选中来决定颜色
                    if (item.text.find("☑") != std::string::npos) {
                        content = base | color(Color::Green);   // 选中的模块用绿色
                    } else {
                        content = base | color(Color::White);   // 未选中的模块用白色
                    }
                }
                break;
            }
                
            case RenderItem::SEPARATOR:
                content = separator() | color(Color::GrayDark);
                break;
                
            case RenderItem::INFO_TEXT: {
                auto base = text(item.text);
                // 根据文本内容决定颜色
                if (item.text.find("⚠️") != std::string::npos) {
                    content = base | color(Color::Yellow);      // 警告信息
                } else if (item.text.find("✅") != std::string::npos) {
                    content = base | color(Color::Green);       // 成功信息  
                } else if (item.text.find("🔧") != std::string::npos || 
                          item.text.find("🚀") != std::string::npos) {
                    content = base | color(Color::Cyan);        // 配置信息
                } else if (item.text.find("📦") != std::string::npos || 
                          item.text.find("📁") != std::string::npos) {
                    content = base | color(Color::Blue) | bold;  // 标题信息
                } else {
                    content = base | dim | color(Color::GrayLight); // 一般信息
                }
                break;
            }
        }
        
        return content;
    }
    
    Component create_status_component() {
        return Renderer([this] {
            // 构建快捷键提示，根据当前标签页显示不同内容
            Elements shortcuts;
            
            // 通用快捷键
            shortcuts.push_back(text("←→: 切换标签") | color(Color::Yellow));
            shortcuts.push_back(text(" | ") | dim);
            shortcuts.push_back(text("↑↓: 导航") | color(Color::Yellow));
            shortcuts.push_back(text(" | ") | dim);
            shortcuts.push_back(text("Space: 选择/展开") | color(Color::Yellow));
            shortcuts.push_back(text(" | ") | dim);
            
            // 标签页特定快捷键
            if (state_.view.active_tab == ViewState::BUILD_CONFIG_TAB) {
                shortcuts.push_back(text("B: 开始构建") | color(Color::Green));
                shortcuts.push_back(text(" | ") | dim);
            }
            
            shortcuts.push_back(text("C: 清除选择") | color(Color::Red));
            shortcuts.push_back(text(" | ") | dim);
            shortcuts.push_back(text("q: 退出") | color(Color::Red));
            
            return vbox({
                // 状态信息行
                hbox({
                    text("当前页面: ") | color(Color::Cyan),
                    text(state_.view.get_tab_name()) | bold | color(Color::White),
                    text("  |  ") | dim,
                    text("已选择: ") | color(Color::Cyan), 
                    text(std::to_string(state_.view.selected_count())) | bold | color(Color::Green),
                    text(" 个模块") | color(Color::White),
                    text("  |  ") | dim,
                    text("总计: ") | color(Color::Cyan),
                    text(std::to_string(state_.modules.size())) | bold | color(Color::White),
                    text(" 个模块") | color(Color::White)
                }),
                
                // 快捷键提示行
                hbox(shortcuts)
            });
        });
    }
    
    bool handle_global_events(Event event) {
        // 搜索更新
        if (search_input_ != state_.view.search_filter) {
            state_.handle_search_update(search_input_);
        }
        
        // 标签页切换 - 左右箭头
        if (event == Event::ArrowLeft) {
            state_.view.prev_tab();
            state_.current_selection = 0;  // 重置选择
            state_.update_render_items();
            return true;
        }
        
        if (event == Event::ArrowRight) {
            state_.view.next_tab(); 
            state_.current_selection = 0;  // 重置选择
            state_.update_render_items();
            return true;
        }
        
        // 键盘导航
        if (event == Event::ArrowUp) {
            state_.move_selection_up();
            return true;
        }
        
        if (event == Event::ArrowDown) {
            state_.move_selection_down();
            return true;
        }
        
        // 选择/展开操作
        if (event == Event::Character(' ') || event == Event::Return) {
            return state_.handle_selection_toggle();
        }
        
        // 构建配置页面的特殊功能
        if (state_.view.active_tab == ViewState::BUILD_CONFIG_TAB) {
            if (event == Event::Character('B') || event == Event::Character('b')) {
                // TODO: 实现构建功能
                return true;
            }
            
            if (event == Event::Character('D') || event == Event::Character('d')) {
                // TODO: 显示依赖关系
                return true;
            }
        }
        
        // 退出操作
        if (event == Event::Character('q') || event == Event::Escape) {
            return true;
        }
        
        if (event == Event::CtrlC) {
            return true;
        }
        
        // 清除所有选择
        if (event == Event::Character('C')) {
            state_.view.clear_selection();
            state_.update_render_items();
            return true;
        }
        
        // Tab键也可以切换标签页
        if (event == Event::Tab) {
            state_.view.next_tab();
            state_.current_selection = 0;
            state_.update_render_items();
            return true;
        }
        
        return false;
    }
};

#else
// 简化的非FTXUI版本
class InteractiveRenderer {
public:
    explicit InteractiveRenderer(UIState& state) : state_(state) {}
    
    int run() {
        TerminalGuard guard;
        
        std::cout << "Sunray TUI - 模块管理器\n";
        std::cout << "注意: 未编译FTXUI支持，使用简化界面\n\n";
        
        // 显示所有项目
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

} // namespace sunray_tui