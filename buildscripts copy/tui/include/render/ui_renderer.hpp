#pragma once

#include "ui_core.hpp"
#include "terminal_guard.hpp"
#include <functional>

#ifdef HAVE_FTXUI
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/animation.hpp"
#endif

namespace sunray_tui {

/**
 * UI渲染器
 * 负责：
 * - FTXUI界面渲染
 * - 鼠标和键盘事件处理
 * - 视觉效果和动画
 * - 组件布局管理
 */
class UIRenderer {
public:
    explicit UIRenderer(UIState& state);
    
    /**
     * 运行渲染器（带构建回调）
     * @param build_callback 构建按钮点击时的回调函数
     * @return 程序退出码
     */
    int run_with_build_callback(std::function<void()> build_callback);

private:
    UIState& state_;
    std::function<void()> build_callback_;
    
#ifdef HAVE_FTXUI
    /**
     * 创建主UI组件
     * @return FTXUI组件
     */
    ftxui::Component create_component();
    
    /**
     * 处理鼠标事件
     * @param mouse 鼠标事件信息
     * @return 是否处理了事件
     */
    bool handle_mouse_event(const ftxui::Mouse& mouse);
    
    /**
     * 处理鼠标移动
     * @param mouse 鼠标事件信息
     * @return 是否处理了事件
     */
    bool handle_mouse_move(const ftxui::Mouse& mouse);
    
    /**
     * 处理鼠标点击
     * @param mouse 鼠标事件信息
     * @return 是否处理了事件
     */
    bool handle_mouse_click(const ftxui::Mouse& mouse);
#endif
};

} // namespace sunray_tui