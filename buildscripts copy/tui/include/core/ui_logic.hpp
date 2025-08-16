#pragma once

#include "ui_core.hpp"
#include "terminal_guard.hpp"
#include "render/ui_renderer.hpp"

namespace sunray_tui {

/**
 * UI逻辑控制器
 * 负责：
 * - 事件处理和响应
 * - 业务逻辑流程控制
 * - TUI→CLI移交机制
 * - 状态管理协调
 */
class UILogic {
public:
    explicit UILogic(UIState& state);
    
    /**
     * 运行主程序逻辑
     * @return 程序退出码
     */
    int run();

private:
    UIState& state_;
    UIRenderer renderer_;
    
    /**
     * 格式化CLI参数
     * @param modules 选中的模块列表
     * @return 格式化的参数字符串
     */
    std::string format_cli_arguments(const std::vector<std::string>& modules) const;
    
    /**
     * 获取选中的模块列表
     * @return 模块名称列表
     */
    std::vector<std::string> get_selected_modules() const;
    
    /**
     * 显示移交信息
     * @param message 要显示的消息
     */
    void display_transition_message(const std::string& message) const;
    
    /**
     * 获取项目根目录路径
     * @return 项目根目录绝对路径
     */
    std::string get_project_root_dir() const;
    
    /**
     * 通过TTY直接清理终端状态（不污染stdout）
     */
    void cleanup_terminal_via_tty() const;
    
    /**
     * 通过TTY恢复终端状态（execl失败时）
     */
    void restore_terminal_via_tty() const;
    
    /**
     * 执行构建流程（TUI→CLI移交）
     */
    void execute_build();
};

} // namespace sunray_tui