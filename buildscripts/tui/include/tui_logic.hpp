#pragma once

#include "tui_core.hpp"
#include "tui_render.hpp"

namespace sunray_tui {

/**
 * @brief UI逻辑控制器
 * 
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
     * @brief 运行主程序逻辑
     * @return 程序退出码
     */
    int run();
    
    /**
     * @brief 执行构建流程（TUI→CLI移交）
     */
    void execute_build();

private:
    UIState& state_;
    UIRenderer renderer_;
    static constexpr const char* LAST_SELECTION_FILE = "buildscripts/tui/build/.sunray_last_build_selection";
    
    /**
     * @brief 格式化CLI参数
     * @param modules 选中的模块列表
     * @return 格式化的参数字符串
     */
    std::string format_cli_arguments(const std::vector<std::string>& modules) const;
    
    /**
     * @brief 获取选中的模块列表
     * @return 模块名称列表
     */
    std::vector<std::string> get_selected_modules() const;
    
    /**
     * @brief 显示移交信息
     * @param message 要显示的消息
     */
    void display_transition_message(const std::string& message) const;
    
    /**
     * @brief 获取项目根目录路径
     * @return 项目根目录绝对路径
     */
    std::string get_project_root_dir() const;
    
    /**
     * @brief 保存当前选择到隐藏文件
     */
    void save_current_selection() const;
    
    /**
     * @brief 从隐藏文件加载上次选择（仅供内部使用）
     */
    void load_last_selection();
};

} // namespace sunray_tui