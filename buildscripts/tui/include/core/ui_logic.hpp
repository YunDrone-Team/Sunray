#pragma once

#include <string>
#include <vector>
#include "../ui_core.hpp"
#include "../terminal_guard.hpp"
#include "../render/ui_renderer.hpp"

namespace sunray_tui {

/**
 * UI逻辑控制器
 * 负责：
 * - 事件处理和响应
 * - 业务逻辑流程控制
 * - 状态管理协调
 * 
 * 注意：已移除TUI→CLI移交机制，现在编译在TUI内部进行
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
     * 获取选中的模块列表
     * @return 模块名称列表
     */
    std::vector<std::string> get_selected_modules() const;
    
    /**
     * 获取项目根目录路径
     * @return 项目根目录绝对路径
     */
    std::string get_project_root_dir() const;
};

} // namespace sunray_tui