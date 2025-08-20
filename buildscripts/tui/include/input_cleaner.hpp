#pragma once

#include "input_cleanup_config.hpp"
#include "tui_reset.hpp"
#include <iostream>

namespace terminal {

/**
 * Linus风格：简化输入清理执行器
 * "Do one thing and do it well" - 只执行2层核心防护
 */
class InputCleaner {
private:
    const InputCleanupConfig& config_;
    
    void debug_log(const std::string& message) const {
        if (config_.debug_mode) {
            std::cerr << "[INPUT_CLEANER] " << message << std::endl;
        }
    }
    
public:
    explicit InputCleaner(const InputCleanupConfig& config) : config_(config) {}
    
    /**
     * 执行简化的2层输入清理流程
     * 返回值：true=成功, false=失败
     */
    bool execute_cleanup() {
        debug_log("开始执行输入清理流程");
        
        // Layer 1: 禁用鼠标跟踪
        if (config_.disable_mouse_tracking) {
            debug_log("Layer 1: 禁用鼠标跟踪");
            if (!disable_mouse_tracking()) {
                debug_log("Layer 1 失败");
                return false;
            }
        }
        
        // Layer 2: 重置终端状态  
        if (config_.reset_terminal_state) {
            debug_log("Layer 2: 重置终端状态");
            if (!reset_terminal_state_impl()) {
                debug_log("Layer 2 失败");
                return false;
            }
        }
        
        debug_log("输入清理流程完成");
        return true;
    }
    
private:
    bool disable_mouse_tracking() {
        // 调用tui_reset.hpp中的函数
        disable_all_input_aggressive();
        return true; // disable_all_input_aggressive没有返回值，假设成功
    }
    
    bool reset_terminal_state_impl() {
        // 调用tui_reset.hpp中的函数
        reset_terminal_state();
        return true; // reset_terminal_state没有返回值，假设成功
    }
};

} // namespace terminal