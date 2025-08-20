#pragma once

namespace terminal {

/**
 * Linus风格：输入清理配置 - 只保留2层核心防护
 * "Configuration should be simple and obvious" - 删除垃圾，保留核心
 */
struct InputCleanupConfig {
    // Layer 1: 鼠标跟踪禁用
    bool disable_mouse_tracking = false;
    
    // Layer 2: 终端状态重置
    bool reset_terminal_state = false;
    
    // 调试模式：输出每个步骤的执行情况
    bool debug_mode = false;
    
    /**
     * 预设配置工厂方法
     */
    
    // 完全禁用模式：验证是否真的需要这些防护
    static InputCleanupConfig disabled() {
        InputCleanupConfig config;
        // 全部false，就是你刚刚测试的配置
        return config;
    }
    
    // 生产环境：Linus式简化 - 只保留2层核心防护
    static InputCleanupConfig production() {
        InputCleanupConfig config;
        config.disable_mouse_tracking = true;    // 第1层：鼠标跟踪清理
        config.reset_terminal_state = true;      // 第2层：终端状态重置
        // 其他5层都是垃圾，删除
        config.debug_mode = false;
        return config;
    }
    
    // 调试环境：Linus式简化 + 调试日志
    static InputCleanupConfig debug() {
        InputCleanupConfig config;
        config.disable_mouse_tracking = true;    // 第1层：鼠标跟踪清理
        config.reset_terminal_state = true;      // 第2层：终端状态重置
        // 其他5层都是垃圾，即使在调试模式也不要
        config.debug_mode = true;                // 只是开启日志
        return config;
    }
    
    // 极简模式：只重置终端状态
    static InputCleanupConfig minimal() {
        InputCleanupConfig config;
        config.reset_terminal_state = true;      // 只保留最核心的一层
        config.debug_mode = false;
        return config;
    }
    
};

} // namespace terminal