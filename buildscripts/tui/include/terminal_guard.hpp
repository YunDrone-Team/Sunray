#pragma once

#include <atomic>
#include <csignal>
#include <iostream>
#include "terminal_core.hpp"

namespace sunray_tui {

// Linus风格的终端管理：RAII + 原子操作 + 极简信号处理
class TerminalGuard {
public:
    TerminalGuard() {
        setup_signals();
        initialized_.store(true);
    }
    
    ~TerminalGuard() {
        // Linus风格解决双重重置问题：检查是否已经进入build模式
        // 如果是build模式，则跳过TerminalGuard的重置，让execute_build()完全控制
        if (!in_build_mode_.load()) {
            restore_terminal();
        }
    }
    
    // 新增：标记进入build模式，避免析构时的冲突重置
    static void entering_build_mode() {
        in_build_mode_.store(true);
    }
    
    // 禁用拷贝和移动
    TerminalGuard(const TerminalGuard&) = delete;
    TerminalGuard& operator=(const TerminalGuard&) = delete;
    TerminalGuard(TerminalGuard&&) = delete;
    TerminalGuard& operator=(TerminalGuard&&) = delete;

    // 全局清理函数 - 可从信号处理器调用
    static void restore_terminal() {
        if (!cleanup_in_progress_.exchange(true) && initialized_.load()) {
            // Linus风格：使用统一的Terminal核心，包含设备重置
            Terminal::reset(true, true);  // 清屏 + 设备重置
            initialized_.store(false);
        }
    }

private:
    static std::atomic<bool> initialized_;
    static std::atomic<bool> cleanup_in_progress_;
    static std::atomic<bool> in_build_mode_;  // 新增：跟踪build模式状态

    void setup_signals() {
        // Linus哲学：捕获关键信号，确保终端状态正确恢复
        signal(SIGINT, signal_handler);   // Ctrl+C
        signal(SIGTERM, signal_handler);  // kill命令
        signal(SIGQUIT, signal_handler);  // Ctrl+\
        signal(SIGHUP, signal_handler);   // 终端断开
        signal(SIGABRT, signal_handler);  // 程序异常中止
        
        // 忽略不重要的信号
        signal(SIGPIPE, SIG_IGN);
        signal(SIGCHLD, SIG_IGN);
    }

    static void signal_handler(int sig) {
        restore_terminal();
        std::_Exit(sig);  // 立即退出，不调用析构函数
    }
};

// 静态成员声明（定义在terminal_guard.cpp中）
// std::atomic<bool> TerminalGuard::initialized_{false};
// std::atomic<bool> TerminalGuard::cleanup_in_progress_{false};

} // namespace sunray_tui