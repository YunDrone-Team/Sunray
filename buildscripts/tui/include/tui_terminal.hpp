#pragma once

#include <atomic>
#include <csignal>
#include <iostream>

namespace sunray_tui {

// Linus风格的终端管理：RAII + 原子操作 + 极简信号处理
class TerminalGuard {
public:
    TerminalGuard() {
        setup_signals();
        initialized_.store(true);
    }
    
    ~TerminalGuard() {
        restore_terminal();
    }
    
    // 禁用拷贝和移动
    TerminalGuard(const TerminalGuard&) = delete;
    TerminalGuard& operator=(const TerminalGuard&) = delete;
    TerminalGuard(TerminalGuard&&) = delete;
    TerminalGuard& operator=(TerminalGuard&&) = delete;

    // 全局清理函数 - 可从信号处理器调用
    static void restore_terminal() {
        if (!cleanup_in_progress_.exchange(true) && initialized_.load()) {
            // 简化的终端重置，避免产生乱码
            std::cout << "\033[?25h"      // 显示光标
                      << "\033[0m"        // 重置所有属性  
                      << "\033[?1000l"    // 禁用X10鼠标报告
                      << "\033[?1002l"    // 禁用按钮事件跟踪
                      << "\033[?1003l"    // 禁用任何事件跟踪
                      << "\033[?2004l"    // 禁用括号粘贴模式
                      << std::flush;      // 立即刷新
            initialized_.store(false);
        }
    }

private:
    static std::atomic<bool> initialized_;
    static std::atomic<bool> cleanup_in_progress_;

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

} // namespace sunray_tui