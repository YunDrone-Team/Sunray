#pragma once

#include <string>
#include <vector>
#include <deque>
#include <atomic>
#include <memory>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>

namespace sunray_tui {

/**
 * Linus-style BuildOutputManager: 
 * 管理子进程编译，实时捕获输出，零终端状态冲突
 * 
 * 核心原则：
 * - 非阻塞I/O，绝不卡死TUI主循环
 * - 环形缓冲区，内存使用可控
 * - 原子状态管理，线程安全
 * - 简单直接，没有特殊情况
 */
class BuildOutputManager {
public:
    enum class BuildState {
        IDLE,           // 未开始编译
        RUNNING,        // 编译进行中
        COMPLETED,      // 编译完成（成功）
        FAILED,         // 编译失败
        ERROR           // 启动错误（无法fork等）
    };

    BuildOutputManager() = default;
    ~BuildOutputManager();

    /**
     * 启动编译进程
     * @param build_command 完整的构建命令（如 "cd /path && ./build.sh --cli module1 module2"）
     * @return true启动成功，false启动失败
     */
    bool start_build(const std::string& build_command);

    /**
     * 更新状态：从管道读取新输出，检查子进程状态
     * 在TUI主循环的每一帧调用此方法
     */
    void update();

    /**
     * 停止当前编译（如果正在运行）
     */
    void stop_build();

    /**
     * 获取当前构建状态
     */
    BuildState get_state() const { return state_.load(); }

    /**
     * 获取输出行数
     */
    size_t get_output_line_count() const { return output_lines_.size(); }

    /**
     * 获取指定范围的输出行（用于UI显示）
     * @param start_line 起始行号（0-based）
     * @param max_lines 最大行数
     * @return 输出行列表
     */
    std::vector<std::string> get_output_lines(size_t start_line = 0, size_t max_lines = 1000) const;

    /**
     * 获取最新的输出行（用于自动滚动）
     */
    std::vector<std::string> get_latest_lines(size_t max_lines = 100) const;

    /**
     * 清空输出缓冲区
     */
    void clear_output();

    /**
     * 获取退出码（仅在COMPLETED或FAILED状态下有效）
     */
    int get_exit_code() const { return exit_code_; }

private:
    // 状态管理
    std::atomic<BuildState> state_{BuildState::IDLE};
    pid_t child_pid_ = -1;
    int pipe_fd_read_ = -1;
    int pipe_fd_write_ = -1;
    int exit_code_ = 0;

    // 输出缓冲区：环形缓冲区，最多保留1000行
    static constexpr size_t MAX_OUTPUT_LINES = 1000;
    std::deque<std::string> output_lines_;
    std::string partial_line_;  // 未完成的行（等待\n）

    // 私有方法
    void read_output_from_pipe();
    void check_child_status();
    void cleanup_resources();
    void add_output_line(const std::string& line);
};

} // namespace sunray_tui