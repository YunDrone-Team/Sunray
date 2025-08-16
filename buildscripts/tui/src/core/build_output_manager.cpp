#include "core/build_output_manager.hpp"
#include <iostream>
#include <errno.h>
#include <signal.h>
#include <cstring>
#include <algorithm>

// 简化版TRACE宏
#define TRACE_CRITICAL(tag, msg) do { \
    std::cout << "[" << tag << "] " << msg << std::endl; \
} while(0)

namespace sunray_tui {

BuildOutputManager::~BuildOutputManager() {
    cleanup_resources();
}

bool BuildOutputManager::start_build(const std::string& build_command) {
    // 如果已有编译在运行，先停止
    if (state_.load() == BuildState::RUNNING) {
        stop_build();
    }

    TRACE_CRITICAL("BUILD_START", "Starting build: " + build_command);

    // 创建管道
    int pipe_fds[2];
    if (pipe(pipe_fds) == -1) {
        TRACE_CRITICAL("BUILD_ERROR", "Failed to create pipe: " + std::string(strerror(errno)));
        state_ = BuildState::ERROR;
        return false;
    }

    pipe_fd_read_ = pipe_fds[0];
    pipe_fd_write_ = pipe_fds[1];

    // 设置读端为非阻塞
    int flags = fcntl(pipe_fd_read_, F_GETFL);
    if (fcntl(pipe_fd_read_, F_SETFL, flags | O_NONBLOCK) == -1) {
        TRACE_CRITICAL("BUILD_ERROR", "Failed to set pipe non-blocking: " + std::string(strerror(errno)));
        cleanup_resources();
        state_ = BuildState::ERROR;
        return false;
    }

    // 创建子进程
    child_pid_ = fork();
    if (child_pid_ == -1) {
        TRACE_CRITICAL("BUILD_ERROR", "Failed to fork: " + std::string(strerror(errno)));
        cleanup_resources();
        state_ = BuildState::ERROR;
        return false;
    }

    if (child_pid_ == 0) {
        // 子进程：重定向stdout和stderr到管道写端
        close(pipe_fd_read_);  // 子进程不需要读端
        
        // 重定向stdout和stderr到管道写端
        dup2(pipe_fd_write_, STDOUT_FILENO);
        dup2(pipe_fd_write_, STDERR_FILENO);
        close(pipe_fd_write_);

        // 执行构建命令
        execl("/bin/bash", "bash", "-c", build_command.c_str(), nullptr);
        
        // 如果execl失败，输出错误信息并退出
        perror("execl failed");
        _exit(1);
    } else {
        // 父进程：关闭写端，准备读取输出
        close(pipe_fd_write_);
        pipe_fd_write_ = -1;
        
        state_ = BuildState::RUNNING;
        exit_code_ = 0;
        clear_output();
        
        TRACE_CRITICAL("BUILD_FORKED", "Child process started with PID: " + std::to_string(child_pid_));
        return true;
    }
}

void BuildOutputManager::update() {
    if (state_.load() != BuildState::RUNNING) {
        return;
    }

    // 读取管道输出
    read_output_from_pipe();
    
    // 检查子进程状态
    check_child_status();
}

void BuildOutputManager::stop_build() {
    if (state_.load() != BuildState::RUNNING || child_pid_ == -1) {
        return;
    }

    TRACE_CRITICAL("BUILD_STOP", "Stopping build process PID: " + std::to_string(child_pid_));

    // 发送SIGTERM信号给子进程
    kill(child_pid_, SIGTERM);
    
    // 等待一小段时间让子进程优雅退出
    usleep(100000);  // 100ms
    
    // 如果还没退出，发送SIGKILL强制终止
    int status;
    if (waitpid(child_pid_, &status, WNOHANG) == 0) {
        kill(child_pid_, SIGKILL);
        waitpid(child_pid_, &status, 0);
    }

    cleanup_resources();
    state_ = BuildState::FAILED;
}

std::vector<std::string> BuildOutputManager::get_output_lines(size_t start_line, size_t max_lines) const {
    std::vector<std::string> result;
    
    if (start_line >= output_lines_.size()) {
        return result;
    }

    size_t end_line = std::min(start_line + max_lines, output_lines_.size());
    
    for (size_t i = start_line; i < end_line; ++i) {
        result.push_back(output_lines_[i]);
    }

    return result;
}

std::vector<std::string> BuildOutputManager::get_latest_lines(size_t max_lines) const {
    if (output_lines_.empty()) {
        return {};
    }

    size_t start_line = 0;
    if (output_lines_.size() > max_lines) {
        start_line = output_lines_.size() - max_lines;
    }

    return get_output_lines(start_line, max_lines);
}

void BuildOutputManager::clear_output() {
    output_lines_.clear();
    partial_line_.clear();
}

void BuildOutputManager::read_output_from_pipe() {
    if (pipe_fd_read_ == -1) {
        return;
    }

    char buffer[4096];
    ssize_t bytes_read;

    while ((bytes_read = read(pipe_fd_read_, buffer, sizeof(buffer) - 1)) > 0) {
        buffer[bytes_read] = '\0';
        
        // 处理读取的数据，按行分割
        std::string data = partial_line_ + std::string(buffer);
        partial_line_.clear();

        size_t pos = 0;
        size_t newline_pos;
        
        while ((newline_pos = data.find('\n', pos)) != std::string::npos) {
            std::string line = data.substr(pos, newline_pos - pos);
            add_output_line(line);
            pos = newline_pos + 1;
        }

        // 保存未完成的行
        if (pos < data.length()) {
            partial_line_ = data.substr(pos);
        }
    }

    // 检查是否是因为EAGAIN/EWOULDBLOCK而结束（正常情况）
    if (bytes_read == -1 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
        TRACE_CRITICAL("BUILD_READ_ERROR", "Error reading from pipe: " + std::string(strerror(errno)));
    }
}

void BuildOutputManager::check_child_status() {
    if (child_pid_ == -1) {
        return;
    }

    int status;
    pid_t result = waitpid(child_pid_, &status, WNOHANG);
    
    if (result == child_pid_) {
        // 子进程已结束
        TRACE_CRITICAL("BUILD_CHILD_EXIT", "Child process exited");
        
        // 读取剩余的输出
        read_output_from_pipe();
        
        // 处理未完成的行
        if (!partial_line_.empty()) {
            add_output_line(partial_line_);
            partial_line_.clear();
        }

        if (WIFEXITED(status)) {
            exit_code_ = WEXITSTATUS(status);
            if (exit_code_ == 0) {
                state_ = BuildState::COMPLETED;
                TRACE_CRITICAL("BUILD_SUCCESS", "Build completed successfully");
            } else {
                state_ = BuildState::FAILED;
                TRACE_CRITICAL("BUILD_FAILED", "Build failed with exit code: " + std::to_string(exit_code_));
            }
        } else if (WIFSIGNALED(status)) {
            exit_code_ = -WTERMSIG(status);
            state_ = BuildState::FAILED;
            TRACE_CRITICAL("BUILD_KILLED", "Build killed by signal: " + std::to_string(WTERMSIG(status)));
        }

        cleanup_resources();
    } else if (result == -1 && errno != ECHILD) {
        TRACE_CRITICAL("BUILD_WAIT_ERROR", "Error waiting for child: " + std::string(strerror(errno)));
    }
}

void BuildOutputManager::cleanup_resources() {
    if (pipe_fd_read_ != -1) {
        close(pipe_fd_read_);
        pipe_fd_read_ = -1;
    }
    
    if (pipe_fd_write_ != -1) {
        close(pipe_fd_write_);
        pipe_fd_write_ = -1;
    }
    
    child_pid_ = -1;
}

void BuildOutputManager::add_output_line(const std::string& line) {
    output_lines_.push_back(line);
    
    // 维护环形缓冲区大小
    if (output_lines_.size() > MAX_OUTPUT_LINES) {
        output_lines_.pop_front();
    }
}

} // namespace sunray_tui