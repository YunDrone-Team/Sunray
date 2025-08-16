#include "core/ui_logic.hpp"
#include "terminal_reset.hpp"
#include "terminal_guard.hpp"   // 新增：用于entering_build_mode()
#include "terminal_core.hpp"  // 直接使用sunray_tui::Terminal

// 简化版TRACE宏
#define TRACE_CRITICAL(tag, msg) do { \
    std::cout << "[" << tag << "] " << msg << std::endl; \
} while(0)

#ifdef HAVE_FTXUI
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/animation.hpp"
#endif

#include <iostream>
#include <stdexcept>
#include <unistd.h>  // for execl
#include <filesystem>
#include <fstream>
#include <cstdlib>  // for exit
#include <csignal>  // for signal handling
#include <fcntl.h>  // for open
#include <cstring>  // for strlen
#ifdef __APPLE__
#include <mach-o/dyld.h>  // for _NSGetExecutablePath
#include <climits>        // for PATH_MAX
#endif

namespace sunray_tui {

UILogic::UILogic(UIState& state) 
    : state_(state), renderer_(state) {}

int UILogic::run() {
#ifndef HAVE_FTXUI
    // 非FTXUI版本的简化实现
    TerminalGuard guard;
    
    std::cout << "Sunray TUI - 模块管理工具\n";
    std::cout << "注意: 未编译FTXUI支持，使用简化界面\n\n";
    
    // Display groups section
    std::cout << "=== 模块组 ===\n";
    for (const auto& group : state_.groups) {
        std::cout << group.display_name() << "\n";
    }
    
    std::cout << "\n=== 模块列表 ===\n";
    for (const auto& module : state_.modules) {
        std::cout << module.display_name() << "\n";
    }
    
    std::cout << "\n按任意键退出...";
    std::cin.get();
    return 0;
#else
    // FTXUI版本：新架构，不再需要构建回调
    try {
        return renderer_.run_with_build_callback();
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        if (msg == "User requested exit") {
            return 0;  // 正常退出
        }
        std::cerr << "UI错误: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "UI错误: " << e.what() << std::endl;
        return 1;
    }
#endif
}

std::vector<std::string> UILogic::get_selected_modules() const {
    return state_.get_selected_modules();
}

std::string UILogic::get_project_root_dir() const {
    // Linus风格：从可执行文件绝对路径推算项目根目录，避免相对路径依赖
    try {
        // 获取可执行文件的绝对路径
        std::filesystem::path exe_path;
        
        #ifdef __APPLE__
        // macOS上使用_NSGetExecutablePath获取可执行文件路径
        char exe_path_buf[PATH_MAX];
        uint32_t size = sizeof(exe_path_buf);
        if (_NSGetExecutablePath(exe_path_buf, &size) == 0) {
            exe_path = std::filesystem::canonical(exe_path_buf);
        } else {
            throw std::runtime_error("无法获取可执行文件路径");
        }
        #else
        // Linux上使用/proc/self/exe
        exe_path = std::filesystem::canonical("/proc/self/exe");
        #endif
        
        // 从 .../buildscripts/bin/sunray_tui 向上两级到项目根目录
        std::filesystem::path project_root = exe_path.parent_path().parent_path().parent_path();
        
        // 验证这确实是项目根目录（应该包含build.sh）
        std::filesystem::path build_script = project_root / "build.sh";
        if (std::filesystem::exists(build_script)) {
            return project_root.string();
        }
        
        // 如果标准路径不存在，尝试备用方法
        std::vector<std::string> possible_roots = {
            "../../../",     // 从buildscripts/bin/sunray_tui向上三级
            "../../",       // 从buildscripts/tui向上两级
            "../",          // 从buildscripts向上一级
            "./"            // 当前目录（如果直接在项目根运行）
        };
        
        for (const auto& root_path : possible_roots) {
            std::filesystem::path test_path = std::filesystem::canonical(root_path);
            std::filesystem::path test_build = test_path / "build.sh";
            if (std::filesystem::exists(test_build)) {
                return test_path.string();
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "路径解析异常: " << e.what() << std::endl;
    }
    
    // 最后的fallback：返回当前目录
    return std::filesystem::current_path().string();
}

} // namespace sunray_tui