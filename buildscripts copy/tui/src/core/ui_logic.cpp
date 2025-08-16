#include "core/ui_logic.hpp"
#include "terminal_reset.hpp"

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
    
    state_.update_render_items();
    for (const auto& item : state_.render_items) {
        switch (item.type) {
            case RenderItem::GROUP_HEADER:
                std::cout << item.text << "\n";
                break;
            case RenderItem::MODULE_ITEM:
                std::cout << std::string(item.indent_level * 2, ' ') << item.text << "\n";
                break;
            case RenderItem::SEPARATOR:
                std::cout << "---\n";
                break;
            case RenderItem::INFO_TEXT:
                std::cout << item.text << "\n";
                break;
        }
    }
    
    std::cout << "\n按任意键退出...";
    std::cin.get();
    return 0;
#else
    // FTXUI版本的完整实现
    try {
        return renderer_.run_with_build_callback([this]() {
            this->execute_build();
        });
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
    std::vector<std::string> selected_modules;
    
    // 遍历所有已选中的模块
    for (const auto& module_name : state_.view.selected_modules) {
        selected_modules.push_back(module_name);
    }
    
    return selected_modules;
}

std::string UILogic::format_cli_arguments(const std::vector<std::string>& modules) const {
    std::string args;
    for (size_t i = 0; i < modules.size(); ++i) {
        if (i > 0) args += " ";
        args += modules[i];
    }
    return args;
}

void UILogic::display_transition_message(const std::string& message) const {
    std::cout << "\n" << message << "\n";
    std::cout.flush();
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

void UILogic::cleanup_terminal_via_tty() const {
    // Linus-style: Simple, direct, no external dependencies
    terminal::reset_terminal_state();
}

void UILogic::restore_terminal_via_tty() const {
    // Linus-style: execl failed, restore terminal state
    terminal::reset_terminal_state();
    std::cout << "\n" << std::flush;  // Clear current line
}

void UILogic::execute_build() {
    auto selected_modules = get_selected_modules();
    
    if (selected_modules.empty()) {
        std::cerr << "错误: 没有选择任何模块进行构建\n";
        return;
    }
    
    std::string cli_args = format_cli_arguments(selected_modules);
    
    // 构建命令
    std::string project_root = get_project_root_dir();
    std::string build_script = project_root + "/build.sh";
    std::string full_cmd = build_script + " --cli --from-tui " + cli_args;
    
    // 验证构建脚本存在
    if (!std::filesystem::exists(build_script)) {
        std::cerr << "错误: 找不到构建脚本 " << build_script << std::endl;
        std::cerr << "项目根目录: " << project_root << std::endl;
        return;
    }
    
    // Linus风格：通过TTY直接清理，不污染stdout，无多余输出
    cleanup_terminal_via_tty();
    
    // 执行CLI构建 - TUI进程将被完全替换
    execl("/bin/bash", "bash", "-c", full_cmd.c_str(), nullptr);
    
    // 如果execl返回，说明执行失败 - 强力恢复终端
    restore_terminal_via_tty();
    std::cerr << "错误: 无法启动构建脚本\n";
    exit(1);
}

} // namespace sunray_tui