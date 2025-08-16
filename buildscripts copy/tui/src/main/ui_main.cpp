#include "ui_core.hpp"
#include "config_loader.hpp"
#include "terminal_guard.hpp"
#include "core/ui_logic.hpp"

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <vector>

using namespace sunray_tui;

void show_help() {
    std::cout << "Sunray TUI - 交互式模块管理工具\n\n";
    std::cout << "用法: sunray_tui [选项]\n\n";
    std::cout << "选项:\n";
    std::cout << "  --help, -h     显示此帮助信息\n";
    std::cout << "  --version, -v  显示版本信息\n\n";
    std::cout << "交互控制:\n";
    std::cout << "  上下键         上下导航\n";
    std::cout << "  空格           选择模块/展开组\n";
    std::cout << "  C              清除所有选择\n";
    std::cout << "  q/Esc          退出程序\n\n";
}

void show_version() {
    std::cout << "Sunray TUI v1.0\n";
    std::cout << "基础交互式模块管理工具\n";
}

int main(int argc, char* argv[]) {
    // 处理命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h") {
            show_help();
            return 0;
        } else if (arg == "--version" || arg == "-v") {
            show_version();
            return 0;
        }
    }

#ifndef HAVE_FTXUI
    std::cerr << "错误: 此版本未包含FTXUI支持，无法启动TUI界面\n";
    std::cerr << "请使用 --help 查看可用选项\n";
    return 1;
#endif

    try {
        // 动态计算配置文件路径 - 从可执行文件目录向上找
        std::string config_path;
        
        // 尝试多个可能的路径
        std::vector<std::string> possible_paths = {
            "../modules.yaml",           // 从bin目录向上一级
            "../../modules.yaml",        // 从build目录向上两级  
            "modules.yaml",              // 当前目录
            "buildscripts/modules.yaml", // 从项目根目录
            "../../../modules.yaml"     // 其他可能路径
        };
        
        bool config_found = false;
        for (const auto& path : possible_paths) {
            std::ifstream test_file(path);
            if (test_file.good()) {
                config_path = path;
                config_found = true;
                break;
            }
        }
        
        if (!config_found) {
            std::cerr << "错误: 无法找到modules.yaml配置文件\n";
            std::cerr << "尝试过的路径:\n";
            for (const auto& path : possible_paths) {
                std::cerr << "  - " << path << "\n";
            }
            return 1;
        }
        
        auto config = ConfigData::load_from_file(config_path);
        if (!config) {
            std::cerr << "错误: 无法加载配置文件: " << config_path << std::endl;
            return 1;
        }

        // 创建UI状态
        UIState state = config->create_ui_state();

        // 创建并运行UI逻辑
        UILogic logic(state);
        return logic.run();

    } catch (const std::exception& e) {
        std::cerr << "启动错误: " << e.what() << std::endl;
        return 1;
    }
}