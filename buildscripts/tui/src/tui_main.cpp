#include "tui_config.hpp"
#include "tui_core.hpp"
#include "tui_logic.hpp"
#include <fstream>
#include <iostream>
#include <vector>


using namespace sunray_tui;

void show_help() {
  std::cout << "Sunray TUI - 交互式模块管理工具\n\n用法: sunray_tui "
               "[选项]\n\n选项:\n";
  std::cout << "  --help, -h     显示此帮助信息\n  --version, -v  "
               "显示版本信息\n\n交互控制:\n";
  std::cout << "  上下键         上下导航\n  空格           选择模块/展开组\n  "
               "C              清除所有选择\n  q/Esc          退出程序\n\n";
}

void show_version() {
  std::cout << "Sunray TUI v1.0\n基础交互式模块管理工具\n";
}

int main(int argc, char *argv[]) {
  // TUI模式不接受任何参数，如果有参数则显示提示
  if (argc > 1) {
    std::cout << "注意: TUI模式不接受参数，所有参数将被忽略\n";
    std::cout << "如需使用参数功能，请使用CLI模式\n\n";
  }

  // 检查帮助请求
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

  try {
    std::vector<std::string> possible_paths = {
        "../modules.yaml", "../../modules.yaml", "modules.yaml",
        "buildscripts/modules.yaml", "../../../modules.yaml"};

    std::string config_path;
    bool config_found = false;
    for (const auto &path : possible_paths) {
      if (std::ifstream(path).good()) {
        config_path = path;
        config_found = true;
        break;
      }
    }

    if (!config_found) {
      std::cerr << "错误: 无法找到modules.yaml配置文件\n尝试过的路径:\n";
      for (const auto &path : possible_paths)
        std::cerr << "  - " << path << "\n";
      return 1;
    }

    auto config = ConfigData::load_from_file(config_path);
    if (!config) {
      std::cerr << "错误: 无法加载配置文件: " << config_path << std::endl;
      return 1;
    }

    UIState state = ConfigDataSimplified::create_ui_state(*config);
    UILogic logic(state);

    return logic.run();

  } catch (const std::exception &e) {
    std::cerr << "启动错误: " << e.what() << std::endl;
    return 1;
  }
}