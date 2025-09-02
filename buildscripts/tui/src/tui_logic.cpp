#include "tui_logic.hpp"
#include "input_cleaner.hpp"
#include "tui_reset.hpp"
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>


#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

namespace sunray_tui {

UILogic::UILogic(UIState &state) : state_(state), renderer_(state) {}

int UILogic::run() {
  try {
    return renderer_.run_with_build_callback(
        [this]() { this->execute_build(); });
  } catch (const std::runtime_error &e) {
    if (std::string(e.what()) == "User requested exit")
      return 0;
    std::cerr << "UI错误: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cerr << "UI错误: " << e.what() << std::endl;
    return 1;
  }
}

std::vector<std::string> UILogic::get_selected_modules() const {
  return {state_.view.selected_modules.begin(),
          state_.view.selected_modules.end()};
}

std::string
UILogic::format_cli_arguments(const std::vector<std::string> &modules) const {
  std::string args;
  for (size_t i = 0; i < modules.size(); ++i) {
    if (i > 0)
      args += " ";
    args += modules[i];
  }
  return args;
}

void UILogic::display_transition_message(const std::string &message) const {
  std::cout << "\n" << message << "\n" << std::flush;
}

std::string UILogic::get_project_root_dir() const {
  try {
    std::filesystem::path exe_path;
#ifdef __APPLE__
    char exe_path_buf[PATH_MAX];
    uint32_t size = sizeof(exe_path_buf);
    if (_NSGetExecutablePath(exe_path_buf, &size) == 0) {
      exe_path = std::filesystem::canonical(exe_path_buf);
    } else {
      throw std::runtime_error("无法获取可执行文件路径");
    }
#else
    exe_path = std::filesystem::canonical("/proc/self/exe");
#endif
    std::filesystem::path project_root =
        exe_path.parent_path().parent_path().parent_path();
    if (std::filesystem::exists(project_root / "build.sh"))
      return project_root.string();

    for (const auto &root_path : {"../../../", "../../", "../", "./"}) {
      std::filesystem::path test_path = std::filesystem::canonical(root_path);
      if (std::filesystem::exists(test_path / "build.sh"))
        return test_path.string();
    }
  } catch (const std::exception &e) {
    std::cerr << "路径解析异常: " << e.what() << std::endl;
  }
  return std::filesystem::current_path().string();
}

void UILogic::execute_build() {
  auto selected_modules = get_selected_modules();
  if (selected_modules.empty()) {
    std::cerr << "错误: 没有选择任何模块进行构建\n";
    return;
  }

  // 保存当前选择到隐藏文件
  save_current_selection();

  std::string project_root = get_project_root_dir();
  std::string build_script = project_root + "/build.sh";
  if (!std::filesystem::exists(build_script)) {
    std::cerr << "错误: 找不到构建脚本 " << build_script
              << "\n项目根目录: " << project_root << std::endl;
    return;
  }

  std::string full_cmd =
      build_script + " --from-tui " + format_cli_arguments(selected_modules);
  auto cleanup_config = terminal::InputCleanupConfig::production();
  terminal::InputCleaner cleaner(cleanup_config);
  if (!cleaner.execute_cleanup()) {
    std::cerr << "致命错误: 输入清理失败\n";
    exit(1);
  }

  execl("/bin/bash", "bash", "-c", full_cmd.c_str(), nullptr);
  std::cerr << "致命错误: 无法启动构建脚本\n";
  exit(1);
}

void UILogic::save_current_selection() const {
  auto selected_modules = get_selected_modules();
  if (selected_modules.empty()) {
    return;
  }

  try {
    std::string project_root = get_project_root_dir();
    std::string file_path = project_root + "/" + LAST_SELECTION_FILE;

    // 使用临时文件确保原子性写入
    std::string temp_file_path = file_path + ".tmp";
    std::ofstream temp_file(temp_file_path);
    if (!temp_file.is_open()) {
      std::cerr << "警告: 无法创建临时文件保存选择: " << temp_file_path
                << std::endl;
      return;
    }

    // 写入文件头和选择的模块
    temp_file << "# Sunray构建系统 - 上次模块选择\n";
    temp_file << "# 生成时间: " << std::time(nullptr) << "\n";
    temp_file << "# 格式: 每行一个模块名\n\n";

    for (const auto &module : selected_modules) {
      temp_file << module << "\n";
    }

    temp_file.close();

    // 原子性移动文件
    std::filesystem::rename(temp_file_path, file_path);

  } catch (const std::exception &e) {
    std::cerr << "警告: 保存模块选择时出错: " << e.what() << std::endl;
  }
}

void UILogic::load_last_selection() {
  try {
    std::string project_root = get_project_root_dir();
    std::string file_path = project_root + "/" + LAST_SELECTION_FILE;

    std::ifstream file(file_path);
    if (!file.is_open()) {
      std::cout << "提示: 未找到上次选择记录，使用默认选择\n";
      return;
    }

    std::vector<std::string> loaded_modules;
    std::string line;
    while (std::getline(file, line)) {
      // 跳过注释行和空行
      if (line.empty() || line[0] == '#') {
        continue;
      }

      // 清理模块名并添加
      line.erase(0, line.find_first_not_of(" \t"));
      line.erase(line.find_last_not_of(" \t") + 1);
      if (!line.empty()) {
        loaded_modules.push_back(line);
      }
    }

    if (loaded_modules.empty()) {
      std::cout << "提示: 上次选择记录为空，使用默认选择\n";
      return;
    }

    // 应用上次选择到当前状态
    state_.view.selected_modules.clear();
    for (const auto &module : loaded_modules) {
      state_.view.selected_modules.insert(module);
    }

    std::cout << "已加载上次选择的 " << loaded_modules.size()
              << " 个模块，按空格开始构建\n";

  } catch (const std::exception &e) {
    std::cerr << "警告: 加载上次选择时出错: " << e.what() << std::endl;
  }
}

} // namespace sunray_tui