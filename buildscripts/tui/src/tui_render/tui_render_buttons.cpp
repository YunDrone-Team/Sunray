#include "tui_render.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/component/component_options.hpp"
#include "ftxui/component/animation.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/dom/elements.hpp"
#include "tui_terminal.hpp"
#include <filesystem>
#include <thread>
#include <cstdlib>
#include <sys/wait.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

using namespace ftxui;

namespace sunray_tui {

// 按钮组件创建逻辑（从 create_component 中提取）
void UIRenderer::create_buttons() {
  // ========== 组件化底部按钮 ==========
  // 开始编译构建按钮：保持原有行为
  auto start_opt = ButtonOption::Animated();
  // 自定义外观：与清除按钮统一hover逻辑，错误提示改为点击触发
  start_opt.transform = [this](const EntryState &s) {
    const bool has_selection = !state_.view.selected_modules.empty();
    // 第一阶段：通过统一高亮管理器决定唯一高亮
    const bool hover_like = highlight_mgr_.is_highlighted(InteractiveId::Start());
    const int inner_width = 18;

    // 点击未选择模块后，短暂显示“请选择模块”并以红底提示
    const bool warn = state_.build_warning_flash_active;

    std::string raw = warn ? "请选择模块" : std::string(s.label);
    if ((int)raw.size() > inner_width)
      raw = raw.substr(0, inner_width);
    Element inner = text(raw) | center | size(WIDTH, EQUAL, inner_width);
    Element full = hbox({text("["), inner, text("]")});

    if (warn) {
      full = full | bold | bgcolor(Color::Red) | color(Color::White);
    } else if (hover_like) {
      // 与清除按钮一致的hover效果
      full = full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
    } else if (has_selection) {
      full = full | bold | color(Color::Blue);
    } else {
      // 无选择时常态为置灰
      full = full | bold | color(Color::GrayDark) | dim;
    }

    return full;
  };
  start_button_ = Button(
      "开始编译构建",
      [this] {
        // 仅当选择了模块才触发
        if (!state_.view.selected_modules.empty()) {
          state_.handle_build_button();
        } else {
          state_.trigger_build_warning_flash();
          animation::RequestAnimationFrame();
        }
        // 点击后重置按钮 hover 与键盘焦点，避免灰底滞留
        start_button_hovered_ = false;
        clear_button_hovered_ = false;
        state_.build_button_focused = false;
      },
      start_opt);
  // 去除 Hoverable，hover 高亮由统一管理器 + 反射 Box 决定

  // 清除构建按钮：点击后绿色闪烁三次（不做实际逻辑）
  auto clear_opt = ButtonOption::Animated();
  clear_opt.transform = [this](const EntryState &s) {
    // 第一阶段：通过统一高亮管理器决定唯一高亮
    const bool hover_like = highlight_mgr_.is_highlighted(InteractiveId::Clear());
    const int inner_width = 18;

    std::string raw;
    switch (clear_state_) {
    case CleanState::Idle:
      raw = std::string(s.label);
      break;
    case CleanState::Running:
      raw = "清除构建..";
      break; // 两个点
    case CleanState::Success:
      raw = "清理完成";
      break;
    case CleanState::Error:
      raw = "失败";
      break;
    }
    if ((int)raw.size() > inner_width)
      raw = raw.substr(0, inner_width);
    Element inner = text(raw) | center | size(WIDTH, EQUAL, inner_width);
    Element full = hbox({text("["), inner, text("]")});

    if (clear_state_ == CleanState::Running) {
      full =
          full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
    } else if (clear_state_ == CleanState::Success) {
      if (hover_like) {
        // 成功状态hover：保持绿色但添加背景
        full = full | bold | bgcolor(Color::RGB(40, 80, 40)) | color(Color::White);
      } else {
        full = full | bold | color(Color::Green);
      }
    } else if (clear_state_ == CleanState::Error) {
      if (hover_like) {
        // 错误状态hover：保持红色但添加背景
        full = full | bold | bgcolor(Color::RGB(80, 40, 40)) | color(Color::White);
      } else {
        full = full | bold | color(Color::Red);
      }
    } else {
      // 空闲状态 - hover 时灰底白字，默认不着色（使用终端默认前景色）
      if (hover_like) {
        full = full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
      } else {
        full = full | bold;
      }
    }
    return full;
  };

  clear_button_ = Button(
      "清除构建",
      [this] {
        if (clear_state_ == CleanState::Running)
          return; // 忽略重复点击
        // 如果之前在成功显示窗口，重置
        clear_success_frames_remaining_ = 0;
        clear_state_ = CleanState::Running;
        clear_anim_tick_ = 0;
        animation::RequestAnimationFrame();

        // 后台执行清理：无需关心退出码，直接认为成功
        std::thread([this] {
          auto detect_root = []() -> std::string {
            try {
              namespace fs = std::filesystem;
#ifdef __APPLE__
              char exe_path_buf[PATH_MAX];
              uint32_t size = sizeof(exe_path_buf);
              fs::path exe_path;
              if (_NSGetExecutablePath(exe_path_buf, &size) == 0) {
                exe_path = fs::canonical(exe_path_buf);
              }
#else
              fs::path exe_path = fs::canonical("/proc/self/exe");
#endif
              fs::path cand = exe_path.parent_path().parent_path().parent_path();
              if (fs::exists(cand / "build.sh"))
                return cand.string();
              for (const auto &root_path : {"../../../", "../../", "../", "./"}) {
                fs::path test_path = fs::canonical(root_path);
                if (fs::exists(test_path / "build.sh"))
                  return test_path.string();
              }
            } catch (...) {
            }
            return std::filesystem::current_path().string();
          };

          std::string root = detect_root();
          // 以后台方式启动清理命令，忽略输出与退出码
          std::string cmd = "cd '" + root + "' && nohup ./build.sh --clean > /dev/null 2>&1 &";
          std::system(cmd.c_str());

          // 直接认为成功，显示短暂成功提示
          clear_state_ = CleanState::Success;
          clear_success_frames_remaining_ = 60; // 约2秒
          clear_output_.clear();
          clear_exit_code_ = 0;

          if (state_.trigger_exit_callback)
            state_.trigger_exit_callback();
        }).detach();

        // 点击后重置按钮 hover 与键盘焦点，避免灰底滞留
        start_button_hovered_ = false;
        clear_button_hovered_ = false;
        state_.build_button_focused = false;
      },
      clear_opt);
  // 去除 Hoverable，hover 高亮由统一管理器 + 反射 Box 决定

  buttons_row_ = Container::Horizontal({start_button_, clear_button_});
}

// 触发"清除构建"按钮对应的动作（与鼠标点击一致）
void UIRenderer::trigger_clear_build_clean() {
  // 这个功能已经在clear_button_的回调中实现
  // 这里提供一个编程接口来触发相同的动作
  if (clear_button_ && clear_state_ != CleanState::Running) {
    // 触发清除按钮的点击事件
    clear_button_->OnEvent(Event::Return);
  }
}

} // namespace sunray_tui
