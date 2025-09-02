#include "tui_render.hpp"
#include "ftxui/component/animation.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/component/component_options.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/screen/terminal.hpp"
#include "tui_terminal.hpp"
#include <cstdlib>
#include <filesystem>
#include <sys/wait.h>
#include <thread>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

using namespace ftxui;

namespace sunray_tui {

UIRenderer::UIRenderer(UIState &state) : state_(state) {}

int UIRenderer::run_with_build_callback(std::function<void()> build_callback) {
  build_callback_ = build_callback;

  TerminalGuard guard;

  auto component = create_component();

  auto screen = ScreenInteractive::Fullscreen();

  // 设置触发退出的回调，解决TUI退出延迟问题
  state_.trigger_exit_callback = [&screen]() {
    screen.PostEvent(Event::Custom);
  };

  // 立即检查构建请求，修复了延迟问题
  bool should_build = false;

  auto wrapped_component = CatchEvent(component, [&](Event event) -> bool {
    // 如果构建被请求，立即退出
    if (state_.build_requested) {
      state_.build_requested = false;
      should_build = true;
      screen.ExitLoopClosure()();
      return true;
    }
    return false;
  });

  screen.Loop(wrapped_component);

  // FTXUI循环退出后，检查是否需要执行构建
  if (should_build && build_callback_) {
    build_callback_(); // TUI已退出，现在执行构建
  }

  return 0;
}

Component UIRenderer::create_component() {
  // 初始化双栏渲染项
  state_.update_group_render_items();
  state_.update_module_render_items();

  // ========== 组件化底部按钮 ==========
  // 开始编译构建按钮：保持原有行为
  auto start_opt = ButtonOption::Animated();
  // 自定义外观（方括号+括号内固定宽度6，hover包含括号，未hover无背景）
  start_opt.transform = [this](const EntryState &s) {
    const bool has_selection = !state_.view.selected_modules.empty();
    const bool hover_like = (start_button_hovered_ || s.focused || s.active);
    const int inner_width = 18;

    std::string raw = (!has_selection && start_button_hovered_)
                          ? "请选择模块"
                          : std::string(s.label);
    if ((int)raw.size() > inner_width)
      raw = raw.substr(0, inner_width);
    Element inner = text(raw) | center | size(WIDTH, EQUAL, inner_width);
    Element full = hbox({text("["), inner, text("]")});

    if (!has_selection) {
      if (start_button_hovered_) {
        full = full | bold | bgcolor(Color::Red) | color(Color::White);
      } else {
        full = full | bold | color(Color::GrayDark) | dim;
      }
    } else {
      if (hover_like) {
        full =
            full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
      } else {
        full = full | bold | color(Color::Blue);
      }
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
      },
      start_opt);
  // 绑定 Hover 状态
  start_button_ = Hoverable(start_button_, &start_button_hovered_);

  // 清除构建按钮：点击后绿色闪烁三次（不做实际逻辑）
  auto clear_opt = ButtonOption::Animated();
  clear_opt.transform = [this](const EntryState &s) {
    const bool hover_like = (clear_button_hovered_ || s.focused || s.active);
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
      raw = "已完成";
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
      full = full | bold | color(Color::Green);
    } else if (clear_state_ == CleanState::Error) {
      full = full | bold | color(Color::Red);
    } else {
      if (hover_like) {
        full =
            full | bold | bgcolor(Color::RGB(60, 60, 60)) | color(Color::White);
      } else {
        full = full | bold | color(Color::Green);
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

        // 后台线程执行清理
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
              fs::path cand =
                  exe_path.parent_path().parent_path().parent_path();
              if (fs::exists(cand / "build.sh"))
                return cand.string();
              for (const auto &root_path :
                   {"../../../", "../../", "../", "./"}) {
                fs::path test_path = fs::canonical(root_path);
                if (fs::exists(test_path / "build.sh"))
                  return test_path.string();
              }
            } catch (...) {
            }
            return std::filesystem::current_path().string();
          };

          std::string root = detect_root();
          // 捕获输出，并在末尾打印显式退出码标记，避免 pclose()/wait
          // 宏兼容性问题
          std::string cmd = "cd '" + root +
                            "' && ( ./build.sh --clean 2>&1; printf "
                            "\"__EXIT_CODE:%d\\n\" $? )";
          std::string out;
          FILE *pipe = popen(cmd.c_str(), "r");
          if (pipe) {
            char buf[512];
            while (fgets(buf, sizeof(buf), pipe) != nullptr) {
              out.append(buf);
            }
          }
          int status = pipe ? pclose(pipe) : -1;
          // 优先从输出中解析显式退出码
          bool ok = false;
          int parsed_exit = -1;
          const std::string marker = "__EXIT_CODE:";
          size_t mpos = out.rfind(marker);
          if (mpos != std::string::npos) {
            size_t line_end = out.find('\n', mpos);
            std::string code_str =
                out.substr(mpos + marker.size(),
                           (line_end == std::string::npos
                                ? std::string::npos
                                : line_end - (mpos + marker.size())));
            try {
              parsed_exit = std::stoi(code_str);
            } catch (...) {
              parsed_exit = -1;
            }
            ok = (parsed_exit == 0);
            // 移除标记行
            out.erase(mpos,
                      (line_end == std::string::npos ? out.size() - mpos
                                                     : (line_end - mpos + 1)));
          } else {
            // 回退：使用 pclose 返回的 wait 状态
            ok = (status != -1) && WIFEXITED(status) &&
                 (WEXITSTATUS(status) == 0);
            parsed_exit =
                (status != -1 && WIFEXITED(status)) ? WEXITSTATUS(status) : -1;
          }
          if (ok) {
            clear_state_ = CleanState::Success;
            clear_success_frames_remaining_ = 30 + 30; // 约2秒
            clear_output_.clear();
            clear_exit_code_ = 0;
          } else {
            clear_state_ = CleanState::Error;
            clear_success_frames_remaining_ = 60; // 失败提示约2秒
            clear_exit_code_ = parsed_exit;
            clear_output_ = std::move(out);
          }
          // 唤醒渲染循环
          if (state_.trigger_exit_callback)
            state_.trigger_exit_callback();
        }).detach();
      },
      clear_opt);
  // 绑定 Hover 状态
  clear_button_ = Hoverable(clear_button_, &clear_button_hovered_);

  buttons_row_ = Container::Horizontal({start_button_, clear_button_});

  auto renderer = Renderer([&] {
    // 首先检查窗口尺寸
    if (!state_.check_window_size()) {
      auto [current_width, current_height] = state_.get_terminal_size();

      // 窗口太小，显示尺寸警告
      Elements warning_elements;
      warning_elements.push_back(text("") | center); // 空行用于垂直居中
      warning_elements.push_back(text("⚠️  终端窗口尺寸过小  ⚠️") | bold |
                                 color(Color::Red) | center);
      warning_elements.push_back(text("") | center);
      warning_elements.push_back(
          text("最小要求: " + std::to_string(UIState::MIN_TERMINAL_WIDTH) +
               " × " + std::to_string(UIState::MIN_TERMINAL_HEIGHT)) |
          color(Color::Yellow) | center);
      warning_elements.push_back(
          text("当前大小: " + std::to_string(current_width) + " × " +
               std::to_string(current_height)) |
          color(Color::Cyan) | center);
      warning_elements.push_back(text("") | center);
      warning_elements.push_back(text("请调整终端窗口大小") |
                                 color(Color::White) | center);
      warning_elements.push_back(text("") | center);

      return vbox(warning_elements) | center | border;
    }

    // 窗口尺寸正常，渲染正常UI
    Elements elements;

    // 标题区域
    elements.push_back(text("Sunray Build System - TUI") | bold | center);
    elements.push_back(separator());

    // 创建左栏（组选择器）
    Elements left_column_elements;
    left_column_elements.push_back(text("模块组") | bold | center);
    left_column_elements.push_back(separator());

    for (size_t i = 0; i < state_.group_render_items.size(); ++i) {
      const auto &item = state_.group_render_items[i];
      bool is_selected =
          (i == static_cast<size_t>(state_.group_selection_index));
      bool is_focused = state_.left_pane_focused;
      bool is_hovered = (i == static_cast<size_t>(state_.group_hover_index));
      left_column_elements.push_back(
          render_group_item(item, is_selected, is_focused, is_hovered));
    }
    if (state_.group_render_items.empty()) {
      left_column_elements.push_back(text("没有可用的模块组") | dim | center);
    }

    // 创建右栏（所有模块显示器）- 支持滚动
    Elements right_column_elements;
    right_column_elements.push_back(text("所有模块") | bold | center);
    right_column_elements.push_back(separator());

    // 重新计算可见数量（终端可能调整大小）
    state_.calculate_module_visible_count();
    state_.ensure_module_selection_visible();

    // 只渲染可见范围内的模块
    if (!state_.module_render_items.empty()) {
      const int start_index = state_.module_scroll_offset;
      const int end_index =
          std::min(static_cast<int>(state_.module_render_items.size()),
                   start_index + state_.module_visible_count);

      for (int i = start_index; i < end_index; ++i) {
        const auto &item = state_.module_render_items[i];
        bool is_selected = (i == state_.module_selection_index);
        bool is_focused = !state_.left_pane_focused;
        bool is_hovered = (i == state_.module_hover_index);
        right_column_elements.push_back(
            render_module_item(item, is_selected, is_focused, is_hovered));
      }

      // 显示滚动指示器（如果需要）
      const int total_modules =
          static_cast<int>(state_.module_render_items.size());
      if (total_modules > state_.module_visible_count) {
        std::string scroll_info = "(" + std::to_string(start_index + 1) + "-" +
                                  std::to_string(end_index) + "/" +
                                  std::to_string(total_modules) + ")";
        right_column_elements.push_back(text(scroll_info) | dim |
                                        color(Color::GrayLight) | center);
      }
    } else {
      right_column_elements.push_back(text("没有可用的模块") | dim | center);
    }

    // 获取终端尺寸并计算双栏可用高度
    auto [terminal_width, terminal_height] = state_.get_terminal_size();

    // 计算固定UI元素占用的高度
    // 标题(1) + 分隔符(1) + 分隔符(1) + 描述(1) + 详细信息(3) + 分隔符(1) +
    // 构建按钮(1) + 分隔符(1) + 按键提示(3) + 调试窗口(5) + 边框(2)
    const int fixed_ui_height =
        1 + 1 + 1 + 1 + 3 + 1 + 1 + 1 + 3 + 5 + 2; // = 20行
    const int available_height_for_columns =
        std::max(8, terminal_height - fixed_ui_height); // 最少8行给双栏

    // 组合左右栏为双栏布局 - 使用固定的50/50分割和动态高度
    Element left_column = vbox(left_column_elements) | border |
                          size(WIDTH, EQUAL, terminal_width / 2 - 1) |
                          size(HEIGHT, EQUAL, available_height_for_columns) |
                          flex;
    Element right_column = vbox(right_column_elements) | border |
                           size(WIDTH, EQUAL, terminal_width / 2 - 1) |
                           size(HEIGHT, EQUAL, available_height_for_columns) |
                           flex;
    elements.push_back(hbox({left_column, right_column}));
    elements.push_back(separator());

    // 记录当前元素数量，用于计算描述行Y坐标
    const int description_line_y = static_cast<int>(elements.size());

    // 显示当前选中项的信息 - 固定4行高度
    elements.push_back(
        text("描述: " + (state_.current_item_description.empty()
                             ? "NULL"
                             : state_.current_item_description)) |
        bold);

    // 收集详细信息行，确保总共4行
    std::vector<Element> detail_lines;
    if (!state_.current_item_details.empty()) {
      std::string details = state_.current_item_details;
      size_t pos = 0;
      while (pos < details.length()) {
        size_t end = details.find('\n', pos);
        std::string line = (end == std::string::npos)
                               ? details.substr(pos)
                               : details.substr(pos, end - pos);
        pos = (end == std::string::npos) ? details.length() : end + 1;

        // 检查是否是冲突行且包含非NULL内容
        if (line.find("冲突: ") == 0 &&
            line.find("NULL") == std::string::npos) {
          if (state_.conflict_flash_active &&
              (state_.conflict_flash_count % 2 == 1)) {
            detail_lines.push_back(text(line) | bgcolor(Color::Yellow) |
                                   color(Color::Black) | bold | blink);
          } else {
            detail_lines.push_back(text(line) | bgcolor(Color::Red) |
                                   color(Color::White) | bold);
          }
        } else {
          detail_lines.push_back(text(line) | dim);
        }
        if (end == std::string::npos)
          break;
      }
    }
    // 确保总共有4行（包括描述行，还需要3行详细信息）
    while (detail_lines.size() < 3) {
      detail_lines.push_back(text("") | dim);
    }
    // 只显示前3行详细信息（加上描述行总共4行）
    for (size_t i = 0; i < std::min(detail_lines.size(), size_t(3)); ++i) {
      elements.push_back(detail_lines[i]);
    }

    elements.push_back(separator());

    // 底部按钮改为组件，无需计算构建按钮的Y坐标
    state_.build_button_screen_y = -1;

    // 添加底部按钮行（组件渲染）- 紧贴详细信息下方
    Element buttons_row_el = buttons_row_->Render();
    elements.push_back(hbox({filler(), buttons_row_el, filler()}));

    // 失败详情输出（自动换行）
    if (clear_state_ == CleanState::Error && !clear_output_.empty()) {
      elements.push_back(separator());
      elements.push_back(
          text("清理失败 (exit: " + std::to_string(clear_exit_code_) + ")") |
          color(Color::Red) | bold);
      elements.push_back(paragraph(clear_output_) | color(Color::Red));
    }

    // 渲染结束后重置动画状态
    if (state_.animation_in_progress) {
      state_.animation_in_progress = false;
      state_.previous_group_selection = -1;
      state_.previous_module_selection = -1;
    }
    // 更新冲突闪烁状态
    if (state_.conflict_flash_active) {
      state_.update_conflict_flash();
      animation::RequestAnimationFrame();
    }
    // 更新构建按钮警告闪烁状态
    if (state_.build_warning_flash_active) {
      state_.update_build_warning_flash();
      animation::RequestAnimationFrame();
    }
    // 清除构建按钮状态推进
    if (clear_state_ == CleanState::Running) {
      clear_anim_tick_ = (clear_anim_tick_ + 1) % 1000;
      animation::RequestAnimationFrame();
    } else if (clear_state_ == CleanState::Success ||
               clear_state_ == CleanState::Error) {
      if (clear_success_frames_remaining_ > 0) {
        --clear_success_frames_remaining_;
        animation::RequestAnimationFrame();
      } else {
        // 恢复到 Idle
        clear_state_ = CleanState::Idle;
        clear_anim_tick_ = 0;
      }
    }

    // 主界面内容（包含底部按钮，不包含按键指南）
    Element main_content = vbox(elements) | border;

    // 🔥 创建按键指南作为独立区域
    Element key_guide = render_key_guide();

    // 🔥 只有在有调试元素启用时才显示调试窗口
    if (calculate_debug_content_lines() > 0) {
      Element debug_window = render_debug_window();
      Element full_interface =
          vbox({main_content | flex, key_guide, debug_window});

      // 底部按钮改为组件，不再参与坐标映射
      state_.build_button_screen_y = -1;
      // 重建统一的坐标映射（双栏版本），不含构建按钮
      rebuild_dual_column_coordinate_mapping();

      return full_interface;
    } else {
      // 没有调试元素时，调试窗口完全消失
      Element full_interface = vbox({main_content | flex, key_guide});

      // 底部按钮改为组件，不再参与坐标映射
      state_.build_button_screen_y = -1;
      // 重建统一的坐标映射（双栏版本），不含构建按钮
      rebuild_dual_column_coordinate_mapping();

      return full_interface;
    }
  });

  return CatchEvent(renderer, [this](Event event) {
    // 先交给底部按钮组件处理（鼠标/键盘都可）
    if (buttons_row_ && buttons_row_->OnEvent(event)) {
      return true;
    }
    // 其余交由双栏键盘/鼠标逻辑
    return handle_dual_column_keyboard_event(event);
  });
}

// ==================== 双栏渲染辅助方法 ====================

ftxui::Element UIRenderer::render_group_item(const RenderItem &item,
                                             bool is_selected, bool is_focused,
                                             bool is_hovered) {
  Element left_content;
  bool is_active_group = (state_.view.active_group == item.identifier);
  if (item.identifier == "ungrouped") {
    left_content = text(item.text) | italic | dim;
  } else if (is_active_group) {
    left_content = text(item.text) | color(Color::Yellow) | bold;
  } else {
    left_content = text(item.text) | bold;
  }
  Element right_content =
      item.has_selected_items
          ? text(" " + item.counter_text + " ") | bgcolor(Color::Yellow) |
                color(Color::Black) | bold
          : text(" " + item.counter_text + " ") | color(Color::GrayLight);
  Element content = hbox({left_content | flex, right_content});

  // 统一hover效果 - 只使用hover状态，完全移除focus概念
  if (is_hovered) {
    content = hbox({text("→") | color(Color::Yellow) | bold, text(" "),
                    content | flex}) |
              bgcolor(Color::RGB(80, 80, 80));
  } else if (item.has_selected_items) {
    content = hbox(
        {text("●") | color(Color::Yellow) | bold, text(" "), content | flex});
  } else {
    content = hbox({text("  "), content | flex});
  }
  return content;
}

ftxui::Element UIRenderer::render_module_item(const RenderItem &item,
                                              bool is_selected, bool is_focused,
                                              bool is_hovered) {
  Element text_content = text(item.text);
  bool module_selected =
      state_.view.selected_modules.count(item.identifier) > 0;
  if (item.is_disabled) {
    text_content = text_content | color(Color::GrayDark) | dim;
  } else if (module_selected) {
    text_content = text_content | color(Color::White) | bold;
  }
  Element content = text_content;

  // 统一hover效果 - 只使用hover状态，完全移除focus概念
  if (is_hovered) {
    if (module_selected) {
      content = hbox({text("→") | color(Color::Yellow) | bold, text(" "),
                      content | flex}) |
                bgcolor(Color::Green) | color(Color::White);
    } else {
      content = hbox({text("→") | color(Color::Yellow) | bold, text(" "),
                      content | flex}) |
                bgcolor(Color::RGB(80, 80, 80));
    }
  } else if (module_selected) {
    content = hbox({text("✓") | color(Color::White) | bold, text(" "),
                    content | flex}) |
              bgcolor(Color::Green) | color(Color::White);
  } else {
    content = hbox({text("  "), content | flex});
  }

  return content;
}

// ==================== 双栏事件处理 ====================

bool UIRenderer::handle_dual_column_keyboard_event(const Event &event) {
  // 🔧 更新调试信息 - 键盘按键
  if (event == Event::Tab || event == Event::TabReverse) {
    state_.debug_info.last_key = event == Event::Tab ? "Tab" : "Shift+Tab";
  } else if (event == Event::ArrowUp) {
    state_.debug_info.last_key = "Up";
  } else if (event == Event::ArrowDown) {
    state_.debug_info.last_key = "Down";
  } else if (event == Event::ArrowLeft) {
    state_.debug_info.last_key = "Left";
  } else if (event == Event::ArrowRight) {
    state_.debug_info.last_key = "Right";
  } else if (event == Event::Return) {
    state_.debug_info.last_key = "Enter";
  } else if (event == Event::Escape) {
    state_.debug_info.last_key = "Esc";
  } else if (event.is_character()) {
    state_.debug_info.last_key = event.character();
  } else {
    state_.debug_info.last_key = "Other";
  }

  // Tab/Shift+Tab键焦点切换 - 支持双向导航
  if (event == Event::Tab) {
    state_.handle_tab_focus();
    // 同步hover状态到当前活动栏位
    sync_hover_to_active_pane();
    return true;
  }
  if (event == Event::TabReverse) {
    state_.handle_tab_focus_reverse();
    // 同步hover状态到当前活动栏位
    sync_hover_to_active_pane();
    return true;
  }

  // 方向键导航 - 基于当前hover位置移动
  if (event == Event::ArrowUp) {
    if (!state_.build_button_focused) {
      if (state_.left_pane_focused) {
        // 基于hover位置移动组选择
        move_group_hover_up();
      } else {
        // 基于hover位置移动模块选择
        move_module_hover_up();
      }
      animation::RequestAnimationFrame();
    }
    return true;
  }

  if (event == Event::ArrowDown) {
    if (!state_.build_button_focused) {
      if (state_.left_pane_focused) {
        // 基于hover位置移动组选择
        move_group_hover_down();
      } else {
        // 基于hover位置移动模块选择
        move_module_hover_down();
      }
      animation::RequestAnimationFrame();
    }
    return true;
  }

  // 左右方向键 - 切换栏位焦点
  if (event == Event::ArrowLeft || event == Event::ArrowRight) {
    if (!state_.build_button_focused) {
      state_.handle_pane_switch();
      // 同步hover状态到新的活动栏位
      sync_hover_to_active_pane();
    }
    return true;
  }

  // 回车键 - 根据当前焦点栏位执行不同操作
  if (event == Event::Return) {
    if (state_.build_button_focused) {
      state_.handle_build_button();
    } else if (state_.left_pane_focused) {
      // 左栏焦点：批量toggle组内模块
      state_.handle_group_activation();
      state_.update_group_render_items(); // 更新组统计显示
    } else {
      // 右栏焦点：切换模块选择
      state_.handle_module_selection();
    }
    return true;
  }

  // 空格键 - 兼容旧的展开操作，现在用于批量toggle组内模块
  if (event == Event::Character(' ')) {
    if (!state_.build_button_focused && state_.left_pane_focused) {
      state_.handle_group_activation();
      state_.update_group_render_items(); // 更新组统计显示
    }
    return true;
  }

  // 清除选择
  if (event == Event::Character('C') || event == Event::Character('c')) {
    // 🔥 使用InteractionManager清空所有选择（包括冲突检测系统）
    if (state_.interaction_manager) {
      state_.interaction_manager->clear_all_selections();
    }

    // 🔥 同步清空传统状态
    state_.view.selected_modules.clear();

    // 更新双栏显示
    state_.update_group_render_items();
    state_.update_module_render_items();
    return true;
  }

  // 退出
  if (event == Event::Character('q') || event == Event::Escape ||
      event == Event::CtrlC) {
    throw std::runtime_error("User requested exit");
  }

  // 鼠标支持
  if (event.is_mouse()) {
    return handle_dual_column_mouse_event(
        const_cast<ftxui::Event &>(event).mouse());
  }

  return false;
}

// ==================== 双栏鼠标事件处理 ====================

bool UIRenderer::handle_dual_column_mouse_event(const Mouse &mouse) {
  // 🔧 更新调试信息 - 鼠标按钮状态
  if (mouse.button == Mouse::Left) {
    state_.debug_info.left_button = (mouse.motion == Mouse::Pressed);
  } else if (mouse.button == Mouse::Right) {
    state_.debug_info.right_button = (mouse.motion == Mouse::Pressed);
  }

  // 处理鼠标滚轮事件 - 支持模块列表滚动
  if (mouse.button == Mouse::WheelUp || mouse.button == Mouse::WheelDown) {
    return handle_mouse_wheel(mouse);
  }

  if (mouse.motion == Mouse::Moved) {
    return handle_dual_column_mouse_move(mouse);
  } else if (mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed) {
    return handle_dual_column_mouse_click(mouse);
  } else if (mouse.button == Mouse::Right && mouse.motion == Mouse::Pressed) {
    // 右键点击事件 - 可以扩展功能
    return false;
  }
  return false;
}

bool UIRenderer::handle_dual_column_mouse_move(const Mouse &mouse) {
  // 🔧 更新调试信息
  state_.debug_info.mouse_x = mouse.x;
  state_.debug_info.mouse_y = mouse.y;

  bool state_changed = false;

  // 保存当前hover状态用于比较
  int old_group_hover = state_.group_hover_index;
  int old_module_hover = state_.module_hover_index;
  bool old_build_hover = false;

  // 重置所有hover状态 - 确保全局只有一个hover
  state_.group_hover_index = -1;
  state_.module_hover_index = -1;
  // 底部按钮由组件管理，无需hover状态

  // 使用统一的坐标映射系统进行hover检测
  ElementInfo element =
      state_.coordinate_mapper.get_element_at(mouse.y, mouse.x);

  // 🔧 更新调试信息 - 元素信息
  state_.debug_info.element_type = static_cast<int>(element.type);
  state_.debug_info.element_index = element.render_item_index;

  // 根据元素类型设置对应的hover状态（确保只有一个）
  switch (element.type) {
  case ElementType::GROUP_HEADER:
    // 左栏组项目hover
    if (element.render_item_index >= 0 &&
        element.render_item_index <
            static_cast<int>(state_.group_render_items.size())) {
      state_.group_hover_index = element.render_item_index;
    }
    break;

  case ElementType::MODULE_ITEM:
    // 右栏模块项目hover
    if (element.render_item_index >= 0 &&
        element.render_item_index <
            static_cast<int>(state_.module_render_items.size())) {
      state_.module_hover_index = element.render_item_index;
    }
    break;

    // 底部按钮由组件管理，不在映射中

  case ElementType::UNKNOWN:
  default:
    // 没有hover到任何交互元素，保持重置后的状态
    break;
  }

  // 检测状态变化
  if (old_group_hover != state_.group_hover_index ||
      old_module_hover != state_.module_hover_index) {
    state_changed = true;

    // 🔥 hover时实时更新details区域信息
    update_details_on_hover();
  }

  return state_changed;
}

bool UIRenderer::handle_dual_column_mouse_click(const Mouse &mouse) {
  // 使用统一的坐标映射系统进行点击检测
  ElementInfo element =
      state_.coordinate_mapper.get_element_at(mouse.y, mouse.x);

  // 根据元素类型执行对应的点击处理
  switch (element.type) {
  case ElementType::GROUP_HEADER:
    // 左栏组项目点击
    if (element.render_item_index >= 0 &&
        element.render_item_index <
            static_cast<int>(state_.group_render_items.size())) {
      // 设置组选择索引并触发组激活
      state_.group_selection_index = element.render_item_index;
      state_.left_pane_focused = true; // 点击左栏时设置左栏焦点
      return state_.handle_group_activation();
    }
    break;

  case ElementType::MODULE_ITEM:
    // 右栏模块项目点击
    if (element.render_item_index >= 0 &&
        element.render_item_index <
            static_cast<int>(state_.module_render_items.size())) {
      // 设置模块选择索引并触发模块选择
      state_.module_selection_index = element.render_item_index;
      state_.left_pane_focused = false; // 点击右栏时设置右栏焦点
      return state_.handle_module_selection();
    }
    break;

    // 底部按钮由组件管理，不在映射中

  case ElementType::UNKNOWN:
  default:
    // 点击到非交互区域，不处理
    break;
  }

  return false;
}

// ==================== 鼠标滚轮事件处理 ====================

bool UIRenderer::handle_mouse_wheel(const Mouse &mouse) {
  // 🔧 更新调试信息 - 记录鼠标滚轮方向
  if (mouse.button == Mouse::WheelUp) {
    state_.debug_info.last_scroll = "Up";
  } else if (mouse.button == Mouse::WheelDown) {
    state_.debug_info.last_scroll = "Down";
  }

  // 检查鼠标位置是否在右栏（模块列表）区域
  ElementInfo element =
      state_.coordinate_mapper.get_element_at(mouse.y, mouse.x);

  // 如果鼠标在模块区域或者右栏焦点激活时，处理滚轮事件
  bool in_module_area = (element.type == ElementType::MODULE_ITEM);
  bool right_pane_active = !state_.left_pane_focused;

  if (in_module_area || right_pane_active) {
    int scroll_direction = 0;

    if (mouse.button == Mouse::WheelUp) {
      // 向上滚动：向前滚动列表（显示较早的项目）
      scroll_direction = -3; // 一次滚动3行
    } else if (mouse.button == Mouse::WheelDown) {
      // 向下滚动：向后滚动列表（显示较晚的项目）
      scroll_direction = 1; // 一次滚动3行
    }

    if (scroll_direction != 0) {
      // 执行滚动
      state_.scroll_module_list(scroll_direction);

      // 如果选择项不在可视范围内，调整选择位置
      if (state_.module_selection_index < state_.module_scroll_offset) {
        state_.module_selection_index = state_.module_scroll_offset;
        state_.module_hover_index = state_.module_selection_index;
      } else if (state_.module_selection_index >=
                 state_.module_scroll_offset + state_.module_visible_count) {
        state_.module_selection_index =
            state_.module_scroll_offset + state_.module_visible_count - 1;
        state_.module_hover_index = state_.module_selection_index;
      }

      // 更新详情信息
      update_details_on_hover();

      // 重建坐标映射以反映滚动后的新位置
      rebuild_dual_column_coordinate_mapping();

      return true; // 处理了滚轮事件
    }
  }

  return false; // 未处理滚轮事件
}

// ==================== UI区域行数动态计算 ====================

/**
 * @brief 计算调试窗口的实际内容行数
 * 基于调试信息开关动态计算实际显示的行数
 */
int UIRenderer::calculate_debug_content_lines() const {
  // 🔧 收集所有启用的调试元素
  std::vector<bool> enabled_elements = {
      state_.debug_info.show_mouse_coords,  // Mouse: (0,67)
      state_.debug_info.show_mouse_buttons, // Buttons: L0 R1
      state_.debug_info.show_mouse_scroll,  // Scroll: Up
      state_.debug_info.show_keyboard,      // Key: Other
      state_.debug_info.show_element_info,  // Element: Type=6 Index=-1
      state_.debug_info.show_build_coords,  // Build: (44,61)
      state_.debug_info.show_module_stats,  // Modules: 16 Groups: 6
      state_.debug_info.show_terminal_size, // Terminal: 89x73
      state_.debug_info.show_build_hover    // BuildHover: N
  };

  // 计算启用的元素总数
  int enabled_count = 0;
  for (bool enabled : enabled_elements) {
    if (enabled)
      enabled_count++;
  }

  // 如果没有启用任何元素，调试窗口完全消失
  if (enabled_count == 0) {
    return 0;
  }

  // 按行填充：每行3个元素，计算需要的行数
  const int elements_per_row = 3;
  return (enabled_count + elements_per_row - 1) / elements_per_row; // 向上取整
}

/**
 * @brief 计算按键指南的实际内容行数
 * 当前按键指南是四列布局，统一为2行
 */
int UIRenderer::calculate_key_guide_content_lines() const {
  // 四列布局，每列统一2行：
  // 第1列: ↑↓←→, Tab
  // 第2列: Enter, Space/C
  // 第3列: 鼠标, 滚轮/点击
  // 第4列: q/Esc, Shift+Tab
  return 2;
}

// ==================== 双栏坐标映射和调试 ====================

void UIRenderer::rebuild_dual_column_coordinate_mapping() {
  // 动态计算Y坐标偏移量 - 基于UI结构而非硬编码
  // 结构分析：title(1) + separator(1) + left_title(1) + left_separator(1) = 4
  const int left_content_start_y = 6;  // 左栏内容开始位置
  const int right_content_start_y = 6; // 右栏内容开始位置（与左栏对齐）

  // 动态计算X坐标边界 - 根据实际终端尺寸和布局比例
  auto [left_width, right_start] = calculate_dynamic_column_boundaries();

  // 注：编译按钮坐标现在在鼠标hover时实时捕获

  // 使用动态参数重建双栏坐标映射
  state_.coordinate_mapper.rebuild_dual_column_mapping(
      state_.group_render_items,   // 左栏：组列表
      state_.module_render_items,  // 右栏：模块列表
      left_content_start_y,        // 左栏内容起始Y坐标
      right_content_start_y,       // 右栏内容起始Y坐标
      left_width,                  // 动态计算的左栏宽度边界
      right_start,                 // 动态计算的右栏起始X坐标
      state_.module_scroll_offset, // 右栏滚动偏移
      state_.module_visible_count  // 右栏可见数量
  );
}

std::pair<int, int> UIRenderer::calculate_dynamic_column_boundaries() {
  // 根据用户发现的精确公式实现
  // 左栏起始：x = 2（固定，由边框结构决定）
  // 右栏起始：x = int((W-1)/2)，其中W为终端宽度

  int terminal_width = 80; // 默认值

  // 获取实际终端宽度
  try {
    auto terminal_size = ftxui::Terminal::Size();
    terminal_width = terminal_size.dimx;
  } catch (...) {
    terminal_width = 90; // 使用测试环境的宽度作为fallback
  }

  // 应用用户发现的通用公式
  const int left_column_start_x = 2; // 固定起始位置
  const int right_column_start_x =
      (terminal_width - 1) / 2; // 整数除法自动向下取整

  // 左栏结束位置：右栏起始前的一个位置减去边框间隔
  const int left_column_end_x = right_column_start_x - 3; // 为边框预留空间

  // 最小宽度保护
  if (left_column_end_x <= left_column_start_x + 20 ||
      terminal_width - right_column_start_x < 25) {
    // 空间太小，使用保守值
    return {35, 40};
  }

  return {left_column_end_x, right_column_start_x};
}

// ==================== 调试窗口渲染 ====================

ftxui::Element UIRenderer::render_debug_window() {
  // 🔧 准备所有调试元素的数据
  struct DebugElement {
    bool enabled;
    std::string content;
    Color color;
  };

  // 🔥 获取终端尺寸（用于显示）
  int terminal_width = -1;
  int terminal_height = -1;
  try {
    auto terminal_size = ftxui::Terminal::Size();
    terminal_width = terminal_size.dimx;
    terminal_height = terminal_size.dimy;
  } catch (...) {
    // 获取失败
  }

  // 按顺序定义所有9个调试元素
  std::vector<DebugElement> debug_elements = {
      {// 1. Mouse coordinates
       state_.debug_info.show_mouse_coords,
       "Mouse: (" + std::to_string(state_.debug_info.mouse_x) + "," +
           std::to_string(state_.debug_info.mouse_y) + ")",
       Color::Cyan},
      {// 2. Mouse buttons
       state_.debug_info.show_mouse_buttons,
       "Buttons: L" + std::string(state_.debug_info.left_button ? "1" : "0") +
           " R" + std::string(state_.debug_info.right_button ? "1" : "0"),
       Color::Yellow},
      {// 3. Mouse scroll
       state_.debug_info.show_mouse_scroll,
       "Scroll: " + state_.debug_info.last_scroll, Color::Magenta},
      {// 4. Keyboard
       state_.debug_info.show_keyboard, "Key: " + state_.debug_info.last_key,
       Color::Green},
      {// 5. Element info
       state_.debug_info.show_element_info,
       "Element: Type=" + std::to_string(state_.debug_info.element_type) +
           " Index=" + std::to_string(state_.debug_info.element_index),
       Color::Magenta},
      {// 6. Build coordinates
       state_.debug_info.show_build_coords,
       "Build: (" + std::to_string(state_.debug_info.build_button_x) + "," +
           std::to_string(state_.debug_info.build_button_y) + ")",
       Color::Red},
      {// 7. Module statistics
       state_.debug_info.show_module_stats,
       "Modules: " + std::to_string(state_.module_render_items.size()) +
           " Groups: " + std::to_string(state_.group_render_items.size()),
       Color::White},
      {// 8. Terminal size
       state_.debug_info.show_terminal_size,
       "Terminal: " + std::to_string(terminal_width) + "x" +
           std::to_string(terminal_height),
       Color::Cyan},
      {// 9. Build hover
       state_.debug_info.show_build_hover,
       "BuildHover: " + std::string(state_.build_button_hovered ? "Y" : "N"),
       Color::Yellow}};

  // 🔧 收集启用的元素
  std::vector<Element> enabled_elements;
  for (const auto &debug_elem : debug_elements) {
    if (debug_elem.enabled) {
      enabled_elements.push_back(text(debug_elem.content) |
                                 color(debug_elem.color));
    }
  }

  // 如果没有启用任何元素，显示提示信息
  if (enabled_elements.empty()) {
    enabled_elements.push_back(text("[调试信息关闭]") | color(Color::GrayDark));
  }

  // 🔧 按行排列：每行3个元素
  const int elements_per_row = 3;
  std::vector<Element> rows;

  for (size_t i = 0; i < enabled_elements.size(); i += elements_per_row) {
    std::vector<Element> row_elements;

    // 添加当前行的元素（最多3个）
    for (int j = 0; j < elements_per_row && (i + j) < enabled_elements.size();
         ++j) {
      if (j > 0) {
        row_elements.push_back(text(" | ") | color(Color::GrayLight));
      }
      row_elements.push_back(enabled_elements[i + j] | flex);
    }

    // 如果这一行不满3个元素，用空白填充
    int current_row_elements = std::min(
        elements_per_row, static_cast<int>(enabled_elements.size() - i));
    for (int j = current_row_elements; j < elements_per_row; ++j) {
      if (j > 0) {
        row_elements.push_back(text(" | ") | color(Color::GrayLight));
      }
      row_elements.push_back(text("") | flex);
    }

    rows.push_back(hbox(row_elements));
  }
  // 动态计算调试窗口高度：边框(2) + 实际行数
  const int actual_content_lines = static_cast<int>(rows.size());
  const int debug_window_height = 2 + actual_content_lines;

  return vbox(rows) | border | bgcolor(Color::RGB(20, 20, 20)) |
         size(HEIGHT, EQUAL, debug_window_height);
}

// ==================== 键盘导航辅助方法 ====================

void UIRenderer::move_group_hover_up() {
  // 清除其他栏位的hover状态，确保全局只有一个hover
  state_.module_hover_index = -1;

  if (state_.group_hover_index <= 0) {
    // 已经在顶部，循环到底部
    state_.group_hover_index =
        static_cast<int>(state_.group_render_items.size()) - 1;
  } else {
    state_.group_hover_index--;
  }
  // 同步选择索引到hover位置
  state_.group_selection_index = state_.group_hover_index;
  // 立即更新详情信息
  update_details_on_hover();
}

void UIRenderer::move_group_hover_down() {
  // 清除其他栏位的hover状态，确保全局只有一个hover
  state_.module_hover_index = -1;

  if (state_.group_hover_index >=
      static_cast<int>(state_.group_render_items.size()) - 1) {
    // 已经在底部，循环到顶部
    state_.group_hover_index = 0;
  } else {
    state_.group_hover_index++;
  }
  // 同步选择索引到hover位置
  state_.group_selection_index = state_.group_hover_index;
  // 立即更新详情信息
  update_details_on_hover();
}

void UIRenderer::move_module_hover_up() {
  // 清除其他栏位的hover状态，确保全局只有一个hover
  state_.group_hover_index = -1;

  if (state_.module_hover_index <= 0) {
    // 已经在顶部，循环到底部
    state_.module_hover_index =
        static_cast<int>(state_.module_render_items.size()) - 1;
  } else {
    state_.module_hover_index--;
  }
  // 同步选择索引到hover位置
  state_.module_selection_index = state_.module_hover_index;
  // 立即更新详情信息
  update_details_on_hover();
  // 确保选择项在滚动视图中可见
  state_.ensure_module_selection_visible();
}

void UIRenderer::move_module_hover_down() {
  // 清除其他栏位的hover状态，确保全局只有一个hover
  state_.group_hover_index = -1;

  if (state_.module_hover_index >=
      static_cast<int>(state_.module_render_items.size()) - 1) {
    // 已经在底部，循环到顶部
    state_.module_hover_index = 0;
  } else {
    state_.module_hover_index++;
  }
  // 同步选择索引到hover位置
  state_.module_selection_index = state_.module_hover_index;
  // 立即更新详情信息
  update_details_on_hover();
  // 确保选择项在滚动视图中可见
  state_.ensure_module_selection_visible();
}

void UIRenderer::sync_hover_to_active_pane() {
  // 清除所有hover状态，然后根据活动栏位设置单一hover
  state_.group_hover_index = -1;
  state_.module_hover_index = -1;

  if (state_.left_pane_focused) {
    // 左栏有焦点，设置组hover到当前选择位置
    state_.group_hover_index = state_.group_selection_index;
    // 如果hover位置超出范围，调整到有效范围
    if (state_.group_hover_index < 0 ||
        state_.group_hover_index >=
            static_cast<int>(state_.group_render_items.size())) {
      state_.group_hover_index = 0;
      state_.group_selection_index = 0;
    }
  } else {
    // 右栏有焦点，设置模块hover到当前选择位置
    state_.module_hover_index = state_.module_selection_index;
    // 如果hover位置超出范围，调整到有效范围
    if (state_.module_hover_index < 0 ||
        state_.module_hover_index >=
            static_cast<int>(state_.module_render_items.size())) {
      state_.module_hover_index = 0;
      state_.module_selection_index = 0;
    }
  }
  // 立即更新详情信息
  update_details_on_hover();
}

// ==================== hover时更新details ====================

void UIRenderer::update_details_on_hover() {
  // 🔥 hover时实时更新details区域信息
  // 清空现有信息
  state_.current_item_description.clear();
  state_.current_item_details.clear();

  // 检查是否有组被hover
  if (state_.group_hover_index >= 0 &&
      state_.group_hover_index <
          static_cast<int>(state_.group_render_items.size())) {
    // 显示hover组的信息
    const auto &group_item =
        state_.group_render_items[state_.group_hover_index];
    const ModuleGroup *group = state_.find_group(group_item.identifier);
    if (group) {
      state_.current_item_description =
          group->description.empty() ? "NULL" : group->description;
      state_.current_item_details =
          "包含 " + std::to_string(group->modules.size()) + " 个模块";
    }
    return;
  }

  // 检查是否有模块被hover
  if (state_.module_hover_index >= 0 &&
      state_.module_hover_index <
          static_cast<int>(state_.module_render_items.size())) {
    // 显示hover模块的信息
    const auto &module_item =
        state_.module_render_items[state_.module_hover_index];
    const Module *module = state_.find_module(module_item.identifier);
    if (module) {
      // 设置模块描述
      state_.current_item_description =
          module->description.empty() ? "NULL" : module->description;

      // 生成详细信息 - 始终显示所有字段，即使为空
      std::string details;

      // 依赖信息
      details += "依赖: ";
      if (module->dependencies.empty()) {
        details += "NULL";
      } else {
        for (size_t i = 0; i < module->dependencies.size(); ++i) {
          if (i > 0)
            details += ", ";
          details += module->dependencies[i];
        }
      }
      details += "\n";

      // 冲突信息
      details += "冲突: ";
      if (module->conflicts_with.empty()) {
        details += "NULL";
      } else {
        for (size_t i = 0; i < module->conflicts_with.size(); ++i) {
          if (i > 0)
            details += ", ";
          details += module->conflicts_with[i];
        }
      }
      details += "\n";

      // 路径信息
      details += "路径: " +
                 (module->source_path.empty() ? "NULL" : module->source_path);

      state_.current_item_details = details;
    }
    return;
  }

  // 如果没有hover任何item，根据当前焦点显示selection的信息（回退到原有逻辑）
  state_.update_current_item_info();
}

// ==================== 按键指南渲染 ====================

ftxui::Element UIRenderer::render_key_guide() {
  // 按键提示 - 动态四列版本（统一2行）
  return hbox({// 第一列 - 导航
               vbox({text("↑↓←→") | color(Color::Cyan),
                     text("Tab") | color(Color::Cyan)}) |
                   flex,
               text("  ") | color(Color::Default),
               vbox({text("导航") | color(Color::GrayLight),
                     text("焦点") | color(Color::GrayLight)}) |
                   flex,
               text("   ") | color(Color::Default),
               // 第二列 - 操作
               vbox({text("Enter") | color(Color::Cyan),
                     text("Space/C") | color(Color::Cyan)}) |
                   flex,
               text("  ") | color(Color::Default),
               vbox({text("选择") | color(Color::GrayLight),
                     text("批量/清空") | color(Color::GrayLight)}) |
                   flex,
               text("   ") | color(Color::Default),
               // 第三列 - 鼠标
               vbox({text("鼠标") | color(Color::Cyan),
                     text("滚轮/点击") | color(Color::Cyan)}) |
                   flex,
               text("  ") | color(Color::Default),
               vbox({text("悬停") | color(Color::GrayLight),
                     text("滚动/交互") | color(Color::GrayLight)}) |
                   flex,
               text("   ") | color(Color::Default),
               // 第四列 - 退出
               vbox({text("q/Esc") | color(Color::Cyan),
                     text("Shift+Tab") | color(Color::Cyan)}) |
                   flex,
               text("  ") | color(Color::Default),
               vbox({text("退出") | color(Color::GrayLight),
                     text("反向") | color(Color::GrayLight)}) |
                   flex}) |
         border | bgcolor(Color::RGB(30, 30, 30));
}

} // namespace sunray_tui
