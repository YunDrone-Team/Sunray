#include "tui_render.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/component/component_options.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"

using namespace ftxui;

namespace sunray_tui {

ftxui::Component UIRenderer::create_component() {
  // 创建按钮组件
  create_buttons();

  // ==================== 双栏滚动列表组件 ===================
  // 左栏 - 组列表
  Component left_list = Container::Vertical({});

  // 右栏 - 模块列表（带滚动）
  Component right_list = Container::Vertical({}, &state_.module_selection_index);

  // 包装右栏为滚动视图
  auto right_scrollable = Renderer(right_list, [this] {
    // 计算滚动范围
    int total_modules = static_cast<int>(state_.module_render_items.size());
    if (total_modules == 0) {
      return vbox({text("(无模块)") | center | color(Color::GrayDark)});
    }

    int visible_count = state_.module_visible_count;
    int scroll_offset = state_.module_scroll_offset;

    // 确保滚动范围有效
    if (scroll_offset < 0)
      scroll_offset = 0;
    if (scroll_offset >= total_modules)
      scroll_offset = std::max(0, total_modules - visible_count);
    if (visible_count > total_modules)
      visible_count = total_modules;

    // 渲染可见的模块项
    std::vector<Element> visible_elements;
    for (int i = 0; i < visible_count && (scroll_offset + i) < total_modules;
         ++i) {
      int module_idx = scroll_offset + i;
      const auto &item = state_.module_render_items[module_idx];

      bool is_selected = (state_.module_selection_index == module_idx);
      bool is_hovered = (state_.module_hover_index == module_idx);

      visible_elements.push_back(
          render_module_item(item, is_selected, false, is_hovered));
    }

    return vbox(visible_elements);
  });

  // 创建双栏布局
  Component dual_column = Container::Horizontal({
      // 左栏：组列表
      Renderer([this] {
        std::vector<Element> elements;
        for (size_t i = 0; i < state_.group_render_items.size(); ++i) {
          const auto &item = state_.group_render_items[i];
          bool is_selected = (state_.group_selection_index == (int)i);
          bool is_hovered = (state_.group_hover_index == (int)i);

          elements.push_back(
              render_group_item(item, is_selected, false, is_hovered));
        }

        return vbox(elements) | flex;
      }),

      // 右栏：模块列表（带滚动）
      right_scrollable});

  // 创建主容器 - 包含双栏、详情区、按钮行和调试信息
  Component main_container = Container::Vertical({dual_column, buttons_row_});

  // ==================== 事件处理包装器 ====================
  Component event_handler =
      CatchEvent(main_container, [this](Event event) -> bool {
        // 先处理键盘事件
        if (handle_dual_column_keyboard_event(event)) {
          // 重建坐标映射以反映状态变化
          rebuild_dual_column_coordinate_mapping();
          return true;
        }
        return false;
      });

  // ==================== 最终渲染器 ====================
  Component final_renderer = Renderer(event_handler, [this] {
    // 重建坐标映射 - 在每次渲染前更新
    rebuild_dual_column_coordinate_mapping();
    // 推进构建按钮的警告闪烁计时，确保点击后能自动恢复
    state_.update_build_warning_flash();
    if (state_.build_warning_flash_active) {
      animation::RequestAnimationFrame();
    }

    // 动态计算UI区域高度
    int debug_lines = calculate_debug_content_lines();
    int key_guide_lines = calculate_key_guide_content_lines();

    // 构建主要内容区域
    std::vector<Element> main_content;

    // 标题
    main_content.push_back(
        text("Sunray Build System - TUI") | bold | center);

    main_content.push_back(separator());

    // 获取终端尺寸并计算双栏布局
    int terminal_width = 80;
    int terminal_height = 24;
    try {
      auto terminal_size = ftxui::Terminal::Size();
      terminal_width = terminal_size.dimx;
      terminal_height = terminal_size.dimy;
    } catch (...) {}
    
    // 计算固定UI元素占用的高度 - 标题(1) + 分隔符(1) + 栏目标题(1) + 分隔符(1) + 描述(4) + 分隔符(1) + 按钮(1) + 分隔符(1) + 按键指南(3) + 调试(5) + 边框(2)
    const int fixed_ui_height = 1 + 1 + 1 + 1 + 4 + 1 + 1 + 1 + 3 + 5 + 2; // = 21行
    const int available_height_for_columns = std::max(8, terminal_height - fixed_ui_height);

    // 创建左栏内容
    std::vector<Element> left_column_elements;
    left_column_elements.push_back(text("模块组") | bold | center);
    left_column_elements.push_back(separator());
    for (size_t i = 0; i < state_.group_render_items.size(); ++i) {
      const auto &item = state_.group_render_items[i];
      bool is_selected = (state_.group_selection_index == (int)i);
      bool is_hovered = (state_.group_hover_index == (int)i);
      left_column_elements.push_back(
          render_group_item(item, is_selected, false, is_hovered));
    }
    if (state_.group_render_items.empty()) {
      left_column_elements.push_back(text("没有可用的模块组") | dim | center);
    }

    // 创建右栏内容
    std::vector<Element> right_column_elements;
    right_column_elements.push_back(text("所有模块") | bold | center);
    right_column_elements.push_back(separator());
    
    // 重新计算可见数量并填充右栏内容
    state_.calculate_module_visible_count();
    state_.ensure_module_selection_visible();
    
    if (!state_.module_render_items.empty()) {
      const int start_index = state_.module_scroll_offset;
      const int end_index = std::min(
          static_cast<int>(state_.module_render_items.size()),
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
      const int total_modules = static_cast<int>(state_.module_render_items.size());
      if (total_modules > state_.module_visible_count) {
        std::string scroll_info = "(" + std::to_string(start_index + 1) + "-" + 
                                  std::to_string(end_index) + "/" + 
                                  std::to_string(total_modules) + ")";
        right_column_elements.push_back(
            text(scroll_info) | dim | color(Color::GrayLight) | center);
      }
    } else {
      right_column_elements.push_back(text("没有可用的模块") | dim | center);
    }

    // 创建双栏布局 - 使用固定的50/50分割和动态高度
    Element left_column = vbox(left_column_elements) | border |
                          size(WIDTH, EQUAL, terminal_width / 2 - 1) |
                          size(HEIGHT, EQUAL, available_height_for_columns) | flex;
    Element right_column = vbox(right_column_elements) | border |
                           size(WIDTH, EQUAL, terminal_width / 2 - 1) |
                           size(HEIGHT, EQUAL, available_height_for_columns) | flex;
    main_content.push_back(hbox({left_column, right_column}));
    main_content.push_back(separator());

    // 显示当前选中项的信息 - 固定4行高度
    main_content.push_back(
        text("描述: " + (state_.current_item_description.empty()
                             ? "NULL"
                             : state_.current_item_description)) |
        bold);

    // 收集详细信息行，确保总共4行
    std::vector<Element> detail_lines;
    if (!state_.current_item_details.empty()) {
      std::string details = state_.current_item_details;
      size_t pos = 0;
      while (pos < details.length() && detail_lines.size() < 3) {
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
      }
    }

    // 确保总共3行详细信息（加上描述行总共4行）
    while (detail_lines.size() < 3) {
      detail_lines.push_back(text(" "));
    }

    for (const auto &line : detail_lines) {
      main_content.push_back(line);
    }

    // 编译按钮区域
    main_content.push_back(separatorLight());

    main_content.push_back(buttons_row_->Render() | center);

    // 清理失败时显示详细输出（如果有）
    if (clear_state_ == CleanState::Error && !clear_output_.empty() &&
        clear_success_frames_remaining_ > 0) {
      main_content.push_back(separatorLight());
      main_content.push_back(
          vbox({text("清理失败 (退出码: " + std::to_string(clear_exit_code_) +
                     ")")
                    | color(Color::Red) | bold,
                text(clear_output_) | color(Color::GrayLight)}) |
          border | bgcolor(Color::RGB(60, 20, 20)));
    }

    // 按键指南（如果启用）
    if (key_guide_lines > 0) {
      main_content.push_back(separatorLight());
      main_content.push_back(render_key_guide());
    }

    // 调试信息（如果启用）
    if (debug_lines > 0) {
      main_content.push_back(separatorLight());
      main_content.push_back(render_debug_window());
    }

    // 修复：在渲染器内部更新清理按钮计时器，确保每帧都执行
    if (clear_success_frames_remaining_ > 0) {
      clear_success_frames_remaining_--;
      if (clear_success_frames_remaining_ <= 0) {
        clear_state_ = CleanState::Idle;  // 恢复为空闲状态，按钮文本回到"清除构建"
        clear_output_.clear();
      }
      // 触发重新渲染以更新按钮状态
      animation::RequestAnimationFrame();
    }

    return vbox(main_content);
  });

  return final_renderer;
}

} // namespace sunray_tui
