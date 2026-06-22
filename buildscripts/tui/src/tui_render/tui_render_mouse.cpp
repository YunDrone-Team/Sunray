#include "tui_render.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/screen/terminal.hpp"

using namespace ftxui;

namespace sunray_tui {

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
  // 同步清除按钮悬停标志；若鼠标在按钮上，事件将继续传播并由Hoverable重新设置
  start_button_hovered_ = false;
  clear_button_hovered_ = false;
  // 鼠标移动即退出按钮键盘焦点样式，由 Hoverable 决定是否高亮
  state_.build_button_focused = false;
  // 统一高亮：初始认为不在按钮上，由列表命中设置 pointer_hover
  highlight_mgr_.set_pointer_hover(std::nullopt);
  // 底部按钮由组件管理，不参与坐标映射

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
      highlight_mgr_.set_pointer_hover(InteractiveId::Group(state_.group_hover_index));
    }
    break;

  case ElementType::MODULE_ITEM:
    // 右栏模块项目hover
    if (element.render_item_index >= 0 &&
        element.render_item_index <
            static_cast<int>(state_.module_render_items.size())) {
      state_.module_hover_index = element.render_item_index;
      highlight_mgr_.set_pointer_hover(InteractiveId::Module(state_.module_hover_index));
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

  // 若鼠标位于按钮盒子范围，覆盖为按钮高亮（按钮优先于列表）
  auto within = [](const ftxui::Box& b, int x, int y) {
    return x >= b.x_min && x <= b.x_max && y >= b.y_min && y <= b.y_max;
  };
  if (within(start_button_box_, mouse.x, mouse.y)) {
    state_.group_hover_index = -1;
    state_.module_hover_index = -1;
    highlight_mgr_.set_pointer_hover(InteractiveId::Start());
    state_changed = true;
  } else if (within(clear_button_box_, mouse.x, mouse.y)) {
    state_.group_hover_index = -1;
    state_.module_hover_index = -1;
    highlight_mgr_.set_pointer_hover(InteractiveId::Clear());
    state_changed = true;
  }

  // 重新计算唯一高亮
  highlight_mgr_.compute_highlighted();

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
      if (!state_.is_selectable_group_index(state_.group_selection_index)) {
        return false;
      }
      state_.left_pane_focused = true;  // 点击左栏时设置左栏焦点
      state_.build_button_focused = false;  // 鼠标点击列表时，按钮失焦
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
      if (!state_.is_selectable_module_index(state_.module_selection_index)) {
        return false;
      }
      state_.left_pane_focused = false;  // 点击右栏时设置右栏焦点
      state_.build_button_focused = false;  // 鼠标点击列表时，按钮失焦
      return state_.handle_module_selection();
    }
    break;

    // 底部按钮由组件管理，不在映射中

  case ElementType::UNKNOWN:
  default:
    // 点击到非交互区域，不处理
    break;
  }

  // 按钮点击命中（通过反射 Box）
  auto within = [](const ftxui::Box& b, int x, int y) {
    return x >= b.x_min && x <= b.x_max && y >= b.y_min && y <= b.y_max;
  };
  if (within(start_button_box_, mouse.x, mouse.y)) {
    // Start 按钮：触发构建或警告
    if (!state_.view.selected_modules.empty()) {
      state_.handle_build_button();
    } else {
      state_.trigger_build_warning_flash();
      ftxui::animation::RequestAnimationFrame();
    }
    return true;
  }
  if (within(clear_button_box_, mouse.x, mouse.y)) {
    // Clear 按钮：复用已有逻辑
    trigger_clear_build_clean();
    return true;
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

  // 检查鼠标位置所在栏。label 行和空白行也允许滚动。
  ElementInfo element =
      state_.coordinate_mapper.get_element_at(mouse.y, mouse.x);
  int terminal_width = 80;
  try {
    auto terminal_size = ftxui::Terminal::Size();
    terminal_width = terminal_size.dimx;
  } catch (...) {}
  const int right_column_start_x = (terminal_width - 1) / 2;
  const bool in_group_area = mouse.x < right_column_start_x;

  int scroll_direction = 0;
  if (mouse.button == Mouse::WheelUp) {
    // 向上滚动：向前滚动列表（显示较早的项目）
    scroll_direction = -3;
  } else if (mouse.button == Mouse::WheelDown) {
    // 向下滚动：向后滚动列表（显示较晚的项目）
    scroll_direction = 3;
  }

  if (scroll_direction == 0) {
    return false;
  }

  if (in_group_area) {
    state_.scroll_group_list(scroll_direction);

    if (state_.group_selection_index < state_.group_scroll_offset) {
      state_.group_selection_index = state_.find_next_selectable_group_index(
          state_.group_scroll_offset, 1);
      state_.group_hover_index = state_.group_selection_index;
    } else if (state_.group_selection_index >=
               state_.group_scroll_offset + state_.group_visible_count) {
      state_.group_selection_index = state_.find_next_selectable_group_index(
          state_.group_scroll_offset + state_.group_visible_count - 1, -1);
      state_.group_hover_index = state_.group_selection_index;
    }

    state_.module_hover_index = -1;
    update_details_on_hover();
    rebuild_dual_column_coordinate_mapping();
    return true;
  }

  // 如果鼠标在右栏区域或者右栏焦点激活时，处理模块滚轮事件
  bool in_module_area =
      (mouse.x >= right_column_start_x) ||
      (element.type == ElementType::MODULE_ITEM) ||
      (element.type == ElementType::LABEL_HEADER);
  bool right_pane_active = !state_.left_pane_focused;

  if (in_module_area || right_pane_active) {
    // 执行滚动
    state_.scroll_module_list(scroll_direction);

    // 如果选择项不在可视范围内，调整选择位置
    if (state_.module_selection_index < state_.module_scroll_offset) {
      state_.module_selection_index = state_.find_next_selectable_module_index(
          state_.module_scroll_offset, 1);
      state_.module_hover_index = state_.module_selection_index;
    } else if (state_.module_selection_index >=
               state_.module_scroll_offset + state_.module_visible_count) {
      state_.module_selection_index = state_.find_next_selectable_module_index(
          state_.module_scroll_offset + state_.module_visible_count - 1, -1);
      state_.module_hover_index = state_.module_selection_index;
    }

    state_.group_hover_index = -1;
    // 更新详情信息
    update_details_on_hover();

    // 重建坐标映射以反映滚动后的新位置
    rebuild_dual_column_coordinate_mapping();

    return true; // 处理了滚轮事件
  }

  return false; // 未处理滚轮事件
}

} // namespace sunray_tui
