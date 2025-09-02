#include "tui_render.hpp"
#include "ftxui/component/event.hpp"
#include "ftxui/component/animation.hpp"

using namespace ftxui;

namespace sunray_tui {

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
    // 进入/离开按钮焦点时，统一全局“hover”归属
    if (state_.build_button_focused) {
      // 进入按钮区：清除列表 hover 和按钮 hover，由鼠标 Hover 决定样式
      state_.group_hover_index = -1;
      state_.module_hover_index = -1;
      start_button_hovered_ = false;
      clear_button_hovered_ = false;
    } else {
      // 离开按钮区域，清除按钮 hover，并同步到当前活动栏位
      start_button_hovered_ = false;
      clear_button_hovered_ = false;
      sync_hover_to_active_pane();
    }
    animation::RequestAnimationFrame();
    return true;
  }
  if (event == Event::TabReverse) {
    state_.handle_tab_focus_reverse();
    if (state_.build_button_focused) {
      state_.group_hover_index = -1;
      state_.module_hover_index = -1;
      start_button_hovered_ = false;
      clear_button_hovered_ = false;
    } else {
      start_button_hovered_ = false;
      clear_button_hovered_ = false;
      sync_hover_to_active_pane();
    }
    animation::RequestAnimationFrame();
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
    if (state_.build_button_focused) {
      // 在按钮行内左右移动焦点，但不设置 hover；仅用于回车触发目标
      state_.button_focus_index = (event == Event::ArrowLeft) ? 0 : 1;
      start_button_hovered_ = false;
      clear_button_hovered_ = false;
      animation::RequestAnimationFrame();
    } else {
      state_.handle_pane_switch();
      // 同步hover状态到新的活动栏位
      sync_hover_to_active_pane();
    }
    return true;
  }

  // 回车键 - 根据当前焦点栏位执行不同操作
  if (event == Event::Return) {
    if (state_.build_button_focused) {
      if (state_.button_focus_index == 0) {
        state_.handle_build_button();
      } else {
        // 触发清除构建
        trigger_clear_build_clean();
      }
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

} // namespace sunray_tui
