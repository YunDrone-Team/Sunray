#include "tui/render/tui_highlight_manager.hpp"

using namespace std::chrono;

namespace sunray_tui {

void UIHighlightManager::register_box(const InteractiveId& id, const ftxui::Box& box) {
  boxes_[id] = box;
}

void UIHighlightManager::clear_boxes_of_kind(InteractiveKind kind) {
  for (auto it = boxes_.begin(); it != boxes_.end();) {
    if (it->first.kind == kind)
      it = boxes_.erase(it);
    else
      ++it;
  }
}

void UIHighlightManager::set_pointer_hover(std::optional<InteractiveId> id) {
  state_.pointer_hover = id;
  state_.last_input = id ? InputSource::Mouse : state_.last_input;
  state_.last_pointer_ts = steady_clock::now();
}

void UIHighlightManager::on_pointer_leave() {
  state_.pointer_hover.reset();
  state_.last_input = InputSource::Mouse;
}

void UIHighlightManager::on_pointer_press(const InteractiveId& id) {
  state_.pressed = id;
  state_.last_input = InputSource::Mouse;
}

void UIHighlightManager::on_pointer_release(const InteractiveId& id) {
  (void)id;
  state_.pressed.reset();
  state_.last_input = InputSource::Mouse;
}

void UIHighlightManager::on_keyboard_focus(std::optional<InteractiveId> id) {
  state_.keyboard_focus = id;
  state_.last_input = InputSource::Keyboard;
}

void UIHighlightManager::compute_highlighted() {
  // 1) pressed 优先
  if (state_.pressed) {
    state_.highlighted = state_.pressed;
    return;
  }

  // 2) 近期鼠标优先
  if (state_.pointer_hover) {
    const auto now = steady_clock::now();
    const auto ms_since_move = duration_cast<milliseconds>(now - state_.last_pointer_ts).count();
    if (ms_since_move <= strategy_.recent_mouse_ms) {
      state_.highlighted = state_.pointer_hover;
      return;
    }
  }

  // 3) 键盘焦点
  if (state_.keyboard_focus) {
    state_.highlighted = state_.keyboard_focus;
    return;
  }

  // 4) 无高亮
  state_.highlighted.reset();
}

bool UIHighlightManager::is_highlighted(const InteractiveId& id) const {
  return state_.highlighted && *state_.highlighted == id;
}

bool UIHighlightManager::is_focused(const InteractiveId& id) const {
  return state_.keyboard_focus && *state_.keyboard_focus == id;
}

bool UIHighlightManager::is_pressed(const InteractiveId& id) const {
  return state_.pressed && *state_.pressed == id;
}

} // namespace sunray_tui
