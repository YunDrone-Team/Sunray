#pragma once

#include <optional>
#include <chrono>
#include <map>
#include "ftxui/dom/elements.hpp" // for ftxui::Box

namespace sunray_tui {

// 交互对象类型
enum class InteractiveKind {
  GroupItem,
  ModuleItem,
  ButtonStart,
  ButtonClear,
  None,
};

// 交互对象ID
struct InteractiveId {
  InteractiveKind kind = InteractiveKind::None;
  int index = -1; // 针对组/模块

  static InteractiveId None() { return {InteractiveKind::None, -1}; }
  static InteractiveId Start() { return {InteractiveKind::ButtonStart, -1}; }
  static InteractiveId Clear() { return {InteractiveKind::ButtonClear, -1}; }
  static InteractiveId Group(int i) { return {InteractiveKind::GroupItem, i}; }
  static InteractiveId Module(int i) { return {InteractiveKind::ModuleItem, i}; }

  bool operator<(const InteractiveId& other) const {
    if (kind != other.kind) return static_cast<int>(kind) < static_cast<int>(other.kind);
    return index < other.index;
  }
  bool operator==(const InteractiveId& other) const {
    return kind == other.kind && index == other.index;
  }
};

enum class InputSource { Mouse, Keyboard, None };

struct HighlightState {
  std::optional<InteractiveId> pointer_hover;
  std::optional<InteractiveId> keyboard_focus;
  std::optional<InteractiveId> pressed;
  std::optional<InteractiveId> highlighted;
  InputSource last_input = InputSource::None;
  std::chrono::steady_clock::time_point last_pointer_ts{};
};

// 策略：仲裁唯一高亮
struct HighlightStrategy {
  int recent_mouse_ms = 600; // 鼠标优先时间窗口
};

// 统一高亮管理器（骨架）
class UIHighlightManager {
 public:
  void register_box(const InteractiveId& id, const ftxui::Box& box);
  void clear_boxes_of_kind(InteractiveKind kind);

  // 鼠标输入
  void set_pointer_hover(std::optional<InteractiveId> id);
  void on_pointer_leave();
  void on_pointer_press(const InteractiveId& id);
  void on_pointer_release(const InteractiveId& id);

  // 键盘输入
  void on_keyboard_focus(std::optional<InteractiveId> id);

  // 计算唯一高亮
  void compute_highlighted();

  // 查询接口
  bool is_highlighted(const InteractiveId& id) const;
  bool is_focused(const InteractiveId& id) const;
  bool is_pressed(const InteractiveId& id) const;

  // 便捷：直接获取当前 highlighted
  std::optional<InteractiveId> highlighted() const { return state_.highlighted; }

  // 策略配置
  void set_strategy(const HighlightStrategy& s) { strategy_ = s; }

 private:
  HighlightState state_{};
  HighlightStrategy strategy_{};
  std::map<InteractiveId, ftxui::Box> boxes_;
};

} // namespace sunray_tui

