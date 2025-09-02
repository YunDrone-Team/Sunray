#include "tui_render.hpp"
#include "ftxui/dom/elements.hpp"

using namespace ftxui;

namespace sunray_tui {

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

} // namespace sunray_tui