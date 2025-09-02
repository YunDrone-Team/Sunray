#include "tui_render.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/screen/terminal.hpp"

using namespace ftxui;

namespace sunray_tui {

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

// ==================== 按键指南渲染 ====================

ftxui::Element UIRenderer::render_key_guide() {
  // 按键提示 - 3列布局（每列2行）
  return hbox({
    // 第一列 - 导航
    vbox({text("↑↓←→") | color(Color::Cyan),
          text("Tab") | color(Color::Cyan)}) | flex,
    text("  ") | color(Color::Default),
    vbox({text("导航") | color(Color::GrayLight),
          text("焦点") | color(Color::GrayLight)}) | flex,
    text("   ") | color(Color::Default),
    
    // 第二列 - 操作
    vbox({text("Enter") | color(Color::Cyan),
          text("Space/C") | color(Color::Cyan)}) | flex,
    text("  ") | color(Color::Default),
    vbox({text("选择") | color(Color::GrayLight),
          text("批量/清空") | color(Color::GrayLight)}) | flex,
    text("   ") | color(Color::Default),
    
    // 第三列 - 退出和其他
    vbox({text("q/Esc") | color(Color::Cyan),
          text("鼠标") | color(Color::Cyan)}) | flex,
    text("  ") | color(Color::Default),
    vbox({text("退出") | color(Color::GrayLight),
          text("交互") | color(Color::GrayLight)}) | flex
  }) | border | bgcolor(Color::RGB(30, 30, 30));
}

} // namespace sunray_tui