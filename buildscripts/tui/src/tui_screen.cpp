#include "tui_screen.hpp"
#include "tui_types.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <unordered_set>


namespace sunray_tui {

void ScreenCoordinateMapper::rebuild_mapping(
    const std::vector<RenderItem> &render_items) {
  coordinate_map.clear();

  // 计算基础偏移量：border(1) + title(1) + separator(1) = 3
  const int content_start_y = 3 + base_offset;

  // 映射render_items到屏幕坐标
  for (size_t i = 0; i < render_items.size(); ++i) {
    const auto &item = render_items[i];
    int screen_y = content_start_y + static_cast<int>(i);

    ElementType element_type = ElementType::UNKNOWN;
    switch (item.type) {
    case RenderItem::GROUP_HEADER:
      element_type = ElementType::GROUP_HEADER;
      break;
    case RenderItem::MODULE_ITEM:
      element_type = ElementType::MODULE_ITEM;
      break;
    case RenderItem::SEPARATOR:
      element_type = ElementType::SEPARATOR;
      break;
    case RenderItem::INFO_TEXT:
      element_type = ElementType::INFO_TEXT;
      break;
    }

    ElementInfo element_info(element_type, static_cast<int>(i),
                             item.identifier);

    // 禁用的模块不可点击
    if (element_type == ElementType::MODULE_ITEM && item.is_disabled) {
      element_info.is_clickable = false;
      element_info.is_hoverable = false;
    }

    coordinate_map.emplace_back(screen_y, element_info);
  }

  // 底部按钮与对话框均由组件/上层管理，不再写入映射

  // 按screen_y排序，便于查找
  std::sort(coordinate_map.begin(), coordinate_map.end(),
            [](const ScreenElement &a, const ScreenElement &b) {
              return a.screen_y < b.screen_y;
            });
}

ElementInfo ScreenCoordinateMapper::get_element_at(int screen_y) const {
  // 精确匹配
  for (const auto &element : coordinate_map) {
    if (element.screen_y == screen_y) {
      return element.element_info;
    }
  }

  // 返回默认的未知元素
  return ElementInfo();
}

bool ScreenCoordinateMapper::is_clickable_at(int screen_y) const {
  ElementInfo info = get_element_at(screen_y);
  return info.is_clickable;
}

bool ScreenCoordinateMapper::is_hoverable_at(int screen_y) const {
  ElementInfo info = get_element_at(screen_y);
  return info.is_hoverable;
}

ElementInfo
ScreenCoordinateMapper::find_nearest_interactive(int screen_y,
                                                 int max_distance) const {
  // 首先尝试精确匹配
  ElementInfo exact = get_element_at(screen_y);
  if (exact.is_clickable || exact.is_hoverable) {
    return exact;
  }

  // 在指定范围内查找最近的可交互元素
  ElementInfo best_match;
  int best_distance = max_distance + 1;

  for (const auto &element : coordinate_map) {
    if (!element.element_info.is_clickable &&
        !element.element_info.is_hoverable) {
      continue;
    }

    int distance = std::abs(element.screen_y - screen_y);
    if (distance <= max_distance && distance < best_distance) {
      best_match = element.element_info;
      best_distance = distance;
    }
  }

  return best_match;
}

void ScreenCoordinateMapper::auto_detect_terminal_offset() {
  // 简单的终端检测：尝试不同的偏移量
  // 在实际使用中，这可以通过检测第一个有效点击来自动校正

  // 检测环境变量中的终端类型
  const char *term = std::getenv("TERM");
  const char *term_program = std::getenv("TERM_PROGRAM");

  if (term_program && std::string(term_program) == "Apple_Terminal") {
    // macOS Terminal可能需要不同的偏移
    base_offset = 0;
  } else if (term && std::string(term).find("xterm") != std::string::npos) {
    // xterm系列
    base_offset = 0;
  } else if (term && std::string(term).find("screen") != std::string::npos) {
    // screen/tmux
    base_offset = 0;
  } else {
    // 默认值
    base_offset = 0;
  }
}

bool ScreenCoordinateMapper::validate_mapping() const {
  if (coordinate_map.empty()) {
    return false;
  }

  // 检查坐标是否按顺序排列
  for (size_t i = 1; i < coordinate_map.size(); ++i) {
    if (coordinate_map[i].screen_y <= coordinate_map[i - 1].screen_y) {
      std::cout << "Warning: Coordinate mapping not properly sorted!"
                << std::endl;
      return false;
    }
  }

  // 检查是否有重复的坐标
  std::unordered_set<int> seen_coords;
  for (const auto &element : coordinate_map) {
    if (seen_coords.count(element.screen_y)) {
      std::cout << "Warning: Duplicate screen coordinates found: Y="
                << element.screen_y << std::endl;
      return false;
    }
    seen_coords.insert(element.screen_y);
  }

  return true;
}

void ScreenCoordinateMapper::rebuild_dual_column_mapping(
    const std::vector<RenderItem> &left_items,
    const std::vector<RenderItem> &right_items, int left_content_start_y,
    int right_content_start_y, int left_column_width, int right_column_start_x,
    int right_scroll_offset, int right_visible_count) {

  coordinate_map.clear();

  // 使用传入的动态参数，而不是硬编码偏移量

  // 映射左栏项目（组）- 类型为GROUP_HEADER
  for (size_t i = 0; i < left_items.size(); ++i) {
    const auto &item = left_items[i];
    int screen_y = left_content_start_y + static_cast<int>(i);

    ElementInfo element_info(ElementType::GROUP_HEADER, static_cast<int>(i),
                             item.identifier);

    // 禁用的组不可点击（虽然组一般不会被禁用）
    if (item.is_disabled) {
      element_info.is_clickable = false;
      element_info.is_hoverable = false;
    }

    coordinate_map.emplace_back(screen_y, element_info);
  }

  // 映射右栏项目（模块）- 考虑滚动偏移
  int visible_count = (right_visible_count > 0)
                          ? right_visible_count
                          : static_cast<int>(right_items.size());
  int end_index = std::min(static_cast<int>(right_items.size()),
                           right_scroll_offset + visible_count);

  for (int i = right_scroll_offset; i < end_index; ++i) {
    if (i >= 0 && i < static_cast<int>(right_items.size())) {
      const auto &item = right_items[i];
      // 屏幕Y坐标 = 起始位置 + 相对于可见区域的索引
      int screen_y = right_content_start_y + (i - right_scroll_offset);

      ElementInfo element_info(ElementType::MODULE_ITEM, i, item.identifier);

      // 禁用的模块不可点击
      if (item.is_disabled) {
        element_info.is_clickable = false;
        element_info.is_hoverable = false;
      }

      coordinate_map.emplace_back(screen_y, element_info);
    }
  }

  // 底部按钮与对话框由组件管理，不加入映射

  // 存储动态布局参数供get_element_at使用
  this->left_column_width = left_column_width;
  this->right_column_start_x = right_column_start_x;

  // 按screen_y排序，便于查找
  std::sort(coordinate_map.begin(), coordinate_map.end(),
            [](const ScreenElement &a, const ScreenElement &b) {
              return a.screen_y < b.screen_y;
            });
}

ElementInfo ScreenCoordinateMapper::get_element_at(int screen_y,
                                                   int screen_x) const {
  // 使用动态计算的布局参数，并确保考虑边框偏移
  const int left_column_max_x = left_column_width > 0 ? left_column_width : 32;
  const int right_column_min_x =
      right_column_start_x > 0 ? right_column_start_x : 40;

  // 根据实际测试结果推导的边框偏移计算公式
  // 测试数据：终端宽度90px，左栏从x=2开始，右栏从x=44开始
  // 边框结构：主窗口(1px) + 左栏边框(1px) = 左栏内容从x=2开始
  const int main_border_left = 1; // 主窗口左边框
  const int left_border_left = 1; // 左栏左边框
  const int effective_left_start = main_border_left + left_border_left; // = 2

  // 收集所有匹配Y坐标的元素
  std::vector<ElementInfo> matching_elements;
  for (const auto &element : coordinate_map) {
    if (element.screen_y == screen_y) {
      matching_elements.push_back(element.element_info);
    }
  }

  if (matching_elements.empty()) {
    return ElementInfo(); // 没有找到任何元素
  }

  // 根据X坐标和元素类型选择正确的元素
  for (const auto &element : matching_elements) {
    // 左栏区域：GROUP_HEADER类型，X坐标必须在有效范围内且不在边框区域
    if (element.type == ElementType::GROUP_HEADER) {
      // 确保点击位置在左栏的有效交互区域内
      if (screen_x >= effective_left_start && screen_x <= left_column_max_x) {
        return element;
      }
    }

    // 右栏区域：MODULE_ITEM类型，X坐标必须在有效范围内
    if (element.type == ElementType::MODULE_ITEM) {
      if (screen_x >= right_column_min_x) {
        return element;
      }
    }
  }

  // X坐标不匹配任何元素，返回未知元素
  return ElementInfo();
}

} // namespace sunray_tui
