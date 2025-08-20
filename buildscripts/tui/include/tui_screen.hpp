#pragma once

#include <string>
#include <vector>

namespace sunray_tui {

// 屏幕元素类型
enum class ElementType {
  GROUP_HEADER,
  MODULE_ITEM,
  SEPARATOR,
  INFO_TEXT,
  BUILD_BUTTON,
  DIALOG_OK_BUTTON,
  UNKNOWN
};

// 屏幕元素信息
struct ElementInfo {
  ElementType type = ElementType::UNKNOWN;
  int render_item_index = -1; // -1 for special elements like buttons
  std::string identifier;
  bool is_clickable = false;
  bool is_hoverable = false;

  ElementInfo() = default;
  ElementInfo(ElementType t, int idx = -1, const std::string &id = "")
      : type(t), render_item_index(idx), identifier(id) {
    // 设置默认的交互属性
    is_clickable = (type == ElementType::GROUP_HEADER ||
                    type == ElementType::MODULE_ITEM ||
                    type == ElementType::BUILD_BUTTON ||
                    type == ElementType::DIALOG_OK_BUTTON);
    is_hoverable = is_clickable;
  }
};

// 统一的屏幕坐标映射器 - Linus风格：简单、可靠、统一
class ScreenCoordinateMapper {
private:
  struct ScreenElement {
    int screen_y;
    ElementInfo element_info;

    ScreenElement(int y, const ElementInfo &info)
        : screen_y(y), element_info(info) {}
  };

  std::vector<ScreenElement> coordinate_map;
  int base_offset = 0; // 基础偏移量，用于终端适配
  
  // 动态布局参数
  int left_column_width = 0;
  int right_column_start_x = 0;

public:
  // 重建坐标映射 - 每次渲染后调用
  void rebuild_mapping(const std::vector<struct RenderItem> &render_items,
                       int build_button_y = -1, bool dialog_shown = false,
                       int dialog_ok_button_y = -1);

  // 双栏版本：重建坐标映射 - 支持左栏和右栏独立项目列表
  void rebuild_dual_column_mapping(const std::vector<struct RenderItem> &left_items,
                                   const std::vector<struct RenderItem> &right_items,
                                   int left_content_start_y, int right_content_start_y,
                                   int left_column_width, int right_column_start_x,
                                   int build_button_y = -1, bool dialog_shown = false,
                                   int dialog_ok_button_y = -1, 
                                   int right_scroll_offset = 0, int right_visible_count = -1);

  // 获取指定屏幕坐标的元素信息
  ElementInfo get_element_at(int screen_y) const;

  // 获取指定屏幕坐标的元素信息 - 双栏版本，包含X坐标检测
  ElementInfo get_element_at(int screen_y, int screen_x) const;

  // 检查指定坐标是否可点击
  bool is_clickable_at(int screen_y) const;

  // 检查指定坐标是否可hover
  bool is_hoverable_at(int screen_y) const;

  // 查找最近的可交互元素 - 容错机制
  ElementInfo find_nearest_interactive(int screen_y,
                                       int max_distance = 2) const;

  // 设置基础偏移量 - 用于终端适配
  void set_base_offset(int offset) { base_offset = offset; }

  // 终端自适应检测和调整
  void auto_detect_terminal_offset();

  // 验证坐标映射的准确性
  bool validate_mapping() const;

  // 清空映射
  void clear() { coordinate_map.clear(); }
};

} // namespace sunray_tui