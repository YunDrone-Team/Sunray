#pragma once

#include <string>
#include <vector>

namespace sunray_tui {

/// 屏幕元素类型
enum class ElementType {
  GROUP_HEADER,
  MODULE_ITEM,
  SEPARATOR,
  INFO_TEXT,
  UNKNOWN
};

/// 屏幕元素信息
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
                    type == ElementType::MODULE_ITEM);
    is_hoverable = is_clickable;
  }
};

/**
 * @brief 屏幕坐标映射器（双栏）
 *
 * - 将渲染项映射到屏幕坐标，用于鼠标命中检测
 * - 仅管理组/模块两类交互元素；底部按钮改由组件管理
 */
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

  // 构建按钮的X命中范围（由FTXUI reflect提供）。-1表示未知，退化为整行命中。
  int build_button_x_min = -1;
  int build_button_x_max = -1;

public:
  /// 重建单列坐标映射
  void rebuild_mapping(const std::vector<struct RenderItem> &render_items);

  /// 重建双栏坐标映射
  void rebuild_dual_column_mapping(const std::vector<struct RenderItem> &left_items,
                                   const std::vector<struct RenderItem> &right_items,
                                   int left_content_start_y, int right_content_start_y,
                                   int left_column_width, int right_column_start_x,
                                   int right_scroll_offset = 0, int right_visible_count = -1);

  /// 获取指定Y的元素信息（单列）
  ElementInfo get_element_at(int screen_y) const;

  /// 获取指定(X,Y)的元素信息（双栏）
  ElementInfo get_element_at(int screen_y, int screen_x) const;

  /// 指定Y是否可点击
  bool is_clickable_at(int screen_y) const;

  /// 指定Y是否可hover
  bool is_hoverable_at(int screen_y) const;

  /// 在范围内查找最近的可交互元素
  ElementInfo find_nearest_interactive(int screen_y,
                                       int max_distance = 2) const;

  /// 设置基础偏移量（终端适配）
  void set_base_offset(int offset) { base_offset = offset; }

  /// 终端自适应检测和调整
  void auto_detect_terminal_offset();

  /// 验证映射是否有序且不重复
  bool validate_mapping() const;

  /// 清空映射
  void clear() { coordinate_map.clear(); }

  // 不再需要设置构建按钮的X范围（由组件管理）
};

} // namespace sunray_tui
