#include "tui_render.hpp"
#include "ftxui/screen/terminal.hpp"

using namespace ftxui;

namespace sunray_tui {

// ==================== 双栏坐标映射和调试 ====================

void UIRenderer::rebuild_dual_column_coordinate_mapping() {
  // 动态计算Y坐标偏移量 - 基于修复后的UI结构
  // 结构分析：title(1) + separator(1) + 双栏标题在列表内部(模块组+separator在border内)
  // 所以双栏内容开始位置：title(1) + separator(1) + border_top(1) + 模块组标题(1) + separator(1) = 5
  const int left_content_start_y = 5;   // 左栏内容开始位置（边框内的模块组标题下方）
  const int right_content_start_y = 5;  // 右栏内容开始位置（边框内的所有模块标题下方）

  // 动态计算X坐标边界 - 根据实际终端尺寸和布局比例
  auto [left_width, right_start] = calculate_dynamic_column_boundaries();

  // 使用动态参数重建双栏坐标映射
  state_.coordinate_mapper.rebuild_dual_column_mapping(
      state_.group_render_items,   // 左栏：组列表
      state_.module_render_items,  // 右栏：模块列表
      left_content_start_y,        // 左栏内容起始Y坐标
      right_content_start_y,       // 右栏内容起始Y坐标
      left_width,                  // 动态计算的左栏宽度边界
      right_start,                 // 动态计算的右栏起始X坐标
      state_.module_scroll_offset, // 右栏滚动偏移
      state_.module_visible_count  // 右栏可见数量
  );
}

std::pair<int, int> UIRenderer::calculate_dynamic_column_boundaries() {
  // 根据用户发现的精确公式实现
  // 左栏起始：x = 2（固定，由边框结构决定）
  // 右栏起始：x = int((W-1)/2)，其中W为终端宽度

  int terminal_width = 80; // 默认值

  // 获取实际终端宽度
  try {
    auto terminal_size = ftxui::Terminal::Size();
    terminal_width = terminal_size.dimx;
  } catch (...) {
    terminal_width = 90; // 使用测试环境的宽度作为fallback
  }

  // 应用用户发现的通用公式
  const int left_column_start_x = 2; // 固定起始位置
  const int right_column_start_x =
      (terminal_width - 1) / 2; // 整数除法自动向下取整

  // 左栏结束位置：右栏起始前的一个位置减去边框间隔
  const int left_column_end_x = right_column_start_x - 3; // 为边框预留空间

  // 最小宽度保护
  if (left_column_end_x <= left_column_start_x + 20 ||
      terminal_width - right_column_start_x < 25) {
    // 空间太小，使用保守值
    return {35, 40};
  }

  return {left_column_end_x, right_column_start_x};
}

} // namespace sunray_tui