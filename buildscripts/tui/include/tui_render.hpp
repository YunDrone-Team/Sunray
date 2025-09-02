#pragma once

#include "ftxui/component/component.hpp"
#include "ftxui/dom/elements.hpp"
#include "tui_core.hpp"
#include <functional>


namespace ftxui {
class ComponentBase;
using Component = std::shared_ptr<ComponentBase>;
struct Mouse;
struct Event;
} // namespace ftxui

namespace sunray_tui {

/**
 * @brief UI渲染器 - 双栏版本
 *
 * 负责：
 * - 双栏FTXUI界面渲染
 * - 双栏鼠标和键盘事件处理
 * - 视觉效果和动画
 * - 双栏组件布局管理
 */
/**
 * @brief UI 渲染器（双栏 + 组件化底部按钮）
 *
 * - 渲染双栏（组/模块）与底部区域（按键指南/调试）
 * - 底部“开始编译构建/清除构建”采用 FTXUI 组件（Button + Container）
 * - 鼠标与键盘事件：按钮组件优先消费，未消费则由双栏逻辑处理
 */
class UIRenderer {
public:
  explicit UIRenderer(UIState &state);

  /**
   * @brief 运行渲染器（带构建回调）
   * @param build_callback 构建按钮点击时的回调函数
   * @return 程序退出码
   */
  /**
   * @brief 运行渲染器（TUI→CLI 移交）
   * @param build_callback 触发“开始编译构建”后的回调（TUI退出后执行）
   * @return 退出码
   */
  int run_with_build_callback(std::function<void()> build_callback);

private:
  UIState &state_;
  std::function<void()> build_callback_;

  // ==================== 底部按钮（组件化） ====================
  /// 开始编译构建按钮组件
  ftxui::Component start_button_;
  /// 清除构建按钮组件（点击后绿色闪烁三次，仅视觉）
  ftxui::Component clear_button_;
  /// 水平容器，承载两个按钮
  ftxui::Component buttons_row_;
  /// 清除构建按钮闪烁计数（>0时闪烁），单位：帧；3次闪烁≈6帧
  int clear_button_flash_remaining_ = 0;

  // Hover 状态（使用 FTXUI Hoverable 装饰器获取）
  bool start_button_hovered_ = false;
  bool clear_button_hovered_ = false;

  // 清理过程的状态机
  enum class CleanState { Idle, Running, Success, Error };
  CleanState clear_state_ = CleanState::Idle;
  int clear_success_frames_remaining_ = 0; // 成功提示剩余帧数（约2秒）
  int clear_anim_tick_ = 0;                // 等待动画计数器

  // 清理失败详细输出与退出码（显示在按钮下方，自动换行）
  int clear_exit_code_ = -1;
  std::string clear_output_;
  /**
   * @brief 创建主UI组件
   * @return FTXUI组件
   */
  ftxui::Component create_component();

  // ==================== 双栏渲染辅助方法 ====================

  /**
   * @brief 渲染组项目（左栏）
   * @param item 组渲染项目
   * @param is_selected 是否被选中
   * @param is_focused 所在栏位是否有焦点
   * @param is_hovered 是否被鼠标悬停
   * @return FTXUI元素
   */
  ftxui::Element render_group_item(const RenderItem &item, bool is_selected,
                                   bool is_focused, bool is_hovered);

  /**
   * @brief 渲染模块项目（右栏）
   * @param item 模块渲染项目
   * @param is_selected 是否被选中
   * @param is_focused 所在栏位是否有焦点
   * @param is_hovered 是否被鼠标悬停
   * @return FTXUI元素
   */
  ftxui::Element render_module_item(const RenderItem &item, bool is_selected,
                                    bool is_focused, bool is_hovered);

  /**
   * @brief 渲染按键指南区域
   * @return FTXUI元素
   */
  ftxui::Element render_key_guide();

  /**
   * @brief 渲染底部调试窗口
   * @return FTXUI元素
   */
  ftxui::Element render_debug_window();

  // 旧的文本构建按钮渲染已由组件化按钮取代

  // ==================== 双栏事件处理 ====================

  /**
   * @brief 处理双栏键盘事件
   * @param event 键盘事件
   * @return 是否处理了事件
   */
  bool handle_dual_column_keyboard_event(const ftxui::Event &event);

  /**
   * @brief 处理双栏鼠标事件
   * @param mouse 鼠标事件信息
   * @return 是否处理了事件
   */
  bool handle_dual_column_mouse_event(const ftxui::Mouse &mouse);

  /**
   * @brief 处理双栏鼠标移动
   * @param mouse 鼠标事件信息
   * @return 是否处理了事件
   */
  bool handle_dual_column_mouse_move(const ftxui::Mouse &mouse);

  /**
   * @brief 处理双栏鼠标点击
   * @param mouse 鼠标事件信息
   * @return 是否处理了事件
   */
  bool handle_dual_column_mouse_click(const ftxui::Mouse &mouse);

  /**
   * @brief 处理鼠标滚轮事件
   * @param mouse 鼠标事件信息
   * @return 是否处理了事件
   */
  bool handle_mouse_wheel(const ftxui::Mouse &mouse);

  /**
   * @brief 计算调试窗口的实际内容行数
   * @return 调试窗口内容行数（不包括边框）
   */
  int calculate_debug_content_lines() const;

  /**
   * @brief 计算按键指南的实际内容行数
   * @return 按键指南内容行数（不包括边框）
   */
  int calculate_key_guide_content_lines() const;

  // ==================== 双栏坐标映射和调试 ====================

  /**
   * @brief 重建双栏坐标映射（不含底部按钮）
   */
  void rebuild_dual_column_coordinate_mapping();

  /**
   * @brief 动态计算双栏边界坐标
   * @return pair<左栏最大X, 右栏起始X>
   */
  std::pair<int, int> calculate_dynamic_column_boundaries();

  /**
   * @brief hover时更新details区域信息
   * 根据当前hover的组或模块实时更新底部信息显示
   */
  void update_details_on_hover();

  // 触发“清除构建”按钮对应的动作（与鼠标点击一致）
  void trigger_clear_build_clean();

  // ==================== 键盘导航辅助方法 ====================

  /**
   * @brief 向上移动组hover位置
   */
  void move_group_hover_up();

  /**
   * @brief 向下移动组hover位置
   */
  void move_group_hover_down();

  /**
   * @brief 向上移动模块hover位置
   */
  void move_module_hover_up();

  /**
   * @brief 向下移动模块hover位置
   */
  void move_module_hover_down();

  /**
   * @brief 同步hover状态到当前活动栏位
   */
  void sync_hover_to_active_pane();
};

} // namespace sunray_tui
