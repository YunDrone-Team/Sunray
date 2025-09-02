#pragma once
#include "tui_screen.hpp"
#include "tui_types.hpp"
#include "tui_interaction.hpp"
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "ftxui/dom/elements.hpp"  // for ftxui::Box

namespace sunray_tui {

/**
 * @brief UI核心状态管理器 - 双栏布局版本
 * 
 * 这是整个TUI系统的核心数据结构，负责管理双栏UI的所有状态信息。
 * 
 * 架构重构说明：
 * - 从单列表层次结构改为双栏并行结构
 * - 左栏：组选择器，显示所有可用的模块组
 * - 右栏：模块显示器，显示当前激活组的模块
 * - 移除了所有展开/折叠逻辑，简化交互模式
 * 
 * 设计理念：
 * - 关注点分离：组选择和模块选择是独立的功能
 * - 状态最小化：只保留真正必要的状态信息
 * - 交互简化：消除复杂的层次导航，采用直观的双栏选择
 * 
 * 双栏UI生命周期：
 * 1. 构造：创建空的UIState对象
 * 2. 初始化：加载配置，生成双栏渲染数据
 * 3. 运行：独立管理左右栏的选择和导航
 * 4. 销毁：自动清理所有资源
 */
struct UIState {
  // ==================== 数据层 ====================
  
  /** 
   * @brief 核心配置数据
   * 存储从YAML文件加载的模块和组的定义信息
   * 这是不可变的基础数据，作为整个UI系统的数据源
   */
  CoreData core_data;
  
  /** 
   * @brief 视图状态管理器 - 双栏版本
   * 管理激活组、模块选择状态和搜索状态
   * 移除了展开/折叠相关的复杂状态
   */
  ViewState view;
  
  /** 
   * @brief 交互管理器（智能指针）
   * 负责处理模块选择逻辑和冲突解决
   * 使用unique_ptr实现延迟初始化和内存安全管理
   */
  std::unique_ptr<InteractionManager> interaction_manager;
  
  // ==================== 双栏渲染层 ====================
  
  /** 
   * @brief 左栏渲染项目列表（组选择器）
   * 存储所有可用组的渲染项目，用户在此选择要查看的组
   * 每个项目代表一个模块组，显示组名和选择统计
   */
  std::vector<RenderItem> group_render_items;
  
  /** 
   * @brief 右栏渲染项目列表（模块显示器）
   * 存储当前激活组内所有模块的渲染项目
   * 根据激活组动态更新，显示模块选择状态和冲突状态
   */
  std::vector<RenderItem> module_render_items;
  
  /** 
   * @brief 屏幕坐标映射器
   * 将逻辑上的渲染项目映射到屏幕上的具体坐标
   * 支持双栏布局的鼠标交互和精确的点击检测
   */
  ScreenCoordinateMapper coordinate_mapper;
  
  // ==================== 双栏导航状态 ====================
  
  /** 
   * @brief 左栏（组选择器）当前选中索引
   * 指向group_render_items数组中当前高亮显示的组
   * 范围：0 到 group_render_items.size()-1
   */
  int group_selection_index = 0;
  
  /** 
   * @brief 右栏（模块显示器）当前选中索引
   * 指向module_render_items数组中当前高亮显示的模块
   * 范围：0 到 module_render_items.size()-1
   */
  int module_selection_index = 0;
  
  /** 
   * @brief 当前焦点所在栏位
   * true = 左栏（组选择器）有焦点
   * false = 右栏（模块显示器）有焦点
   * 影响键盘导航的响应行为
   */
  bool left_pane_focused = true;
  
  // ==================== 调试辅助状态 ====================
  
  /** 
   * @brief 调试窗口状态
   * 用于实时显示鼠标和键盘事件信息，帮助开发和调试UI交互
   */
  struct DebugInfo {
    // ==================== 鼠标状态数据 ====================
    /** @brief 当前鼠标X坐标 - 显示为 "Mouse: (0,67)" */
    int mouse_x = 0;
    /** @brief 当前鼠标Y坐标 - 显示为 "Mouse: (0,67)" */
    int mouse_y = 0;
    /** @brief 左键是否按下 - 显示为 "Buttons: L0 R1" 或 "Buttons: L0 R0" */
    bool left_button = false;
    /** @brief 右键是否按下 - 显示为 "Buttons: L0 R1" 或 "Buttons: L0 R0" */ 
    bool right_button = false;
    /** @brief 最近的鼠标滚轮方向 - 显示为 "Scroll: Up" 或 "Scroll: Down" 或 "Scroll: None" */
    std::string last_scroll = "None";
    
    // ==================== 键盘状态数据 ====================
    /** @brief 最后按下的按键 - 显示为 "Key: Other" 或 "Key: Tab" */
    std::string last_key = "None";
    
    // ==================== 元素检测数据 ====================
    /** @brief 当前鼠标悬停元素的类型ID - 显示为 "Element: Type=6 Index=-1" */
    int element_type = 0;
    /** @brief 当前鼠标悬停元素的索引 - 显示为 "Element: Type=6 Index=-1" */
    int element_index = -1;
    
    // ==================== 构建按钮坐标数据 ====================
    /** @brief 构建按钮X坐标 - 显示为 "Build: (44,61)" */
    int build_button_x = -1;
    /** @brief 构建按钮Y坐标 - 显示为 "Build: (44,61)" */
    int build_button_y = -1;
    /** @brief 构建按钮是否被鼠标悬停 - 显示为 "BuildHover: N" 或 "BuildHover: Y" */
    bool build_button_hovered = true;
    
    // ==================== 调试显示开关 ====================
    /** @brief 是否显示鼠标坐标信息 - 控制 "Mouse: (0,67)" 行 */
    bool show_mouse_coords = true;
    /** @brief 是否显示鼠标按键状态 - 控制 "Buttons: L0 R0" 行 */
      bool show_mouse_buttons = true;
    /** @brief 是否显示鼠标滚轮信息 - 控制 "Scroll: Up" 行 */
    bool show_mouse_scroll = false;
    /** @brief 是否显示键盘按键信息 - 控制 "Key: Other" 行 */
    bool show_keyboard = true;
    /** @brief 是否显示元素检测信息 - 控制 "Element: Type=6 Index=-1" 行 */
    bool show_element_info = false;
    /** @brief 是否显示构建按钮坐标 - 控制 "Build: (44,61)" 行 */
    bool show_build_coords = false;
    /** @brief 是否显示模块统计信息 - 控制 "Modules: 16 Groups: 6" 行 */
    bool show_module_stats = false;
    /** @brief 是否显示终端尺寸信息 - 控制 "Terminal: 89x73" 行 */
    bool show_terminal_size = false;
    /** @brief 是否显示构建按钮悬停状态 - 控制 "BuildHover: N" 行 */
    bool show_build_hover = false;
    
    /** @brief 检查是否有任何调试信息启用 */
    bool has_any_enabled() const {
      return show_mouse_coords || show_mouse_buttons || show_mouse_scroll || show_keyboard || 
             show_element_info || show_build_coords || show_module_stats || show_terminal_size || show_build_hover;
    }
  } debug_info;
  
  /** 
   * @brief 鼠标悬停的项目索引（左栏）
   * 左栏中鼠标悬停项目的索引，-1表示无悬停
   */
  int group_hover_index = -1;
  
  /** 
   * @brief 鼠标悬停的项目索引（右栏）
   * 右栏中鼠标悬停项目的索引，-1表示无悬停
   */
  int module_hover_index = -1;
  
  // ==================== 滚动状态 ====================
  
  /** 
   * @brief 右栏模块列表滚动偏移量
   * 用于当模块数量超过可显示区域时的滚动显示
   * 表示跳过的顶部条目数量
   */
  int module_scroll_offset = 0;
  
  /** 
   * @brief 右栏可显示的模块数量
   * 动态计算，基于终端高度减去固定UI元素的高度
   */
  int module_visible_count = 10;  // 默认值，运行时会重新计算
  
  // ==================== 窗口尺寸控制 ====================
  
  /** 
   * @brief 最小窗口宽度要求
   */
  static constexpr int MIN_TERMINAL_WIDTH = 65;
  
  /** 
   * @brief 最小窗口高度要求
   */
  static constexpr int MIN_TERMINAL_HEIGHT = 35;
  
  /** 
   * @brief 当前窗口是否满足最小尺寸要求
   */
  bool window_size_adequate = true;
  
  // ==================== UI区域高度常量 ====================
  
  /**
   * @brief 按键指南区域内容行数（不包括边框）
   */
  static constexpr int KEY_GUIDE_CONTENT_LINES = 3;
  
  /**
   * @brief 调试区域内容行数（不包括边框）
   */
  static constexpr int DEBUG_AREA_CONTENT_LINES = 3;
  
  /**
   * @brief 底部固定区域总高度
   * 按键指南(边框2+内容3) + 调试区域(边框2+内容3) + 构建按钮(1) = 11行
   */
  static constexpr int BOTTOM_FIXED_AREA_HEIGHT = 
      (2 + KEY_GUIDE_CONTENT_LINES) + (2 + DEBUG_AREA_CONTENT_LINES) + 1;
  
  // ==================== 动画状态 ====================
  
  /** 
   * @brief 上一次选中的组索引
   * 用于计算组选择变化的动画效果
   */
  int previous_group_selection = -1;
  
  /** 
   * @brief 上一次选中的模块索引
   * 用于计算模块选择变化的动画效果
   */
  int previous_module_selection = -1;
  
  /** 
   * @brief 动画进行中标志
   * 当任一栏的选择发生变化时设为true，动画完成后设为false
   */
  bool animation_in_progress = false;
  
  // ==================== 信息显示 ====================
  
  /** 
   * @brief 当前选中项目的描述文本
   * 根据当前焦点栏位显示对应项目的描述
   * 左栏焦点时显示组描述，右栏焦点时显示模块描述
   */
  std::string current_item_description;
  
  /** 
   * @brief 当前选中项目的详细信息
   * 包含依赖关系、冲突信息、路径等详细数据
   * 支持多行显示，用换行符分隔
   */
  std::string current_item_details;
  
  // ==================== 构建系统状态 ====================
  
  /** 
   * @brief 构建确认对话框显示状态
   * true时显示模态对话框，询问用户确认构建
   * false时隐藏对话框，显示正常的双栏界面
   */
  bool show_build_dialog = false;
  
  /** 
   * @brief 构建请求标志
   * 用户确认构建后设为true，触发TUI退出和CLI启动
   * 这是TUI向CLI移交控制权的关键标志
   */
  bool build_requested = false;
  
  /**
   * @brief 触发立即退出的回调函数
   * 由UIRenderer设置，用于在设置build_requested后立即触发事件循环检查
   * 解决FTXUI被动事件循环导致的退出延迟问题
   */
  std::function<void()> trigger_exit_callback;
  
  /** 
   * @brief 构建按钮焦点状态
   * 通过Tab键切换焦点时，构建按钮获得焦点时为true
   * 影响按钮的视觉显示（高亮、边框等）
   */
  bool build_button_focused = false;
  // 按键面板内的按钮焦点：0=开始构建，1=清除构建
  int button_focus_index = 0;
  
  /** 
   * @brief 构建按钮悬停状态
   * 鼠标在构建按钮上悬停时为true
   * 用于实现hover效果，改变按钮颜色
   */
  bool build_button_hovered = false;
  
  /** 
   * @brief 构建按钮在屏幕上的Y坐标
   * 用于鼠标点击检测和坐标映射
   * -1表示按钮不可见或坐标未计算
   */
  int build_button_screen_y = -1;

  /**
   * @brief 构建按钮的布局盒子（由FTXUI reflect捕获）
   * 用于提供更精确的X坐标范围（x_min..x_max）
   */
  ftxui::Box build_button_box;
  
  // ==================== 冲突提示系统 ====================
  
  /** 
   * @brief 冲突闪烁效果激活状态
   * 当用户尝试选择冲突模块时激活闪烁提示
   * 通过视觉效果提醒用户注意冲突
   */
  bool conflict_flash_active = false;
  
  /** 
   * @brief 冲突闪烁计数器
   * 记录当前闪烁的次数，用于控制闪烁频率
   * 与max_flash_count配合实现定时闪烁
   */
  int conflict_flash_count = 0;
  
  /** 
   * @brief 最大闪烁次数
   * 闪烁效果的总次数限制，避免无限闪烁
   * 达到此数值后自动停止闪烁
   */
  const int max_flash_count = 6;

  // ==================== 构建按钮警告系统 ====================
  
  /** 
   * @brief 构建按钮警告闪烁效果激活状态
   * 当用户点击构建按钮但没有选择任何模块时激活
   * 通过黄红交替闪烁提醒用户选择模块
   */
  bool build_warning_flash_active = false;
  
  /** 
   * @brief 构建按钮警告闪烁计数器
   * 记录当前闪烁的次数，用于控制闪烁频率和持续时间
   * 闪烁1秒后自动恢复正常状态
   */
  int build_warning_flash_count = 0;
  
  /** 
   * @brief 构建按钮警告闪烁最大次数
   * 1秒内的闪烁次数，约10次 (100ms间隔)
   */
  const int max_build_warning_flash_count = 10;

  // ==================== 构造与初始化 ====================
  
  /** 
   * @brief 默认构造函数
   * 初始化所有基本类型成员为默认值
   * 复杂对象（如智能指针）将在initialize()中初始化
   */
  UIState() = default;
  
  /**
   * @brief 初始化UIState对象
   * @param data 从配置文件加载的核心数据
   * 
   * 双栏初始化流程：
   * 1. 复制核心数据
   * 2. 创建交互管理器
   * 3. 构建索引和映射
   * 4. 生成左栏（组）渲染列表
   * 5. 激活默认组并生成右栏（模块）渲染列表
   * 6. 初始化焦点和选择状态
   */
  void initialize(const CoreData& data);
  
  /**
   * @brief 构建内部索引结构
   * 为快速查找和访问创建各种索引
   * 包括模块名到对象的映射、组名到对象的映射等
   */
  void build_indices();
  
  // ==================== 交互代理方法 ====================
  
  /**
   * @brief 切换模块选择状态（带冲突检测）
   * @param module_name 要切换的模块名称
   * 
   * 这是用户选择模块的主要接口：
   * 1. 如果模块未选中，尝试选中（检查冲突）
   * 2. 如果模块已选中，取消选中
   * 3. 自动处理依赖关系和冲突解决
   */
  void toggle_module_selection_with_conflicts(const std::string& module_name);
  
  /**
   * @brief 检查模块是否因冲突而被禁用
   * @param module_name 模块名称
   * @return true如果模块被禁用，false如果模块可选择
   * 
   * 禁用原因：
   * - 与已选模块存在冲突
   * - 依赖项未满足（如果实现了依赖检查）
   */
  bool is_module_disabled(const std::string& module_name) const;
  
  /**
   * @brief 检查模块是否已被选中
   * @param module_name 模块名称
   * @return true如果模块已选中，false否则
   */
  bool is_module_selected_new(const std::string& module_name) const;
  
  // ==================== 数据查找代理 ====================
  
  /**
   * @brief 根据名称查找模块
   * @param name 模块名称
   * @return 指向Module对象的指针，未找到时返回nullptr
   */
  const Module* find_module(const std::string& name) const;
  
  /**
   * @brief 根据名称查找模块组
   * @param name 组名称
   * @return 指向ModuleGroup对象的指针，未找到时返回nullptr
   */
  const ModuleGroup* find_group(const std::string& name) const;
  
  // ==================== 双栏渲染系统 ====================
  
  /**
   * @brief 更新左栏渲染项目列表（组选择器）
   * 
   * 生成所有可用组的渲染项目：
   * 1. 遍历所有模块组
   * 2. 计算每个组的选择统计（已选/总数）
   * 3. 标记当前激活的组
   * 4. 应用搜索过滤器（如果有）
   * 5. 生成格式化的显示文本
   * 
   * 调用时机：
   * - 系统初始化时
   * - 模块选择状态改变时
   * - 搜索条件变化时
   */
  void update_group_render_items();
  
  /**
   * @brief 更新右栏渲染项目列表（模块显示器）
   * 
   * 生成当前激活组内所有模块的渲染项目：
   * 1. 获取当前激活组的模块列表
   * 2. 为每个模块创建渲染项目
   * 3. 设置模块的选择状态和禁用状态
   * 4. 应用搜索过滤器（如果有）
   * 5. 添加状态信息和统计数据
   * 
   * 调用时机：
   * - 激活组变化时
   * - 模块选择状态改变时
   * - 冲突状态更新时
   */
  void update_module_render_items();
  
  /**
   * @brief 确保未分组模块的处理
   * 将不属于任何组的模块添加到特殊的"ungrouped"组中
   * 保证所有模块都能在界面中显示
   */
  void ensure_ungrouped_modules();
  
  // ==================== 双栏导航控制 ====================
  
  /**
   * @brief 在左栏向上移动选择
   * 将group_selection_index向前移动，支持循环
   */
  void move_group_selection_up();
  
  /**
   * @brief 在左栏向下移动选择
   * 将group_selection_index向后移动，支持循环
   */
  void move_group_selection_down();
  
  /**
   * @brief 在右栏向上移动选择
   * 将module_selection_index向前移动，支持循环
   */
  void move_module_selection_up();
  
  /**
   * @brief 在右栏向下移动选择
   * 将module_selection_index向后移动，支持循环
   */
  void move_module_selection_down();
  
  /**
   * @brief 获取当前选中的组渲染项目
   * @return 指向当前组RenderItem的指针，索引无效时返回nullptr
   */
  RenderItem* get_current_group_item();
  const RenderItem* get_current_group_item() const;
  
  /**
   * @brief 获取当前选中的模块渲染项目
   * @return 指向当前模块RenderItem的指针，索引无效时返回nullptr
   */
  RenderItem* get_current_module_item();
  const RenderItem* get_current_module_item() const;
  
  // ==================== 双栏事件处理 ====================
  
  /**
   * @brief 处理组激活事件
   * @return true如果处理了事件，false如果事件未处理
   * 
   * 当用户在左栏选择组时调用：
   * 1. 激活选中的组
   * 2. 更新右栏显示该组的模块
   * 3. 重置右栏选择索引
   * 4. 更新详细信息显示
   */
  bool handle_group_activation();
  
  /**
   * @brief 处理模块选择事件
   * @return true如果处理了事件，false如果事件未处理
   * 
   * 当用户在右栏选择模块时调用：
   * 1. 切换模块的选择状态
   * 2. 处理冲突检测
   * 3. 更新左栏的统计显示
   * 4. 更新详细信息显示
   */
  bool handle_module_selection();
  
  /**
   * @brief 处理栏位焦点切换
   * 在左栏和右栏之间切换焦点
   * 通常响应Tab键或左右方向键
   */
  void handle_pane_switch();
  
  /**
   * @brief 处理搜索过滤更新
   * @param filter 搜索过滤字符串
   * 
   * 更新搜索过滤器，重新生成双栏渲染列表
   */
  void handle_search_update(const std::string& filter);
  
  /**
   * @brief 更新当前项目的显示信息
   * 
   * 根据当前焦点栏位和选择项目更新详细信息：
   * - 左栏焦点：显示组的描述和模块统计
   * - 右栏焦点：显示模块的依赖、冲突、路径等信息
   */
  void update_current_item_info();
  
  // ==================== 焦点与按钮管理 ====================
  
  /**
   * @brief 处理Tab键焦点切换
   * 在左栏、右栏和构建按钮之间循环切换焦点
   */
  void handle_tab_focus();
  
  /**
   * @brief 处理Shift+Tab键反向焦点切换
   * 在构建按钮、右栏和左栏之间反向循环切换焦点
   */
  void handle_tab_focus_reverse();
  
  /**
   * @brief 处理构建按钮点击
   * 显示构建确认对话框，列出选中的模块
   */
  void handle_build_button();
  
  /**
   * @brief 关闭构建确认对话框
   * 隐藏模态对话框，返回正常的双栏界面
   */
  void close_build_dialog();
  
  /**
   * @brief 触发冲突提示闪烁效果
   * 当用户尝试选择冲突模块时调用
   */
  void trigger_conflict_flash();
  
  /**
   * @brief 更新冲突闪烁状态
   * 在渲染循环中调用，推进闪烁动画
   */
  void update_conflict_flash();

  /**
   * @brief 触发构建按钮警告闪烁效果
   * 当用户点击构建按钮但没有选择任何模块时调用
   */
  void trigger_build_warning_flash();
  
  /**
   * @brief 更新构建按钮警告闪烁状态
   * 在渲染循环中调用，推进闪烁动画
   */
  void update_build_warning_flash();
  
  // ==================== 滚动控制 ====================
  
  /**
   * @brief 计算右栏可显示的模块数量
   * 基于当前终端尺寸动态计算
   */
  void calculate_module_visible_count();
  
  /**
   * @brief 确保当前选择项可见（滚动到可视区域内）
   * 当键盘导航超出可视范围时自动调整滚动偏移
   */
  void ensure_module_selection_visible();
  
  /**
   * @brief 调整模块列表滚动位置
   * @param direction 滚动方向（正数向下滚动，负数向上滚动）
   */
  void scroll_module_list(int direction);
  
  // ==================== 窗口尺寸管理 ====================
  
  /**
   * @brief 检查窗口尺寸是否满足最小要求
   * @return true 如果窗口尺寸足够，false 如果需要调整
   */
  bool check_window_size();
  
  /**
   * @brief 获取当前终端尺寸
   * @return pair<width, height> 当前终端的宽度和高度
   */
  std::pair<int, int> get_terminal_size() const;
};

/**
 * @brief 配置数据简化适配器
 * 
 * 提供静态工厂方法，简化配置加载和UI状态创建过程
 * 隐藏底层配置系统的复杂性，为UI层提供清洁的接口
 */
struct ConfigDataSimplified {
  /**
   * @brief 从文件加载配置数据
   * @param file_path YAML配置文件的路径
   * @return 配置数据的智能指针，加载失败时返回nullptr
   */
  static std::unique_ptr<ConfigData> load_from_file(const std::string& file_path);
  
  /**
   * @brief 将配置数据转换为UI状态
   * @param config_data 已加载的配置数据
   * @return 完全初始化的双栏UIState对象
   */
  static UIState create_ui_state(const ConfigData& config_data);
};

/**
 * @brief 不区分大小写的字符串匹配函数
 * 
 * @param text 要搜索的文本
 * @param filter 搜索过滤器
 * @return true 如果text包含filter（忽略大小写），false 否则
 * 
 * 用于实现搜索功能，支持模糊匹配。
 */
bool matches_filter(const std::string &text, const std::string &filter);

} // namespace sunray_tui
