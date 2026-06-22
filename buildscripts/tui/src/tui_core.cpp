/**
 * @file tui_core.cpp
 * @brief TUI双栏架构核心状态管理器的实现
 *
 * 本文件实现了UIState的双栏布局核心功能，包括：
 * - 双栏状态初始化和配置管理
 * - 独立的组选择和模块显示逻辑
 * - 双栏导航和事件处理
 * - 交互状态同步和冲突处理
 * - 双栏渲染数据生成
 *
 * 架构重构说明：
 * - 从单列表层次结构改为左右栏并行结构
 * - 左栏：组选择器，显示所有可用的模块组
 * - 右栏：模块显示器，显示当前激活组的模块
 * - 移除了所有展开/折叠逻辑，简化交互模式
 * - 实现完全独立的双栏导航系统
 *
 * @author Sunray TUI Team
 * @date 2024
 */

#include "tui_core.hpp"
#include "ftxui/screen/terminal.hpp"
#include <algorithm>
#include <cctype>
#include <map>
#include <set>


namespace sunray_tui {

// ==================== UIState双栏核心实现 ====================

/**
 * @brief 初始化UIState对象 - 双栏版本
 *
 * 这是UIState的双栏初始化函数，负责设置整个双栏TUI系统的初始状态。
 *
 * @param data 从YAML配置文件加载的核心数据
 *
 * 双栏初始化流程：
 * 1. 复制核心配置数据到本地存储
 * 2. 创建交互管理器实例，负责冲突检测和模块选择逻辑
 * 3. 设置冲突回调，当检测到冲突时触发视觉提示
 * 4. 初始化屏幕坐标映射器，用于鼠标交互
 * 5. 构建内部索引结构
 * 6. 激活第一个组作为默认激活组
 * 7. 生成双栏初始渲染列表
 * 8. 更新当前选中项的详细信息显示
 */
void UIState::initialize(const CoreData &data) {
  // 1. 复制核心数据 - 这是整个系统的数据源
  core_data = data;

  // 2. 创建交互管理器 - 负责复杂的选择和冲突逻辑
  interaction_manager = std::make_unique<InteractionManager>(core_data);

  // 3. 设置冲突检测回调 - 当用户尝试选择冲突模块时触发闪烁提示
  interaction_manager->set_conflict_callback([this]() {
    trigger_conflict_flash(); // Lambda捕获this指针，调用成员函数
  });

  // 4. 初始化坐标映射器 - 自动检测终端类型和坐标偏移
  coordinate_mapper.auto_detect_terminal_offset();

  // 5. 构建索引结构
  build_indices();

  // 6. 用户体验优化：激活第一个组作为默认激活组
  if (!core_data.groups.empty()) {
    view.set_active_group(core_data.groups[0].name);
  }

  // 7. 生成双栏初始UI渲染列表
  update_group_render_items();
  update_module_render_items();

  // 8. 初始化滚动状态
  calculate_module_visible_count();
  ensure_module_selection_visible();

  // 9. 初始化信息显示区域
  update_current_item_info();
}

/**
 * @brief 构建内部索引结构
 *
 * 为提高查找性能，构建各种快速索引：
 * - 模块名到Module对象的映射
 * - 组名到ModuleGroup对象的映射
 * - 依赖关系图
 */
void UIState::build_indices() {
  // 让核心数据构建自己的索引
  core_data.build_indices();

  // 如果交互管理器存在，初始化模块状态
  if (interaction_manager) {
    interaction_manager->init_module_states();
  }

  // 重新检测终端坐标偏移（可能因窗口大小变化）
  coordinate_mapper.auto_detect_terminal_offset();
}

// ==================== 交互代理方法实现 ====================

/**
 * @brief 切换模块选择状态（带冲突检测）
 *
 * 这是用户选择模块的主要入口点。它整合了新的冲突检测系统
 * 和传统的视图状态管理，确保两套系统保持同步。
 *
 * @param module_name 要切换选择状态的模块名称
 *
 * 工作流程：
 * 1. 委托给交互管理器处理实际的选择逻辑和冲突检测
 * 2. 同步更新视图状态，保持向后兼容性
 * 3. 触发UI更新（通过调用方负责）
 */
void UIState::toggle_module_selection_with_conflicts(
    const std::string &module_name) {
  if (interaction_manager) {
    interaction_manager->toggle_module_selection(module_name);
    view.selected_modules.clear();
    for (const auto &selected_module :
         interaction_manager->get_selected_modules()) {
      view.selected_modules.insert(selected_module);
    }
  }
}

/**
 * @brief 检查模块是否因冲突被禁用
 *
 * @param module_name 要检查的模块名称
 * @return true 如果模块被禁用（不可选择），false 如果模块可选择
 */
bool UIState::is_module_disabled(const std::string &module_name) const {
  return interaction_manager
             ? interaction_manager->is_module_disabled(module_name)
             : false;
}

/**
 * @brief 检查模块是否已被选中
 *
 * @param module_name 要检查的模块名称
 * @return true 如果模块已选中，false 否则
 */
bool UIState::is_module_selected_new(const std::string &module_name) const {
  return interaction_manager
             ? interaction_manager->is_module_selected(module_name)
             : false;
}

// ==================== 数据查找代理实现 ====================

/**
 * @brief 查找指定名称的模块
 *
 * @param name 模块名称
 * @return 指向Module对象的指针，未找到返回nullptr
 */
const Module *UIState::find_module(const std::string &name) const {
  return core_data.find_module(name);
}

/**
 * @brief 查找指定名称的模块组
 *
 * @param name 组名称
 * @return 指向ModuleGroup对象的指针，未找到返回nullptr
 */
const ModuleGroup *UIState::find_group(const std::string &name) const {
  return core_data.find_group(name);
}

// ==================== 双栏渲染系统实现 ====================

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
void UIState::update_group_render_items() {
  // 0. 确保所有模块都有归属组
  ensure_ungrouped_modules();

  // 1. 清空现有左栏渲染列表
  group_render_items.clear();

  // 2. 计算格式化参数 - 为了对齐显示，需要知道最大的数字宽度
  size_t max_group_size = 0;
  for (const auto &group : core_data.groups) {
    if (group.modules.size() > max_group_size) {
      max_group_size = group.modules.size();
    }
  }
  int max_digits = std::to_string(max_group_size).length();

  std::map<std::string, std::vector<const ModuleGroup *>> groups_by_label;
  for (const auto &group : core_data.groups) {
    // 3.1 搜索过滤逻辑 - 检查组名是否匹配搜索条件
    if (!view.search_filter.empty()) {
      if (!matches_filter(group.name, view.search_filter) &&
          !matches_filter(group.description, view.search_filter) &&
          !matches_filter(group.label, view.search_filter)) {
        continue; // 不匹配搜索条件，跳过
      }
    }
    groups_by_label[group.label.empty() ? "Other" : group.label].push_back(&group);
  }

  const std::vector<std::string> label_order = {"Product", "Simulation", "Test"};
  std::unordered_set<std::string> emitted_labels;
  auto emit_label_group = [&](const std::string &label) {
    auto it = groups_by_label.find(label);
    if (it == groups_by_label.end() || it->second.empty()) {
      return;
    }
    emitted_labels.insert(label);

    const auto &groups = it->second;
    size_t selected_groups = 0;
    for (const ModuleGroup *group : groups) {
      if (view.is_group_selected(group->name)) {
        selected_groups++;
      }
    }

    RenderItem label_item = RenderItem::label_header(
        label, "[" + label + "]",
        "(" + std::to_string(selected_groups) + "/" +
            std::to_string(groups.size()) + ")");
    label_item.has_selected_items =
        (selected_groups > 0);
    group_render_items.emplace_back(label_item);

    for (const ModuleGroup *group_ptr : groups) {
    const ModuleGroup &group = *group_ptr;
    // 3.2 计算组内选择统计
    size_t selected_in_group = 0;
    for (const auto &module_name : group.modules) {
      if (view.selected_modules.count(module_name)) {
        selected_in_group++;
      }
    }
    size_t total_in_group = group.modules.size();

    // 3.3 检查是否为当前激活组
    bool is_active = view.is_group_active(group.name);

    // 3.4 格式化计数器显示，确保数字对齐 - 例如: "(  2/ 10)"
    char counter_buffer[32];
    snprintf(counter_buffer, sizeof(counter_buffer), "(%*zu/%*zu)", max_digits,
             selected_in_group, max_digits, total_in_group);

    // 3.5 格式化组名显示
    std::string group_name_part = group.name;
    std::string counter_part = std::string(counter_buffer);

    // 3.6 创建组渲染项目
    RenderItem group_item =
        RenderItem::group_header(group.name, group_name_part, counter_part);
    // 黄色圆点只表示用户显式选中的组，模块覆盖全选仅通过计数体现。
    group_item.has_selected_items = view.is_group_selected(group.name);
    group_render_items.emplace_back(group_item);
    }
  };

  for (const auto &label : label_order) {
    emit_label_group(label);
  }
  for (const auto &[label, groups] : groups_by_label) {
    if (!emitted_labels.count(label)) {
      emit_label_group(label);
    }
  }

  // 4. 确保组选择索引有效
  if (group_selection_index >= static_cast<int>(group_render_items.size())) {
    group_selection_index =
        std::max(0, static_cast<int>(group_render_items.size()) - 1);
  }
  if (group_selection_index < 0 && !group_render_items.empty()) {
    group_selection_index = 0;
  }
  if (!group_render_items.empty() &&
      group_render_items[group_selection_index].type != RenderItem::GROUP_HEADER) {
    group_selection_index =
        find_next_selectable_group_index(group_selection_index, 1);
  }
}

bool UIState::is_selectable_group_index(int index) const {
  return index >= 0 && index < static_cast<int>(group_render_items.size()) &&
         group_render_items[index].type == RenderItem::GROUP_HEADER &&
         group_render_items[index].is_selectable &&
         !group_render_items[index].is_disabled;
}

int UIState::find_next_selectable_group_index(int start_index,
                                              int direction) const {
  if (group_render_items.empty()) {
    return -1;
  }

  const int size = static_cast<int>(group_render_items.size());
  const int step = direction >= 0 ? 1 : -1;
  int index = start_index;

  for (int checked = 0; checked < size; ++checked) {
    if (index < 0) {
      index = size - 1;
    } else if (index >= size) {
      index = 0;
    }
    if (is_selectable_group_index(index)) {
      return index;
    }
    index += step;
  }

  return -1;
}

/**
 * @brief 更新右栏渲染项目列表（所有模块显示）
 *
 * 生成所有模块的渲染项目：
 * 1. 遍历所有模块
 * 2. 为每个模块创建渲染项目
 * 3. 设置模块的选择状态和禁用状态
 * 4. 根据激活组高亮显示对应模块
 * 5. 应用搜索过滤器（如果有）
 *
 * 调用时机：
 * - 系统初始化时
 * - 模块选择状态改变时
 * - 激活组变化时（用于高亮）
 * - 冲突状态更新时
 */
void UIState::update_module_render_items() {
  // 1. 清空现有右栏渲染列表
  module_render_items.clear();

  // 2. 按 label_order 分组；未写入 label_order 的 label 统一归入 other。
  std::set<std::string> allowed_labels(core_data.label_order.begin(),
                                       core_data.label_order.end());
  std::vector<const Module *> sorted_modules;
  sorted_modules.reserve(core_data.modules.size());
  for (const auto &module : core_data.modules) {
    sorted_modules.push_back(&module);
  }

  auto normalized_text = [](const std::string &text) {
    std::string normalized = text;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return normalized;
  };

  std::stable_sort(
      sorted_modules.begin(), sorted_modules.end(),
      [&](const Module *lhs, const Module *rhs) {
        std::string lhs_label =
            allowed_labels.count(lhs->label) ? lhs->label : "other";
        std::string rhs_label =
            allowed_labels.count(rhs->label) ? rhs->label : "other";
        if (normalized_text(lhs_label) != normalized_text(rhs_label)) {
          return normalized_text(lhs_label) < normalized_text(rhs_label);
        }
        return normalized_text(lhs->name) < normalized_text(rhs->name);
      });

  std::map<std::string, std::vector<const Module *>> modules_by_label;
  for (const Module *module_ptr : sorted_modules) {
    const Module &module = *module_ptr;
    if (!view.search_filter.empty()) {
      if (!matches_filter(module.name, view.search_filter) &&
          !matches_filter(module.description, view.search_filter) &&
          !matches_filter(module.label, view.search_filter)) {
        continue;
      }
    }
    const std::string label =
        allowed_labels.count(module.label) ? module.label : "other";
    modules_by_label[label].push_back(module_ptr);
  }

  // 3. 生成 label 标题 + 模块项。标题行不可选择，仅用于分段和统计。
  std::vector<std::string> render_label_order = core_data.label_order;
  if (modules_by_label.count("other") && !allowed_labels.count("other")) {
    render_label_order.emplace_back("other");
  }

  for (const auto &label : render_label_order) {
    auto modules_it = modules_by_label.find(label);
    if (modules_it == modules_by_label.end() || modules_it->second.empty()) {
      continue;
    }
    const auto &modules = modules_it->second;
    size_t selected_in_label = 0;
    for (const Module *module_ptr : modules) {
      if (view.selected_modules.count(module_ptr->name)) {
        selected_in_label++;
      }
    }

    const std::string counter =
        "(" + std::to_string(selected_in_label) + "/" +
        std::to_string(modules.size()) + ")";
    RenderItem label_item =
        RenderItem::label_header(label, "[" + label + "]", counter);
    label_item.has_selected_items =
        (selected_in_label == modules.size() && !modules.empty());
    module_render_items.emplace_back(label_item);

    for (const Module *module_ptr : modules) {
      const Module &module = *module_ptr;
      bool is_selected = is_module_selected_new(module.name);
      bool is_disabled = is_module_disabled(module.name);

      RenderItem module_item =
          RenderItem::module_item(module.name, module.name, is_selected, 1);
      module_item.is_disabled = is_disabled;
    module_render_items.emplace_back(module_item);
    }
  }

  // 4. 确保模块选择索引落在可选择模块行上，避免停在 label 标题。
  if (module_selection_index >= static_cast<int>(module_render_items.size())) {
    module_selection_index =
        std::max(0, static_cast<int>(module_render_items.size()) - 1);
  }
  if (module_selection_index < 0 && !module_render_items.empty()) {
    module_selection_index = 0;
  }
  if (!module_render_items.empty() &&
      !is_selectable_module_index(module_selection_index)) {
    module_selection_index = find_next_selectable_module_index(
        module_selection_index, 1);
  }
}

bool UIState::is_selectable_module_index(int index) const {
  return index >= 0 && index < static_cast<int>(module_render_items.size()) &&
         module_render_items[index].type == RenderItem::MODULE_ITEM &&
         module_render_items[index].is_selectable &&
         !module_render_items[index].is_disabled;
}

int UIState::find_next_selectable_module_index(int start_index,
                                               int direction) const {
  if (module_render_items.empty()) {
    return -1;
  }

  const int size = static_cast<int>(module_render_items.size());
  const int step = direction >= 0 ? 1 : -1;
  int index = start_index;

  for (int checked = 0; checked < size; ++checked) {
    if (index < 0) {
      index = size - 1;
    } else if (index >= size) {
      index = 0;
    }
    if (is_selectable_module_index(index)) {
      return index;
    }
    index += step;
  }

  return -1;
}

/**
 * @brief 确保未分组模块得到正确处理
 *
 * 遍历所有模块，找出那些不属于任何定义组的模块，
 * 将它们归入特殊的"ungrouped"虚拟组中。
 *
 * 这确保了所有模块都能在UI中显示，避免配置遗漏导致的模块不可见问题。
 */
void UIState::ensure_ungrouped_modules() {
  // 1. 收集所有已被组包含的模块名称
  std::unordered_set<std::string> grouped_modules;
  for (const auto &group : core_data.groups) {
    for (const auto &module_name : group.modules) {
      grouped_modules.insert(module_name);
    }
  }

  // 2. 找出未归类的模块
  std::vector<std::string> ungrouped_modules;
  for (const auto &module : core_data.modules) {
    if (grouped_modules.find(module.name) == grouped_modules.end()) {
      ungrouped_modules.push_back(module.name);
    }
  }

  // 3. 如果有未归类模块，创建或更新ungrouped虚拟组
  if (!ungrouped_modules.empty()) {
    // 检查ungrouped组是否已存在
    bool ungrouped_exists = false;
    for (auto &group : core_data.groups) {
      if (group.name == "ungrouped") {
        // 更新现有ungrouped组的内容
        group.modules = ungrouped_modules;
        group.description = "未归类模块";
        ungrouped_exists = true;
        break;
      }
    }

    // 如果ungrouped组不存在，创建它
    if (!ungrouped_exists) {
      ModuleGroup ungrouped_group;
      ungrouped_group.name = "ungrouped";
      ungrouped_group.description = "未归类模块";
      ungrouped_group.modules = ungrouped_modules;
      core_data.groups.push_back(ungrouped_group);

      // 重建索引以包含新组
      build_indices();
    }
  }
}

// ==================== 双栏导航控制实现 ====================

/**
 * @brief 在左栏向上移动选择
 * 将group_selection_index向前移动，支持循环
 */
void UIState::move_group_selection_up() {
  if (group_render_items.empty()) {
    return;
  }

  group_selection_index =
      find_next_selectable_group_index(group_selection_index - 1, -1);

  // 更新详细信息显示
  update_current_item_info();
  ensure_group_selection_visible();
}

/**
 * @brief 在左栏向下移动选择
 * 将group_selection_index向后移动，支持循环
 */
void UIState::move_group_selection_down() {
  if (group_render_items.empty()) {
    return;
  }

  group_selection_index =
      find_next_selectable_group_index(group_selection_index + 1, 1);

  // 更新详细信息显示
  update_current_item_info();
  ensure_group_selection_visible();
}

/**
 * @brief 在右栏向上移动选择
 * 将module_selection_index向前移动，支持循环
 */
void UIState::move_module_selection_up() {
  if (module_render_items.empty()) {
    return;
  }

  module_selection_index =
      find_next_selectable_module_index(module_selection_index - 1, -1);

  // 更新详细信息显示
  update_current_item_info();

  // 确保选择项在滚动视图中可见
  ensure_module_selection_visible();
}

/**
 * @brief 在右栏向下移动选择
 * 将module_selection_index向后移动，支持循环
 */
void UIState::move_module_selection_down() {
  if (module_render_items.empty()) {
    return;
  }

  module_selection_index =
      find_next_selectable_module_index(module_selection_index + 1, 1);

  // 更新详细信息显示
  update_current_item_info();

  // 确保选择项在滚动视图中可见
  ensure_module_selection_visible();
}

/**
 * @brief 获取当前选中的组渲染项目（可修改版本）
 * @return 指向当前组RenderItem的指针，索引无效时返回nullptr
 */
RenderItem *UIState::get_current_group_item() {
  if (group_selection_index >= 0 &&
      group_selection_index < static_cast<int>(group_render_items.size())) {
    return &group_render_items[group_selection_index];
  }
  return nullptr;
}

/**
 * @brief 获取当前选中的组渲染项目（只读版本）
 * @return 指向当前组RenderItem的常量指针，索引无效时返回nullptr
 */
const RenderItem *UIState::get_current_group_item() const {
  if (group_selection_index >= 0 &&
      group_selection_index < static_cast<int>(group_render_items.size())) {
    return &group_render_items[group_selection_index];
  }
  return nullptr;
}

/**
 * @brief 获取当前选中的模块渲染项目（可修改版本）
 * @return 指向当前模块RenderItem的指针，索引无效时返回nullptr
 */
RenderItem *UIState::get_current_module_item() {
  if (module_selection_index >= 0 &&
      module_selection_index < static_cast<int>(module_render_items.size())) {
    return &module_render_items[module_selection_index];
  }
  return nullptr;
}

/**
 * @brief 获取当前选中的模块渲染项目（只读版本）
 * @return 指向当前模块RenderItem的常量指针，索引无效时返回nullptr
 */
const RenderItem *UIState::get_current_module_item() const {
  if (module_selection_index >= 0 &&
      module_selection_index < static_cast<int>(module_render_items.size())) {
    return &module_render_items[module_selection_index];
  }
  return nullptr;
}

// ==================== 双栏事件处理实现 ====================

/**
 * @brief 处理组选择事件（批量toggle）
 * @return true如果处理了事件，false如果事件未处理
 *
 * 当用户在左栏选择组时调用：
 * 1. 批量切换组内所有模块的选择状态
 * 2. 更新模块列表显示
 * 3. 更新左栏统计显示
 * 4. 更新详细信息显示
 */
bool UIState::handle_group_activation() {
  RenderItem *group_item = get_current_group_item();
  if (!group_item || group_item->type != RenderItem::GROUP_HEADER ||
      group_item->identifier.empty()) {
    return false;
  }

  const std::string group_name = group_item->identifier;
  const ModuleGroup *group = find_group(group_name);
  if (!group || group_name == "ungrouped") {
    return false;
  }

  // 显式 group 选择独立于模块覆盖关系：只有再次点击同一个已选 group 才取消。
  if (view.is_group_selected(group_name)) {
    view.selected_groups.erase(group_name);
  } else {
    view.selected_groups.insert(group_name);
  }
  view.set_active_group(group_name);

  if (interaction_manager) {
    interaction_manager->clear_all_selections();
    view.selected_modules.clear();

    for (const auto &selected_group_name : view.selected_groups) {
      const ModuleGroup *selected_group = find_group(selected_group_name);
      if (!selected_group) {
        continue;
      }
      for (const auto &module_name : selected_group->modules) {
        if (!interaction_manager->is_module_selected(module_name)) {
          interaction_manager->toggle_module_selection(module_name);
        }
        if (interaction_manager->is_module_selected(module_name)) {
          view.selected_modules.insert(module_name);
        }
      }
    }

    view.selected_modules.clear();
    for (const auto &module_name : interaction_manager->get_selected_modules()) {
      view.selected_modules.insert(module_name);
    }
  }

  update_group_render_items();
  update_module_render_items();
  update_current_item_info();

  return true;
}

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
bool UIState::handle_module_selection() {
  RenderItem *module_item = get_current_module_item();
  if (!module_item || module_item->type != RenderItem::MODULE_ITEM ||
      module_item->identifier.empty() || module_item->is_disabled) {
    return false;
  }

  // 切换模块的选择状态（带冲突检测）
  toggle_module_selection_with_conflicts(module_item->identifier);
  // 手动调整单个模块后，group 显式选择状态不再可靠，清空以避免误导。
  view.selected_groups.clear();

  // 检查是否需要更新激活组状态
  // 如果当前激活组内没有任何模块被选中，清除激活组状态
  if (!view.active_group.empty()) {
    const ModuleGroup *active_group = find_group(view.active_group);
    if (active_group) {
      bool any_selected = false;
      for (const auto &module_name : active_group->modules) {
        if (view.selected_modules.count(module_name)) {
          any_selected = true;
          break;
        }
      }

      if (!any_selected) {
        // 激活组内没有任何模块被选中，清除激活组状态
        view.active_group.clear();
      }
    }
  }

  // 更新右栏显示模块的新状态
  update_module_render_items();

  // 更新左栏的统计显示
  update_group_render_items();

  // 更新详细信息显示
  update_current_item_info();

  return true;
}

/**
 * @brief 处理栏位焦点切换
 * 在左栏和右栏之间切换焦点
 * 通常响应Tab键或左右方向键
 */
void UIState::handle_pane_switch() {
  left_pane_focused = !left_pane_focused;

  // 更新详细信息显示
  update_current_item_info();
}

/**
 * @brief 处理搜索过滤更新
 * @param filter 搜索过滤字符串
 *
 * 更新搜索过滤器，重新生成双栏渲染列表
 */
void UIState::handle_search_update(const std::string &filter) {
  view.search_filter = filter;

  // 重新生成双栏渲染列表
  update_group_render_items();
  update_module_render_items();

  // 重置选择索引
  group_selection_index = 0;
  module_selection_index = 0;

  // 更新详细信息显示
  update_current_item_info();
}

/**
 * @brief 更新当前项目的显示信息
 *
 * 根据当前焦点栏位和选择项目更新详细信息：
 * - 左栏焦点：显示组的描述和模块统计
 * - 右栏焦点：显示模块的依赖、冲突、路径等信息
 */
void UIState::update_current_item_info() {
  // 清空现有信息
  current_item_description.clear();
  current_item_details.clear();

  if (left_pane_focused) {
    // 显示当前选中组的信息
    const RenderItem *group_item = get_current_group_item();
    if (group_item) {
      if (group_item->type == RenderItem::LABEL_HEADER) {
        current_item_description = "模块组标签: " + group_item->identifier;
        current_item_details =
            "包含 " + group_item->counter_text + " 个预设组\n"
            "操作: 标签行仅用于分类显示\n"
            "路径: groups.label";
        return;
      }
      const ModuleGroup *group = find_group(group_item->identifier);
      if (group) {
        current_item_description =
            group->description.empty() ? "NULL" : group->description;
        current_item_details =
            "包含 " + std::to_string(group->modules.size()) + " 个模块";
      }
    }
  } else {
    // 显示当前选中模块的信息
    const RenderItem *module_item = get_current_module_item();
    if (module_item) {
      if (module_item->type == RenderItem::LABEL_HEADER) {
        size_t selected_in_label = 0;
        size_t total_in_label = 0;
        for (const auto &module : core_data.modules) {
          const std::string label = module.label.empty() ? "other" : module.label;
          if (label == module_item->identifier) {
            total_in_label++;
            if (view.selected_modules.count(module.name)) {
              selected_in_label++;
            }
          }
        }
        current_item_description = "模块标签: " + module_item->identifier;
        current_item_details =
            "包含 " + std::to_string(total_in_label) + " 个模块\n" +
            "已勾选: " + std::to_string(selected_in_label) + "\n" +
            "路径: 按 source_path 所在大目录分类";
        return;
      }

      const Module *module = find_module(module_item->identifier);
      if (module) {
        // 设置模块描述
        current_item_description =
            module->description.empty() ? "NULL" : module->description;

        // 生成详细信息 - 始终显示所有字段，即使为空
        std::string details;

        // 依赖信息
        details += "依赖: ";
        if (module->dependencies.empty()) {
          details += "NULL";
        } else {
          for (size_t i = 0; i < module->dependencies.size(); ++i) {
            if (i > 0)
              details += ", ";
            details += module->dependencies[i];
          }
        }
        details += "\n";

        // 冲突信息
        details += "冲突: ";
        if (module->conflicts_with.empty()) {
          details += "NULL";
        } else {
          for (size_t i = 0; i < module->conflicts_with.size(); ++i) {
            if (i > 0)
              details += ", ";
            details += module->conflicts_with[i];
          }
        }
        details += "\n";

        // 源码路径信息
        details += "路径: ";
        details += module->source_path.empty() ? "NULL" : module->source_path;

        current_item_details = details;
      }
    }
  }
}

// ==================== 焦点和按钮管理实现 ====================

/**
 * @brief 处理Tab键焦点切换
 * 在左栏、右栏和构建按钮之间循环切换焦点
 */
void UIState::handle_tab_focus() {
  if (left_pane_focused) {
    // 从左栏切换到右栏
    left_pane_focused = false;
    build_button_focused = false;
  } else if (!build_button_focused) {
    // 从右栏切换到构建按钮
    build_button_focused = true;
  } else {
    // 从构建按钮切换回左栏
    left_pane_focused = true;
    build_button_focused = false;
  }

  // 更新详细信息显示
  update_current_item_info();
}

void UIState::handle_tab_focus_reverse() {
  if (left_pane_focused) {
    // 从左栏反向切换到构建按钮
    left_pane_focused = false;
    build_button_focused = true;
  } else if (build_button_focused) {
    // 从构建按钮反向切换到右栏
    left_pane_focused = false;
    build_button_focused = false;
  } else {
    // 从右栏反向切换到左栏
    left_pane_focused = true;
    build_button_focused = false;
  }

  // 更新详细信息显示
  update_current_item_info();
}

/**
 * @brief 处理构建按钮点击
 * 检查是否有选择模块：
 * - 没有模块：触发警告闪烁效果
 * - 有模块：直接开始构建
 */
void UIState::handle_build_button() {
  // 检查是否有模块被选中
  if (view.selected_modules.empty()) {
    // 没有选择任何模块：触发警告闪烁
    trigger_build_warning_flash();
  } else {
    // 有模块被选中：直接设置构建请求标志并退出TUI
    build_requested = true;
    if (trigger_exit_callback) {
      trigger_exit_callback(); // 立即触发事件循环检查并退出
    }
  }
}

/**
 * @brief 关闭构建确认对话框
 * 隐藏模态对话框，返回正常的双栏界面
 */
void UIState::close_build_dialog() { show_build_dialog = false; }

/**
 * @brief 触发冲突提示闪烁效果
 * 当用户尝试选择冲突模块时调用
 */
void UIState::trigger_conflict_flash() {
  conflict_flash_active = true;
  conflict_flash_count = 0; // 重置闪烁计数器
}

/**
 * @brief 更新冲突闪烁状态
 * 在渲染循环中调用，推进闪烁动画
 */
void UIState::update_conflict_flash() {
  if (conflict_flash_active) {
    conflict_flash_count++;

    // 达到最大闪烁次数后停止
    if (conflict_flash_count >= max_flash_count) {
      conflict_flash_active = false;
      conflict_flash_count = 0;
    }
  }
}

/**
 * @brief 触发构建按钮警告闪烁效果
 * 当用户点击构建按钮但没有选择任何模块时调用
 */
void UIState::trigger_build_warning_flash() {
  build_warning_flash_active = true;
  build_warning_flash_count = 0; // 重置闪烁计数器
}

/**
 * @brief 更新构建按钮警告闪烁状态
 * 在渲染循环中调用，推进闪烁动画
 */
void UIState::update_build_warning_flash() {
  if (build_warning_flash_active) {
    build_warning_flash_count++;

    // 达到最大闪烁次数后停止（1秒）
    if (build_warning_flash_count >= max_build_warning_flash_count) {
      build_warning_flash_active = false;
      build_warning_flash_count = 0;
    }
  }
}

// ==================== ConfigDataSimplified工厂方法实现 ====================

/**
 * @brief 从文件加载配置数据的静态工厂方法
 *
 * @param file_path YAML配置文件路径
 * @return 配置数据智能指针，加载失败返回nullptr
 */
std::unique_ptr<ConfigData>
ConfigDataSimplified::load_from_file(const std::string &file_path) {
  return ConfigData::load_from_file(file_path);
}

/**
 * @brief 将配置数据转换为完全初始化的UIState对象
 *
 * @param config_data 已加载的配置数据
 * @return 完全初始化的UIState对象
 *
 * 这是整个TUI系统的启动入口点，负责创建和初始化所有必要组件。
 */
UIState ConfigDataSimplified::create_ui_state(const ConfigData &config_data) {
  UIState state;

  // 复制配置数据到状态对象
  state.core_data.modules = config_data.modules;
  state.core_data.groups = config_data.groups;
  state.core_data.label_order = config_data.label_order;

  // 执行完整初始化流程
  state.initialize(state.core_data);

  return state;
}

// ==================== 工具函数实现 ====================

/**
 * @brief 不区分大小写的字符串匹配函数
 *
 * @param text 要搜索的文本
 * @param filter 搜索过滤器
 * @return true 如果text包含filter（忽略大小写），false 否则
 *
 * 用于实现搜索功能，支持模糊匹配。
 */
bool matches_filter(const std::string &text, const std::string &filter) {
  if (filter.empty())
    return true; // 空过滤器匹配所有内容

  // 转换为小写进行比较
  std::string lower_text = text;
  std::string lower_filter = filter;

  std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(),
                 ::tolower);
  std::transform(lower_filter.begin(), lower_filter.end(), lower_filter.begin(),
                 ::tolower);

  // 子字符串匹配
  return lower_text.find(lower_filter) != std::string::npos;
}

// ==================== 滚动控制实现 ====================

/**
 * @brief 计算右栏可显示的模块数量
 * 基于当前终端尺寸和固定UI元素高度动态计算
 */
void UIState::calculate_module_visible_count() {
  auto [width, height] = get_terminal_size();
  (void)width;

  // 与主渲染器保持一致：标题/分隔符/详情区/按钮区/单行快捷键栏。
  const int fixed_ui_height = 1 + 1 + 1 + 1 + 3 + 1 + 1 + 1 + 1;
  const int available_height_for_columns = std::max(8, height - fixed_ui_height);

  // 双栏内部结构：栏标题(1) + 分隔符(1) + 内容区域 + 边框(2)
  const int column_header_height = 2;
  const int column_border_height = 2;
  const int available_content_height = available_height_for_columns -
                                       column_header_height -
                                       column_border_height;

  // 保守计算：至少显示3行，但不超过实际模块数量
  module_visible_count =
      std::max(3, std::min(available_content_height,
                           static_cast<int>(module_render_items.size())));

  // 如果计算结果太小，至少保证能显示几个条目
  if (module_visible_count < 3) {
    module_visible_count = 3;
  }

  group_visible_count =
      std::max(3, std::min(available_content_height,
                           static_cast<int>(group_render_items.size())));

  if (group_visible_count < 3) {
    group_visible_count = 3;
  }
}

/**
 * @brief 确保当前组选项可见（滚动到可视区域内）
 */
void UIState::ensure_group_selection_visible() {
  if (group_render_items.empty()) {
    group_scroll_offset = 0;
    return;
  }

  if (!is_selectable_group_index(group_selection_index)) {
    group_selection_index =
        find_next_selectable_group_index(group_selection_index, 1);
  }

  if (group_selection_index < 0) {
    group_scroll_offset = 0;
    return;
  }

  if (group_selection_index < group_scroll_offset) {
    group_scroll_offset = group_selection_index;
  }

  if (group_selection_index >= group_scroll_offset + group_visible_count) {
    group_scroll_offset = group_selection_index - group_visible_count + 1;
  }

  const int max_offset = std::max(
      0, static_cast<int>(group_render_items.size()) - group_visible_count);
  group_scroll_offset =
      std::max(0, std::min(group_scroll_offset, max_offset));
}

/**
 * @brief 确保当前选择项可见（滚动到可视区域内）
 * 自动调整滚动偏移，保持当前选择项始终在可视范围内
 */
void UIState::ensure_module_selection_visible() {
  if (module_render_items.empty()) {
    module_scroll_offset = 0;
    return;
  }

  // 如果选择项在滚动偏移之上，向上滚动
  if (!is_selectable_module_index(module_selection_index)) {
    module_selection_index =
        find_next_selectable_module_index(module_selection_index, 1);
  }

  if (module_selection_index < 0) {
    module_scroll_offset = 0;
    return;
  }

  // 如果选择项在滚动偏移之上，向上滚动
  if (module_selection_index < module_scroll_offset) {
    module_scroll_offset = module_selection_index;
  }

  // 如果选择项在可视范围之下，向下滚动
  if (module_selection_index >= module_scroll_offset + module_visible_count) {
    module_scroll_offset = module_selection_index - module_visible_count + 1;
  }

  // 确保滚动偏移在有效范围内
  const int max_offset = std::max(
      0, static_cast<int>(module_render_items.size()) - module_visible_count);
  module_scroll_offset =
      std::max(0, std::min(module_scroll_offset, max_offset));
}

/**
 * @brief 调整模块列表滚动位置
 * 支持手动滚动控制（如鼠标滚轮事件）
 */
void UIState::scroll_module_list(int direction) {
  if (module_render_items.empty()) {
    return;
  }

  const int old_offset = module_scroll_offset;
  const int max_offset = std::max(
      0, static_cast<int>(module_render_items.size()) - module_visible_count);

  module_scroll_offset += direction;
  module_scroll_offset =
      std::max(0, std::min(module_scroll_offset, max_offset));

  // 如果滚动位置发生变化，触发重新渲染
  if (module_scroll_offset != old_offset) {
    // 触发动画请求（如果需要的话）
    animation_in_progress = true;
  }
}

/**
 * @brief 调整模块组列表滚动位置
 * 支持手动滚动控制（如鼠标滚轮事件）
 */
void UIState::scroll_group_list(int direction) {
  if (group_render_items.empty()) {
    return;
  }

  const int old_offset = group_scroll_offset;
  const int max_offset = std::max(
      0, static_cast<int>(group_render_items.size()) - group_visible_count);

  group_scroll_offset += direction;
  group_scroll_offset =
      std::max(0, std::min(group_scroll_offset, max_offset));

  if (group_scroll_offset != old_offset) {
    animation_in_progress = true;
  }
}

// ==================== 窗口尺寸管理实现 ====================

/**
 * @brief 检查窗口尺寸是否满足最小要求
 * 实时检查终端尺寸，决定是否显示正常UI或尺寸警告
 */
bool UIState::check_window_size() {
  auto [width, height] = get_terminal_size();

  window_size_adequate =
      (width >= MIN_TERMINAL_WIDTH && height >= MIN_TERMINAL_HEIGHT);
  return window_size_adequate;
}

/**
 * @brief 获取当前终端尺寸
 * 安全地获取终端尺寸，提供默认值防止异常
 */
std::pair<int, int> UIState::get_terminal_size() const {
  try {
    auto terminal_size = ftxui::Terminal::Size();
    return {terminal_size.dimx, terminal_size.dimy};
  } catch (...) {
    // 获取失败时返回默认尺寸
    return {80, 24};
  }
}

} // namespace sunray_tui
