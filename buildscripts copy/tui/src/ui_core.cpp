#include "ui_core.hpp"
#include <algorithm>
#include <cctype>

namespace sunray_tui {

// 搜索匹配函数 - 简单的不区分大小写匹配
bool matches_filter(const std::string &text, const std::string &filter) {
  if (filter.empty())
    return true;

  std::string lower_text = text;
  std::string lower_filter = filter;

  std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(),
                 ::tolower);
  std::transform(lower_filter.begin(), lower_filter.end(), lower_filter.begin(),
                 ::tolower);

  return lower_text.find(lower_filter) != std::string::npos;
}

void UIState::ensure_ungrouped_modules() {
  // 收集所有已被group包含的模块
  std::unordered_set<std::string> grouped_modules;
  for (const auto& group : groups) {
    for (const auto& module_name : group.modules) {
      grouped_modules.insert(module_name);
    }
  }
  
  // 找出未归类的模块
  std::vector<std::string> ungrouped_modules;
  for (const auto& module : modules) {
    if (grouped_modules.find(module.name) == grouped_modules.end()) {
      ungrouped_modules.push_back(module.name);
    }
  }
  
  // 如果有未归类模块，创建或更新ungrouped虚拟组
  if (!ungrouped_modules.empty()) {
    // 检查是否已经存在ungrouped组
    bool ungrouped_exists = false;
    for (auto& group : groups) {
      if (group.name == "ungrouped") {
        // 更新现有的ungrouped组
        group.modules = ungrouped_modules;
        group.description = "未归类模块 (无法全选)";
        ungrouped_exists = true;
        break;
      }
    }
    
    // 如果不存在，创建新的ungrouped组
    if (!ungrouped_exists) {
      ModuleGroup ungrouped_group;
      ungrouped_group.name = "ungrouped";
      ungrouped_group.description = "未归类模块 (无法全选)";
      ungrouped_group.modules = ungrouped_modules;
      groups.push_back(ungrouped_group);
      
      // 重建索引以包含新组
      build_indices();
    }
  }
}

void UIState::update_render_items() {
  // 确保未归类模块被包含
  ensure_ungrouped_modules();
  
  render_items.clear();

  // 首先计算所有组的最大模块数量，用于格式化对齐
  size_t max_group_size = 0;
  for (const auto &group : groups) {
    if (group.modules.size() > max_group_size) {
      max_group_size = group.modules.size();
    }
  }
  // 计算最大数字的宽度
  int max_digits = std::to_string(max_group_size).length();

  // 显示组视图：可折叠的组
  for (const auto &group : groups) {
    // 检查组或其模块是否匹配搜索
    bool group_matches = matches_filter(group.name, view.search_filter) ||
                        matches_filter(group.description, view.search_filter);
    bool has_matching_modules = false;

    if (!group_matches) {
      for (const auto &module_name : group.modules) {
        const Module *module = find_module(module_name);
        if (module &&
            matches_filter(module->display_name(), view.search_filter)) {
          has_matching_modules = true;
          break;
        }
      }
    }

    if (!group_matches && !has_matching_modules) {
      continue;
    }

    // 添加组标题，包含选中计数
    bool is_expanded = view.is_group_expanded(group.name);
    size_t selected_in_group = view.selected_count_in_group(group.name, groups);
    size_t total_in_group = group.modules.size();

    // 格式化计数器，保证斜线对齐
    char counter_buffer[32];
    snprintf(counter_buffer, sizeof(counter_buffer), "(%*zu/%*zu)", max_digits,
             selected_in_group, max_digits, total_in_group);

    // 格式化组名显示，分离组名和计数器
    std::string group_name_part;
    if (group.name == "ungrouped") {
      group_name_part = (is_expanded ? "▼ " : "▶ ") + group.name;
    } else {
      group_name_part = (is_expanded ? "▼ " : "▶ ") + group.name;
    }
    std::string counter_part = std::string(counter_buffer);

    // 创建RenderItem，并设置选中状态标志
    RenderItem group_item =
        RenderItem::group_header(group.name, group_name_part, counter_part, is_expanded);
    group_item.has_selected_items = (selected_in_group > 0); // 添加选中状态标志
    render_items.emplace_back(group_item);

    // 如果展开，添加组内模块
    if (is_expanded) {
      for (const auto &module_name : group.modules) {
        const Module *module = find_module(module_name);
        if (module && (group_matches || 
                       matches_filter(module->name, view.search_filter) ||
                       matches_filter(module->description, view.search_filter))) {
          // 使用新的状态系统
          bool is_selected = is_module_selected_new(module_name);
          bool is_disabled = is_module_disabled(module_name);
          
          // 创建带禁用状态的模块项
          RenderItem module_item = RenderItem::module_item(
              module_name, module->name, is_selected, 1);
          module_item.is_disabled = is_disabled;  // 添加禁用状态标记
          render_items.emplace_back(module_item);
        }
      }
    }
  }

  // 添加状态信息
  if (!render_items.empty()) {
    render_items.emplace_back(RenderItem::separator());
  }

  render_items.emplace_back(RenderItem::info_text(
      "已选择: " + std::to_string(view.selected_count()) + " 个模块"));

  if (!view.search_filter.empty()) {
    render_items.emplace_back(
        RenderItem::info_text("搜索: " + view.search_filter));
  }

  // 确保选择索引有效
  if (current_selection >= static_cast<int>(render_items.size())) {
    current_selection = static_cast<int>(render_items.size()) - 1;
  }
  if (current_selection < 0) {
    current_selection = 0;
  }

  // 更新当前选中项信息
  update_current_item_info();
}

bool UIState::handle_expand_toggle() {
  RenderItem *item = get_current_item();
  if (!item)
    return false;

  if (item->is_collapsible) {
    // 处理组折叠/展开
    view.toggle_group_expansion(item->identifier);
    update_render_items();
    return true;
  }

  return false;
}

bool UIState::handle_selection() {
  RenderItem *item = get_current_item();
  if (!item)
    return false;

  if (item->type == RenderItem::GROUP_HEADER) {
    // 特殊处理：ungrouped组无法全选
    if (item->identifier == "ungrouped") {
      // 对于ungrouped组，只展开/折叠，不进行全选操作
      view.toggle_group_expansion(item->identifier);
      update_render_items();
      return true;
    }
    
    // 选择整个组：选择该组下的所有模块
    const ModuleGroup *group = find_group(item->identifier);
    if (group) {
      bool all_selected = true;
      // 检查是否所有模块都已选中
      for (const auto &module_name : group->modules) {
        if (!view.is_module_selected(module_name)) {
          all_selected = false;
          break;
        }
      }

      if (all_selected) {
        // 如果全部已选中，则取消选择所有模块
        for (const auto &module_name : group->modules) {
          // 同时更新新旧状态系统
          view.selected_modules.erase(module_name);
          
          // 更新module_states
          auto it = module_state_index.find(module_name);
          if (it != module_state_index.end()) {
            module_states[it->second].is_selected = false;
          }
        }
      } else {
        // 否则选择所有模块
        for (const auto &module_name : group->modules) {
          // 同时更新新旧状态系统
          view.selected_modules.insert(module_name);
          
          // 更新module_states
          auto it = module_state_index.find(module_name);
          if (it != module_state_index.end()) {
            module_states[it->second].is_selected = true;
          }
        }
        // 选择组时自动展开
        view.expanded_groups.insert(item->identifier);
      }
      
      // 重新计算冲突状态
      resolve_conflicts_realtime();
    }
    update_render_items();
    return true;
  }

  if (item->is_selectable) {
    // 使用新的交互式冲突解决系统
    toggle_module_selection_with_conflicts(item->identifier);
    update_render_items();
    return true;
  }

  return false;
}

void UIState::handle_search_update(const std::string &filter) {
  view.search_filter = filter;
  update_render_items();
  current_selection = 0; // 重置选择到顶部
}

void UIState::update_current_item_info() {
  current_item_description.clear();
  current_item_details.clear();

  const RenderItem *item = get_current_item();
  if (!item)
    return;

  if (item->type == RenderItem::MODULE_ITEM) {
    const Module *module = find_module(item->identifier);
    if (module) {
      // 设置描述
      current_item_description =
          module->description.empty() ? "NULL" : module->description;

      // 添加详细信息，总是显示所有字段
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

      // 路径信息
      details += "路径: ";
      details += module->source_path.empty() ? "NULL" : module->source_path;

      current_item_details = details;
    }
  } else if (item->type == RenderItem::GROUP_HEADER) {
    const ModuleGroup *group = find_group(item->identifier);
    if (group) {
      current_item_description =
          group->description.empty() ? "NULL" : group->description;
      current_item_details =
          "包含 " + std::to_string(group->modules.size()) + " 个模块";
    }
  }
}

// 焦点和按钮管理实现
void UIState::handle_tab_focus() {
  build_button_focused = !build_button_focused;
}

void UIState::handle_build_button() {
  show_build_dialog = true;
}

void UIState::close_build_dialog() {
  show_build_dialog = false;
}

// 冲突闪烁相关方法
void UIState::trigger_conflict_flash() {
  conflict_flash_active = true;
  conflict_flash_count = 0;
}

void UIState::update_conflict_flash() {
  if (conflict_flash_active) {
    conflict_flash_count++;
    if (conflict_flash_count >= max_flash_count) {
      conflict_flash_active = false;
      conflict_flash_count = 0;
    }
  }
}

} // namespace sunray_tui