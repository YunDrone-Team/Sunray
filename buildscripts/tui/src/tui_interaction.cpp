#include "tui_interaction.hpp"

namespace sunray_tui {

InteractionManager::InteractionManager(const CoreData &core_data)
    : core_data_(core_data) {
  init_module_states();
}

void InteractionManager::init_module_states() {
  module_states_.clear();
  module_state_index_.clear();
  module_states_.reserve(core_data_.modules.size());
  for (size_t i = 0; i < core_data_.modules.size(); ++i) {
    module_states_.emplace_back(core_data_.modules[i].name);
    module_state_index_[core_data_.modules[i].name] = i;
  }
}

bool InteractionManager::is_module_selected(
    const std::string &module_name) const {
  auto it = module_state_index_.find(module_name);
  return (it != module_state_index_.end())
             ? module_states_[it->second].is_selected
             : false;
}

bool InteractionManager::is_module_disabled(
    const std::string &module_name) const {
  auto it = module_state_index_.find(module_name);
  return (it != module_state_index_.end())
             ? module_states_[it->second].is_disabled
             : false;
}

void InteractionManager::toggle_module_selection(
    const std::string &module_name) {
  auto it = module_state_index_.find(module_name);
  if (it != module_state_index_.end()) {
    ModuleState &state = module_states_[it->second];
    if (state.is_disabled && !state.is_selected) {
      trigger_conflict_indication();
      return;
    }
    state.is_selected = !state.is_selected;
    resolve_conflicts_realtime();
  }
}

bool InteractionManager::toggle_group_selection(const std::string &group_name) {
  const ModuleGroup *group = core_data_.find_group(group_name);
  if (!group || group_name == "ungrouped") {
    return false;
  }

  // 检查组内模块的当前选择状态
  bool all_selected = true, any_selected = false;
  for (const std::string &module_name : group->modules) {
    bool is_selected = is_module_selected(module_name);
    if (is_selected) {
      any_selected = true;
    } else {
      all_selected = false;
    }
  }

  // 决定操作：如果全部已选中则取消选择，否则全部选中
  bool target_state = !all_selected;

  // 批量切换组内所有模块
  for (const std::string &module_name : group->modules) {
    auto it = module_state_index_.find(module_name);
    if (it != module_state_index_.end()) {
      ModuleState &state = module_states_[it->second];
      if (target_state) {
        if (!state.is_disabled) {
          state.is_selected = true;
        }
      } else {
        state.is_selected = false;
      }
    }
  }
  resolve_conflicts_realtime();
  return true;
}

void InteractionManager::resolve_conflicts_realtime() {
  auto selected_modules = get_selected_module_set();
  for (auto &state : module_states_) {
    state.is_disabled = false;
    if (state.is_selected)
      continue;
    const Module *module = core_data_.find_module(state.name);
    if (module) {
      for (const std::string &conflict : module->conflicts_with) {
        if (selected_modules.count(conflict)) {
          state.is_disabled = true;
          break;
        }
      }
    }
  }
}

std::vector<std::string> InteractionManager::get_selected_modules() const {
  std::vector<std::string> selected;
  for (const auto &state : module_states_) {
    if (state.is_selected) {
      selected.push_back(state.name);
    }
  }
  return selected;
}

size_t InteractionManager::selected_count() const {
  size_t count = 0;
  for (const auto &state : module_states_) {
    if (state.is_selected) {
      count++;
    }
  }
  return count;
}

void InteractionManager::set_conflict_callback(std::function<void()> callback) {
  conflict_callback_ = std::move(callback);
}

void InteractionManager::trigger_conflict_indication() {
  if (conflict_callback_) {
    conflict_callback_();
  }
}

void InteractionManager::clear_all_selections() {
  for (auto &state : module_states_) {
    state.is_selected = false;
    state.is_disabled = false;
  }
}

std::unordered_set<std::string>
InteractionManager::get_selected_module_set() const {
  std::unordered_set<std::string> selected;
  for (const auto &state : module_states_) {
    if (state.is_selected) {
      selected.insert(state.name);
    }
  }
  return selected;
}

} // namespace sunray_tui