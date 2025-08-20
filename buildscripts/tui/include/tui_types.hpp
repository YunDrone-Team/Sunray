#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sunray_tui {

struct ModuleGroup;

struct Module {
  std::string name, description, source_path, build_path;
  std::vector<std::string> dependencies, conflicts_with;
  std::string display_name() const { return name + " - " + description; }
  bool has_conflicts() const { return !conflicts_with.empty(); }
};

struct ModuleState {
  std::string name;
  bool is_selected = false, is_disabled = false;
  ModuleState() = default;
  explicit ModuleState(const std::string &n) : name(n) {}
};

struct ModuleGroup {
  std::string name, description;
  std::vector<std::string> modules;
  std::string display_name() const {
    return name + " (" + std::to_string(modules.size()) + " modules) - " + description;
  }
};

struct ViewState {
  std::string active_group, search_filter;
  std::unordered_set<std::string> selected_modules;
  int scroll_offset = 0;

  bool is_module_selected(const std::string &name) const { return selected_modules.count(name) > 0; }
  bool is_group_active(const std::string &name) const { return active_group == name; }
  void set_active_group(const std::string &name) { active_group = name; }
  void toggle_module_selection(const std::string &name) {
    if (is_module_selected(name)) selected_modules.erase(name);
    else selected_modules.insert(name);
  }
  void clear_selection() { selected_modules.clear(); }
  size_t selected_count() const { return selected_modules.size(); }
  size_t selected_count_in_group(const std::string &group_name, const std::vector<ModuleGroup> &groups) const {
    for (const auto &group : groups) {
      if (group.name == group_name) {
        size_t count = 0;
        for (const auto &module_name : group.modules)
          if (is_module_selected(module_name)) count++;
        return count;
      }
    }
    return 0;
  }
};

struct RenderItem {
  enum Type { GROUP_HEADER, MODULE_ITEM, SEPARATOR, INFO_TEXT };
  Type type;
  std::string text, identifier, counter_text;
  bool is_selectable = false, has_selected_items = false, is_disabled = false;
  int indent_level = 0;

  static RenderItem group_header(const std::string &name, const std::string &text, const std::string &counter) {
    return {GROUP_HEADER, text, name, counter, true, false, false, 0};
  }
  static RenderItem module_item(const std::string &name, const std::string &text, bool selected, int indent = 1) {
    return {MODULE_ITEM, (selected ? "■ " : "□ ") + text, name, "", true, false, false, indent};
  }
  static RenderItem separator() { return {SEPARATOR, "─", "", "", false, false, false, 0}; }
  static RenderItem info_text(const std::string &text) { return {INFO_TEXT, text, "", "", false, false, false, 0}; }
};

struct CoreData {
  std::vector<Module> modules;
  std::vector<ModuleGroup> groups;
  std::unordered_map<std::string, size_t> module_index, group_index;

  void build_indices() {
    module_index.clear(); group_index.clear();
    module_index.reserve(modules.size()); group_index.reserve(groups.size());
    for (size_t i = 0; i < modules.size(); ++i) module_index[modules[i].name] = i;
    for (size_t i = 0; i < groups.size(); ++i) group_index[groups[i].name] = i;
  }
  const Module* find_module(const std::string &name) const {
    auto it = module_index.find(name);
    return (it != module_index.end()) ? &modules[it->second] : nullptr;
  }
  const ModuleGroup* find_group(const std::string &name) const {
    auto it = group_index.find(name);
    return (it != group_index.end()) ? &groups[it->second] : nullptr;
  }
};

struct ConfigData {
  std::vector<Module> modules;
  std::vector<ModuleGroup> groups;
  static std::unique_ptr<ConfigData> load_from_file(const std::string &file_path);
};

} // namespace sunray_tui