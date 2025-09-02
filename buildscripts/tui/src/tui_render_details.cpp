#include "tui_render.hpp"

namespace sunray_tui {

// ==================== hover时更新details ====================

void UIRenderer::update_details_on_hover() {
  // 🔥 hover时实时更新details区域信息
  // 清空现有信息
  state_.current_item_description.clear();
  state_.current_item_details.clear();

  // 检查是否有组被hover
  if (state_.group_hover_index >= 0 &&
      state_.group_hover_index <
          static_cast<int>(state_.group_render_items.size())) {
    // 显示hover组的信息
    const auto &group_item =
        state_.group_render_items[state_.group_hover_index];
    const ModuleGroup *group = state_.find_group(group_item.identifier);
    if (group) {
      state_.current_item_description =
          group->description.empty() ? "NULL" : group->description;
      state_.current_item_details =
          "包含 " + std::to_string(group->modules.size()) + " 个模块";
    }
    return;
  }

  // 检查是否有模块被hover
  if (state_.module_hover_index >= 0 &&
      state_.module_hover_index <
          static_cast<int>(state_.module_render_items.size())) {
    // 显示hover模块的信息
    const auto &module_item =
        state_.module_render_items[state_.module_hover_index];
    const Module *module = state_.find_module(module_item.identifier);
    if (module) {
      // 设置模块描述
      state_.current_item_description =
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

      // 路径信息
      details += "路径: " +
                 (module->source_path.empty() ? "NULL" : module->source_path);

      state_.current_item_details = details;
    }
    return;
  }

  // 如果没有hover任何item，根据当前焦点显示selection的信息（回退到原有逻辑）
  state_.update_current_item_info();
}

} // namespace sunray_tui