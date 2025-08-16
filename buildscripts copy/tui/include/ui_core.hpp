#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include "screen_coordinate_mapper.hpp"

namespace sunray_tui {

// 模块数据结构 - 不可变，拷贝友好
struct Module {
    std::string name;
    std::string description;
    std::string source_path;
    std::string build_path;
    std::vector<std::string> dependencies;
    std::vector<std::string> conflicts_with;

    // 方便调试的输出
    std::string display_name() const {
        return name + " - " + description;
    }

    bool has_conflicts() const {
        return !conflicts_with.empty();
    }
};

// 模块状态管理 - 用于交互式冲突解决
struct ModuleState {
    std::string name;
    bool is_selected = false;    // 用户是否勾选了此模块
    bool is_disabled = false;    // 是否因冲突而被禁用（变灰不可选）
    
    ModuleState() = default;
    ModuleState(const std::string& module_name) : name(module_name) {}
};

// 模块组数据结构
struct ModuleGroup {
    std::string name;
    std::string description;
    std::vector<std::string> modules;

    std::string display_name() const {
        return name + " (" + std::to_string(modules.size()) + " modules) - " + description;
    }
};

// 视图状态 - 简化版本，只支持基础折叠展开
struct ViewState {
    std::unordered_set<std::string> expanded_groups;  // 展开的组
    std::unordered_set<std::string> selected_modules; // 选中的模块
    std::unordered_set<std::string> selected_groups;  // 选中的组
    std::string search_filter;                        // 搜索过滤器
    int scroll_offset = 0;                           // 滚动偏移

    // 查询函数
    bool is_group_expanded(const std::string& group_name) const {
        return expanded_groups.count(group_name) > 0;
    }

    bool is_module_selected(const std::string& module_name) const {
        return selected_modules.count(module_name) > 0;
    }

    bool is_group_selected(const std::string& group_name) const {
        return selected_groups.count(group_name) > 0;
    }

    // 操作函数
    void toggle_group_expansion(const std::string& group_name) {
        if (is_group_expanded(group_name)) {
            expanded_groups.erase(group_name);
        } else {
            expanded_groups.insert(group_name);
        }
    }

    void toggle_module_selection(const std::string& module_name) {
        if (is_module_selected(module_name)) {
            selected_modules.erase(module_name);
        } else {
            selected_modules.insert(module_name);
        }
    }

    void toggle_group_selection(const std::string& group_name) {
        if (is_group_selected(group_name)) {
            selected_groups.erase(group_name);
        } else {
            selected_groups.insert(group_name);
            // 选中组时自动展开
            expanded_groups.insert(group_name);
        }
    }

    void clear_selection() {
        selected_modules.clear();
        selected_groups.clear();
    }

    size_t selected_count() const {
        return selected_modules.size();  // 只计算模块数量
    }
    
    // 计算组内选中的模块数量
    size_t selected_count_in_group(const std::string& group_name, const std::vector<ModuleGroup>& groups) const {
        for (const auto& group : groups) {
            if (group.name == group_name) {
                size_t count = 0;
                for (const auto& module_name : group.modules) {
                    if (is_module_selected(module_name)) {
                        count++;
                    }
                }
                return count;
            }
        }
        return 0;
    }
};

// 渲染项 - 统一的界面元素表示
struct RenderItem {
    enum Type {
        GROUP_HEADER,    // 组标题 (可折叠)
        MODULE_ITEM,     // 模块项 (可选择)
        SEPARATOR,       // 分隔符
        INFO_TEXT        // 信息文本
    };

    Type type;
    std::string text;
    std::string identifier;    // 用于状态查询的唯一标识
    std::string counter_text;  // 用于存储计数器文本（仅GROUP_HEADER使用）
    bool is_selectable = false;
    bool is_collapsible = false;
    bool has_selected_items = false;  // 用于标识组是否有选中的模块
    bool is_disabled = false;         // 用于标识模块是否因冲突被禁用
    int indent_level = 0;

    // 静态创建函数 - Linus风格的简洁接口
    static RenderItem group_header(const std::string& group_name, const std::string& text, const std::string& counter, bool is_expanded) {
        RenderItem item;
        item.type = GROUP_HEADER;
        item.text = text;  // 组名部分
        item.counter_text = counter;  // 计数器部分
        item.identifier = group_name;
        item.is_collapsible = true;
        return item;
    }

    static RenderItem module_item(const std::string& module_name, const std::string& text, bool is_selected, int indent = 1) {
        RenderItem item;
        item.type = MODULE_ITEM;
        item.text = (is_selected ? "■ " : "□ ") + text;  // 使用实心方块和空心方块
        item.identifier = module_name;
        item.is_selectable = true;
        item.indent_level = indent;
        return item;
    }

    static RenderItem separator() {
        RenderItem item;
        item.type = SEPARATOR;
        item.text = "─";
        return item;
    }

    static RenderItem info_text(const std::string& text) {
        RenderItem item;
        item.type = INFO_TEXT;
        item.text = text;
        return item;
    }
};

// UI状态 - 单一的状态存储
struct UIState {
    std::vector<Module> modules;
    std::vector<ModuleGroup> groups;
    ViewState view;
    
    // 当前选中的项目索引 (用于键盘导航)
    int current_selection = 0;
    
    // 鼠标hover状态 - 用于提供视觉反馈
    int mouse_hover_index = -1;  // -1表示没有hover项
    
    // 动画状态 - 用于流畅的过渡效果
    int previous_selection = -1;  // 用于动画过渡的前一个选择位置
    bool animation_in_progress = false;  // 标识是否正在执行动画
    
    std::vector<RenderItem> render_items;  // 当前渲染项列表
    
    // 交互式冲突解决系统
    std::vector<ModuleState> module_states;  // 所有模块的实时状态
    std::unordered_map<std::string, size_t> module_state_index;  // 快速查找模块状态索引
    
    // 当前选中项的详细信息
    std::string current_item_description;
    std::string current_item_details;
    
    // 构建按钮和对话框状态
    bool show_build_dialog = false;
    bool build_requested = false;       // TUI请求执行构建的标志
    bool build_button_focused = false;
    bool build_button_hovered = false;  // 鼠标hover状态
    int build_button_screen_y = -1;  // 编译按钮在屏幕上的Y坐标
    
    // 冲突提示闪烁状态
    bool conflict_flash_active = false;
    int conflict_flash_count = 0;
    const int max_flash_count = 6;  // 闪烁3次（亮暗各3次）

    // 高效查找
    std::unordered_map<std::string, size_t> module_index;
    std::unordered_map<std::string, size_t> group_index;
    
    // 统一的屏幕坐标映射器 - Linus风格的核心组件
    ScreenCoordinateMapper coordinate_mapper;

    // 初始化索引
    void build_indices() {
        module_index.clear();
        group_index.clear();
        
        module_index.reserve(modules.size());
        for (size_t i = 0; i < modules.size(); ++i) {
            module_index[modules[i].name] = i;
        }
        
        group_index.reserve(groups.size());
        for (size_t i = 0; i < groups.size(); ++i) {
            group_index[groups[i].name] = i;
        }
        
        // 初始化模块状态管理
        init_module_states();
        
        // 初始化坐标映射器的终端适配
        coordinate_mapper.auto_detect_terminal_offset();
    }
    
    // 初始化交互式冲突解决系统
    void init_module_states() {
        module_states.clear();
        module_state_index.clear();
        
        module_states.reserve(modules.size());
        for (size_t i = 0; i < modules.size(); ++i) {
            module_states.emplace_back(modules[i].name);
            module_state_index[modules[i].name] = i;
        }
    }
    
    // 核心冲突解决算法 - 实时更新所有模块状态
    void resolve_conflicts_realtime() {
        // 1. 收集当前所有已选模块的集合
        std::unordered_set<std::string> selected_modules;
        for (const auto& state : module_states) {
            if (state.is_selected) {
                selected_modules.insert(state.name);
            }
        }
        
        // 2. 重新计算每个模块的禁用状态
        for (auto& state : module_states) {
            // 首先假设模块可用
            state.is_disabled = false;
            
            // 如果模块本身已被选中，不需要检查冲突
            if (state.is_selected) {
                continue;
            }
            
            // 检查是否与任何已选模块冲突
            const Module* module = find_module(state.name);
            if (module) {
                for (const std::string& conflict : module->conflicts_with) {
                    if (selected_modules.count(conflict)) {
                        state.is_disabled = true;
                        break; // 找到一个冲突就足够了
                    }
                }
            }
        }
    }
    
    // 切换模块选择状态并解决冲突
    void toggle_module_selection_with_conflicts(const std::string& module_name) {
        auto it = module_state_index.find(module_name);
        if (it != module_state_index.end()) {
            ModuleState& state = module_states[it->second];
            
            // 如果模块被禁用，不允许选择
            if (state.is_disabled && !state.is_selected) {
                return;
            }
            
            // 切换选择状态
            state.is_selected = !state.is_selected;
            
            // 实时重新计算所有冲突
            resolve_conflicts_realtime();
            
            // 更新传统的view状态以保持兼容性
            if (state.is_selected) {
                view.selected_modules.insert(module_name);
            } else {
                view.selected_modules.erase(module_name);
            }
        }
    }
    
    // 查询模块状态
    bool is_module_disabled(const std::string& module_name) const {
        auto it = module_state_index.find(module_name);
        return (it != module_state_index.end()) ? module_states[it->second].is_disabled : false;
    }
    
    bool is_module_selected_new(const std::string& module_name) const {
        auto it = module_state_index.find(module_name);
        return (it != module_state_index.end()) ? module_states[it->second].is_selected : false;
    }

    // 查找函数
    const Module* find_module(const std::string& name) const {
        auto it = module_index.find(name);
        return (it != module_index.end()) ? &modules[it->second] : nullptr;
    }

    const ModuleGroup* find_group(const std::string& name) const {
        auto it = group_index.find(name);
        return (it != group_index.end()) ? &groups[it->second] : nullptr;
    }

    // 渲染列表生成 - 核心逻辑
    void update_render_items();
    
    // 检测未归类模块并创建虚拟ungrouped组
    void ensure_ungrouped_modules();
    
    // 导航功能
    void move_selection_up();
    void move_selection_down();
    RenderItem* get_current_item();
    const RenderItem* get_current_item() const;

    // 事件处理
    bool handle_expand_toggle();    // 处理展开/折叠（空格键）
    bool handle_selection();        // 处理选择（回车键）
    void handle_search_update(const std::string& filter);
    void update_current_item_info();  // 更新当前选中项信息
    
    // 焦点和按钮管理
    void handle_tab_focus();        // 处理Tab键焦点切换
    void handle_build_button();     // 处理编译按钮点击
    void close_build_dialog();      // 关闭构建对话框
    void trigger_conflict_flash();   // 触发冲突提示闪烁
    void update_conflict_flash();    // 更新闪烁状态
};

// 配置数据 - 从YAML加载的原始数据
struct ConfigData {
    std::vector<Module> modules;
    std::vector<ModuleGroup> groups;
    
    // 简单工厂函数
    static std::unique_ptr<ConfigData> load_from_file(const std::string& file_path);
    
    // 转换为UI状态
    UIState create_ui_state() const {
        UIState state;
        state.modules = modules;
        state.groups = groups;
        state.build_indices();
        
        // Linus风格：默认展开第一个组，让用户立即看到内容
        if (!state.groups.empty()) {
            state.view.expanded_groups.insert(state.groups[0].name);
        }
        
        state.update_render_items();
        state.update_current_item_info();  // 初始化当前项信息
        return state;
    }
};

// 内联实现一些简单函数
inline void UIState::move_selection_up() {
    int original_selection = current_selection;
    while (current_selection > 0) {
        current_selection--;
        // 跳过不可交互的项目（分隔符和信息文本）
        if (current_selection >= 0 && current_selection < static_cast<int>(render_items.size())) {
            const auto& item = render_items[current_selection];
            if (item.type != RenderItem::SEPARATOR && item.type != RenderItem::INFO_TEXT) {
                return; // 找到可选择的项目
            }
        }
    }
    // 如果没找到可选择项，恢复原始位置
    current_selection = original_selection;
}

inline void UIState::move_selection_down() {
    int original_selection = current_selection;
    while (current_selection < static_cast<int>(render_items.size()) - 1) {
        current_selection++;
        // 跳过不可交互的项目（分隔符和信息文本）
        if (current_selection >= 0 && current_selection < static_cast<int>(render_items.size())) {
            const auto& item = render_items[current_selection];
            if (item.type != RenderItem::SEPARATOR && item.type != RenderItem::INFO_TEXT) {
                return; // 找到可选择的项目
            }
        }
    }
    // 如果没找到可选择项，恢复原始位置
    current_selection = original_selection;
}

inline RenderItem* UIState::get_current_item() {
    if (current_selection >= 0 && current_selection < static_cast<int>(render_items.size())) {
        return &render_items[current_selection];
    }
    return nullptr;
}

inline const RenderItem* UIState::get_current_item() const {
    if (current_selection >= 0 && current_selection < static_cast<int>(render_items.size())) {
        return &render_items[current_selection];
    }
    return nullptr;
}

} // namespace sunray_tui