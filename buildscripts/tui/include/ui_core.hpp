#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include "screen_coordinate_mapper.hpp"
// 简化版TRACE宏，仅用于关键编译输出
#define TRACE_CRITICAL(tag, msg) do { \
    std::cout << "[" << tag << "] " << msg << std::endl; \
} while(0)
#include "core/build_output_manager.hpp"

namespace sunray_tui {

// Linus风格统一焦点管理 - 消除特殊情况
enum class FocusTarget {
    GROUPS = 0,
    MODULES = 1,
    BUILD_BUTTON = 2,
    BUILD_OUTPUT = 3  // 新增：编译输出窗口焦点
};

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

// Linus-style simplified state - eliminate complexity, focus on the problem
struct TwoSectionState {
    // Core selection state - only what matters
    std::unordered_set<std::string> selected_modules;     // User-selected modules
    std::unordered_set<std::string> highlighted_groups;   // Groups with all modules selected
    
    // Dual navigation cursors - separate tracking for each section
    int groups_cursor = 0;       // Current position in groups section
    int modules_cursor = 0;      // Current position in modules section  
    bool focus_on_groups = true; // Which section has focus
    
    // Performance map - group name to module names
    std::unordered_map<std::string, std::vector<std::string>> group_to_modules;
    
    // Animation and hover state management
    std::chrono::steady_clock::time_point animation_start_time;
    bool animation_active = false;
    bool hover_animation_active = false;
    double hover_animation_phase = 0.0;  // 0.0 to 1.0 for breathing effect
    
    // Timing for smooth animations
    static constexpr double HOVER_ANIMATION_SPEED = 2.0;  // cycles per second
    static constexpr double ANIMATION_FRAME_TIME_MS = 16.0;  // ~60 FPS
    
    // Query functions - simple and direct
    bool is_module_selected(const std::string& module_name) const {
        return selected_modules.count(module_name) > 0;
    }
    
    bool is_group_highlighted(const std::string& group_name) const {
        return highlighted_groups.count(group_name) > 0;
    }
    
    // Selection operations
    void select_module(const std::string& module_name) {
        selected_modules.insert(module_name);
    }
    
    void deselect_module(const std::string& module_name) {
        selected_modules.erase(module_name);
    }
    
    void highlight_group(const std::string& group_name) {
        highlighted_groups.insert(group_name);
    }
    
    void unhighlight_group(const std::string& group_name) {
        highlighted_groups.erase(group_name);
    }
    
    void clear_selection() {
        selected_modules.clear();
        highlighted_groups.clear();
    }
    
    size_t selected_count() const {
        return selected_modules.size();
    }
    
    // Check if all modules in a group are selected
    bool is_group_complete(const std::string& group_name) const {
        auto it = group_to_modules.find(group_name);
        if (it == group_to_modules.end()) return false;
        
        for (const auto& module : it->second) {
            if (selected_modules.find(module) == selected_modules.end()) {
                return false;
            }
        }
        return !it->second.empty(); // Don't highlight empty groups
    }
    
    // Initialize group-to-modules mapping
    void build_group_mapping(const std::vector<ModuleGroup>& groups) {
        group_to_modules.clear();
        for (const auto& group : groups) {
            group_to_modules[group.name] = group.modules;
        }
    }
    
    // Animation control methods
    void start_hover_animation() {
        if (!hover_animation_active) {
            hover_animation_active = true;
            animation_start_time = std::chrono::steady_clock::now();
            animation_active = true;
        }
    }
    
    void stop_hover_animation() {
        hover_animation_active = false;
        hover_animation_phase = 0.0;
    }
    
    void update_hover_animation() {
        if (!hover_animation_active) return;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - animation_start_time);
        double time_seconds = elapsed.count() / 1000.0;
        
        // Create breathing effect: sine wave from 0.0 to 1.0
        hover_animation_phase = (std::sin(time_seconds * HOVER_ANIMATION_SPEED * 2.0 * M_PI) + 1.0) / 2.0;
    }
    
    // Check if we need animation frame updates
    bool needs_animation_frame() const {
        return animation_active && hover_animation_active;
    }
    
    void stop_all_animations() {
        animation_active = false;
        hover_animation_active = false;
        hover_animation_phase = 0.0;
    }
};

// Linus-style SelectionManager - handle bidirectional sync between groups and modules
class SelectionManager {
public:
    explicit SelectionManager(TwoSectionState& state) : state_(state) {}
    
    // Select/deselect a group - affects all its modules
    void toggle_group_selection(const std::string& group_name) {
        if (state_.is_group_highlighted(group_name)) {
            // Group is currently highlighted, deselect all its modules
            deselect_group(group_name);
        } else {
            // Group is not highlighted, select all its modules
            select_group(group_name);
        }
        update_group_highlighting();
    }
    
    // Select/deselect a module - may affect group highlighting
    void toggle_module_selection(const std::string& module_name) {
        if (state_.is_module_selected(module_name)) {
            state_.deselect_module(module_name);
        } else {
            state_.select_module(module_name);
        }
        update_group_highlighting();
    }
    
    // Clear all selections
    void clear_all_selections() {
        state_.clear_selection();
    }
    
    // Get currently selected modules for build system
    std::vector<std::string> get_selected_modules() const {
        std::vector<std::string> result;
        result.reserve(state_.selected_modules.size());
        for (const auto& module_name : state_.selected_modules) {
            result.push_back(module_name);
        }
        return result;
    }

private:
    TwoSectionState& state_;
    
    // Select all modules in a group
    void select_group(const std::string& group_name) {
        auto it = state_.group_to_modules.find(group_name);
        if (it != state_.group_to_modules.end()) {
            for (const auto& module_name : it->second) {
                state_.select_module(module_name);
            }
        }
    }
    
    // Deselect all modules in a group
    void deselect_group(const std::string& group_name) {
        auto it = state_.group_to_modules.find(group_name);
        if (it != state_.group_to_modules.end()) {
            for (const auto& module_name : it->second) {
                state_.deselect_module(module_name);
            }
        }
    }
    
    // Update group highlighting based on module selections
    void update_group_highlighting() {
        // Clear all group highlights first
        state_.highlighted_groups.clear();
        
        // Check each group to see if all modules are selected
        for (const auto& [group_name, modules] : state_.group_to_modules) {
            if (state_.is_group_complete(group_name)) {
                state_.highlight_group(group_name);
            }
        }
    }
};

// Linus-style simplified UI state - eliminate unnecessary complexity
struct UIState {
    // Core data - unchanged
    std::vector<Module> modules;
    std::vector<ModuleGroup> groups;
    
    // New simplified state management
    TwoSectionState two_section;
    SelectionManager selection_manager;
    
    // Current item details for description section
    std::string current_item_description;
    std::string current_item_details;
    
    // Build system integration - 新架构：TUI内部显示编译输出
    bool build_requested = false;
    bool build_button_focused = false;  // 保留向后兼容，从current_focus派生
    bool build_running = false;         // 编译是否正在进行
    bool build_output_visible = false;  // 编译输出窗口是否可见
    int build_output_scroll = 0;        // 编译输出滚动位置
    
    // Linus-style编译输出管理器
    std::unique_ptr<BuildOutputManager> build_manager;
    
    // Linus风格：直接退出回调，解决Enter键延迟问题
    std::function<void()> screen_exit_callback;
    
    // Linus风格统一焦点状态 - 单一数据源
    FocusTarget current_focus = FocusTarget::GROUPS;
    
    // Conflict detection and flash animation system
    std::vector<ModuleState> module_states;  // Real-time module state tracking
    std::unordered_map<std::string, size_t> module_state_index;  // Fast lookup
    bool conflict_flash_active = false;
    int conflict_flash_count = 0;
    const int max_flash_count = 6;  // Flash 3 times (on/off = 6 states)
    
    // Performance lookups
    std::unordered_map<std::string, size_t> module_index;
    std::unordered_map<std::string, size_t> group_index;
    
    // Constructor - initialize SelectionManager with TwoSectionState
    UIState() : selection_manager(two_section) {
        build_manager = std::make_unique<BuildOutputManager>();
    }
    
    // Initialize everything - Linus style: do it once, do it right
    void initialize() {
        build_indices();
        two_section.build_group_mapping(groups);
        update_focus_state();  // 确保派生状态同步
        update_current_item_info();
    }
    
    // Build efficient lookup indices
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
        
        // Initialize conflict detection system
        init_module_states();
    }
    
    // Initialize conflict detection module states
    void init_module_states() {
        module_states.clear();
        module_state_index.clear();
        
        module_states.reserve(modules.size());
        for (size_t i = 0; i < modules.size(); ++i) {
            module_states.emplace_back(modules[i].name);
            module_state_index[modules[i].name] = i;
        }
    }
    
    // Update description for currently focused item
    void update_current_item_info() {
        current_item_description.clear();
        current_item_details.clear();
        
        if (two_section.focus_on_groups) {
            if (two_section.groups_cursor < static_cast<int>(groups.size())) {
                const auto& group = groups[two_section.groups_cursor];
                current_item_description = group.description.empty() ? "NULL" : group.description;
                current_item_details = "包含 " + std::to_string(group.modules.size()) + " 个模块";
            }
        } else {
            if (two_section.modules_cursor < static_cast<int>(modules.size())) {
                const auto& module = modules[two_section.modules_cursor];
                
                // Set description
                current_item_description = module.description.empty() ? "NULL" : module.description;
                
                // Build structured details: 4-line format
                std::string details;
                
                // Line 1: Dependencies
                details += "依赖: ";
                if (module.dependencies.empty()) {
                    details += "NULL";
                } else {
                    for (size_t i = 0; i < module.dependencies.size(); ++i) {
                        if (i > 0) details += ", ";
                        details += module.dependencies[i];
                    }
                }
                details += "\n";
                
                // Line 2: Conflicts 
                details += "冲突: ";
                if (module.conflicts_with.empty()) {
                    details += "NULL";
                } else {
                    for (size_t i = 0; i < module.conflicts_with.size(); ++i) {
                        if (i > 0) details += ", ";
                        details += module.conflicts_with[i];
                    }
                }
                details += "\n";
                
                // Line 3: Path
                details += "路径: ";
                details += module.source_path.empty() ? "NULL" : module.source_path;
                
                current_item_details = details;
            }
        }
    }
    
    // Hover state management - triggers breathing animations
    void handle_focus_change() {
        // Start hover animation when focus changes
        two_section.start_hover_animation();
        update_current_item_info();
    }
    
    void update_animations() {
        // Update all active animations
        two_section.update_hover_animation();
        update_conflict_flash();  // Update conflict flash animation
    }
    
    bool needs_animation_updates() const {
        return two_section.needs_animation_frame() || conflict_flash_active;
    }
    
    // Linus风格统一焦点管理API - 消除复杂度
    void cycle_focus_forward() {
        current_focus = static_cast<FocusTarget>((static_cast<int>(current_focus) + 1) % 3);
        update_focus_state();
    }
    
    void cycle_focus_backward() {
        current_focus = static_cast<FocusTarget>((static_cast<int>(current_focus) + 2) % 3);  // +2 % 3 = -1 % 3
        update_focus_state();
    }
    
    void set_focus_target(FocusTarget target) {
        current_focus = target;
        update_focus_state();
    }
    
    // 焦点状态查询 - 清晰的API
    bool is_groups_focused() const { return current_focus == FocusTarget::GROUPS; }
    bool is_modules_focused() const { return current_focus == FocusTarget::MODULES; }
    bool is_build_button_focused() const { return current_focus == FocusTarget::BUILD_BUTTON; }
    
    // 派生状态更新 - 确保向后兼容
    void update_focus_state() {
        two_section.focus_on_groups = (current_focus == FocusTarget::GROUPS);
        build_button_focused = (current_focus == FocusTarget::BUILD_BUTTON);
        update_current_item_info();  // 焦点变化时更新信息
    }
    
    // Conflict detection and resolution system
    void resolve_conflicts_realtime() {
        // 1. Collect currently selected modules
        std::unordered_set<std::string> selected_modules;
        for (const auto& state : module_states) {
            if (state.is_selected) {
                selected_modules.insert(state.name);
            }
        }
        
        // 2. Update disabled state for all modules
        for (auto& state : module_states) {
            state.is_disabled = false;  // Reset
            
            // Skip if module is already selected
            if (state.is_selected) continue;
            
            // Check for conflicts with selected modules
            const Module* module = find_module(state.name);
            if (module) {
                for (const std::string& conflict : module->conflicts_with) {
                    if (selected_modules.count(conflict)) {
                        state.is_disabled = true;
                        break;
                    }
                }
            }
        }
    }
    
    void toggle_module_selection_with_conflicts(const std::string& module_name) {
        auto it = module_state_index.find(module_name);
        if (it != module_state_index.end()) {
            ModuleState& state = module_states[it->second];
            
            // Don't allow selection of disabled modules
            if (state.is_disabled && !state.is_selected) {
                trigger_conflict_flash();  // Flash the conflict info
                return;
            }
            
            // Toggle selection
            state.is_selected = !state.is_selected;
            
            // Update our selection manager to keep sync
            if (state.is_selected) {
                two_section.select_module(module_name);
            } else {
                two_section.deselect_module(module_name);
            }
            
            // Update conflict states
            resolve_conflicts_realtime();
        }
    }
    
    // Conflict flash animation methods
    void trigger_conflict_flash() {
        conflict_flash_active = true;
        conflict_flash_count = 0;
    }
    
    void update_conflict_flash() {
        if (conflict_flash_active) {
            conflict_flash_count++;
            if (conflict_flash_count >= max_flash_count) {
                conflict_flash_active = false;
                conflict_flash_count = 0;
            }
        }
    }
    
    // Query module states
    bool is_module_disabled(const std::string& module_name) const {
        auto it = module_state_index.find(module_name);
        return (it != module_state_index.end()) ? module_states[it->second].is_disabled : false;
    }
    
    bool is_module_selected_new(const std::string& module_name) const {
        auto it = module_state_index.find(module_name);
        return (it != module_state_index.end()) ? module_states[it->second].is_selected : false;
    }
    
    // Find helper methods
    const Module* find_module(const std::string& name) const {
        auto it = module_index.find(name);
        return (it != module_index.end()) ? &modules[it->second] : nullptr;
    }
    
    const ModuleGroup* find_group(const std::string& name) const {
        auto it = group_index.find(name);
        return (it != group_index.end()) ? &groups[it->second] : nullptr;
    }
    
    // Get currently selected modules for build system
    std::vector<std::string> get_selected_modules() const {
        return selection_manager.get_selected_modules();
    }
    
    // Build system state management methods
    void start_build(const std::string& build_command) {
        if (build_manager) {
            build_running = build_manager->start_build(build_command);
            build_output_visible = build_running;
            build_output_scroll = 0;
            if (build_running) {
                current_focus = FocusTarget::BUILD_OUTPUT;
                TRACE_CRITICAL("BUILD_UI_START", "Build started, switching to output view");
            }
        }
    }
    
    void update_build_status() {
        if (build_manager) {
            build_manager->update();
            auto state = build_manager->get_state();
            
            bool was_running = build_running;
            build_running = (state == BuildOutputManager::BuildState::RUNNING);
            
            // 如果编译刚完成，记录日志
            if (was_running && !build_running) {
                if (state == BuildOutputManager::BuildState::COMPLETED) {
                    TRACE_CRITICAL("BUILD_UI_COMPLETE", "Build completed successfully");
                } else if (state == BuildOutputManager::BuildState::FAILED) {
                    TRACE_CRITICAL("BUILD_UI_FAILED", "Build failed with exit code: " + 
                                 std::to_string(build_manager->get_exit_code()));
                }
            }
        }
    }
    
    void stop_build() {
        if (build_manager) {
            build_manager->stop_build();
            build_running = false;
            TRACE_CRITICAL("BUILD_UI_STOP", "Build stopped by user");
        }
    }
    
    bool is_build_running() const {
        return build_running;
    }
    
    bool is_build_output_visible() const {
        return build_output_visible;
    }
    
    void toggle_build_output_visibility() {
        build_output_visible = !build_output_visible;
        if (build_output_visible) {
            current_focus = FocusTarget::BUILD_OUTPUT;
        }
    }
    
    std::vector<std::string> get_build_output_lines(size_t max_lines = 100) const {
        if (build_manager) {
            return build_manager->get_latest_lines(max_lines);
        }
        return {};
    }
    
    size_t get_build_output_line_count() const {
        return build_manager ? build_manager->get_output_line_count() : 0;
    }
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
        state.initialize();  // Use our new Linus-style initialization
        return state;
    }
};

} // namespace sunray_tui