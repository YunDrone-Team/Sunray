#pragma once

#include "tui_types.hpp"
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace sunray_tui {

/**
 * @brief 交互管理器 - 专门处理模块选择和冲突解决
 * 
 * 遵循Linus哲学：做好一件事 - 管理用户交互逻辑
 */
class InteractionManager {
public:
    explicit InteractionManager(const CoreData& core_data);

    // ==================== 模块状态管理 ====================
    
    /**
     * @brief 初始化模块状态
     */
    void init_module_states();
    
    /**
     * @brief 检查模块是否已被选中
     * @param module_name 模块名称
     * @return true如果模块已选中，false否则
     */
    bool is_module_selected(const std::string& module_name) const;
    
    /**
     * @brief 检查模块是否因冲突而被禁用
     * @param module_name 模块名称
     * @return true如果模块被禁用，false否则
     */
    bool is_module_disabled(const std::string& module_name) const;
    
    // ==================== 核心交互功能 ====================
    
    /**
     * @brief 切换模块的选择状态
     * @param module_name 模块名称
     */
    void toggle_module_selection(const std::string& module_name);
    
    /**
     * @brief 批量切换组内所有模块的选择状态
     * @param group_name 组名称
     * @return true如果操作成功，false如果组不存在或不支持批量操作
     */
    bool toggle_group_selection(const std::string& group_name);
    
    /**
     * @brief 实时重新计算所有模块的冲突状态
     */
    void resolve_conflicts_realtime();
    
    // ==================== 状态查询 ====================
    
    /**
     * @brief 获取所有已选中的模块名称列表
     * @return 已选中模块的名称向量
     */
    std::vector<std::string> get_selected_modules() const;
    
    /**
     * @brief 获取已选中模块的数量
     * @return 已选中模块数量
     */
    size_t selected_count() const;
    
    /**
     * @brief 获取所有模块状态的只读引用
     * @return 模块状态向量的常量引用
     */
    const std::vector<ModuleState>& get_module_states() const { return module_states_; }
    
    // ==================== 冲突处理 ====================
    
    /**
     * @brief 设置冲突指示回调函数
     * @param callback 冲突发生时调用的回调函数
     */
    void set_conflict_callback(std::function<void()> callback);
    
    /**
     * @brief 触发冲突指示（调用注册的回调函数）
     */
    void trigger_conflict_indication();
    
    // ==================== 状态重置 ====================
    
    /**
     * @brief 清除所有模块的选择状态
     */
    void clear_all_selections();

private:
    const CoreData& core_data_;
    std::vector<ModuleState> module_states_;
    std::unordered_map<std::string, size_t> module_state_index_;
    std::function<void()> conflict_callback_;
    
    // ==================== 内部方法 ====================
    
    /**
     * @brief 构建模块状态索引映射
     */
    void build_state_index();
    
    /**
     * @brief 检查指定模块是否与已选模块存在冲突
     * @param module_name 模块名称
     * @return true如果存在冲突，false否则
     */
    bool has_conflicts_with_selected(const std::string& module_name) const;
    
    /**
     * @brief 获取已选中模块的集合（用于快速查找）
     * @return 已选中模块名称的无序集合
     */
    std::unordered_set<std::string> get_selected_module_set() const;
};

} // namespace sunray_tui