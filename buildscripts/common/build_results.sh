#!/bin/bash
# 构建结果显示模块

# 导入必要的配置和工具函数
if ! declare -f print_status >/dev/null 2>&1; then
    source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
fi
if ! declare -f get_module_description >/dev/null 2>&1; then
    source "$(dirname "${BASH_SOURCE[0]}")/config.sh"
fi

# 显示构建结果表格

display_build_results_table() {
    local total_modules=${#BUILD_RESULTS_MODULES[@]}
    [[ $total_modules -eq 0 ]] && return 0
    
    echo
    
    # --- 计算宽度 ---
    local title="构建结果汇总"
    local max_module_width=0
    for module in "${BUILD_RESULTS_MODULES[@]}"; do
        local len=${#module}
        [[ $len -gt $max_module_width ]] && max_module_width=$len
    done
    
    # 增加一点右边距
    max_module_width=$((max_module_width + 4))

    # --- 打印标题 (简化) ---
    echo "${CYAN}=== ${title} ===${NC}"
    echo
    
    # --- 表头 (无竖线) ---
    # 为了清晰，我们将状态和用时放在一行
    printf "%-${max_module_width}s %s\n" "模块" "状态 / 用时"
    # 打印一个简单的下划线作为分隔
    printf "%*s\n" $((max_module_width + 20)) "" | tr ' ' '-'

    # --- 表格内容 ---
    for i in "${!BUILD_RESULTS_MODULES[@]}"; do
        local module="${BUILD_RESULTS_MODULES[i]}"
        local status="${BUILD_RESULTS_STATUS[i]}"
        local time="${BUILD_RESULTS_TIMES[i]}"
        
        if [[ "$status" == "SUCCESS" ]]; then
            # 模块名左对齐，状态和时间直接跟在后面
            printf "%-${max_module_width}s ${GREEN}%s${NC} (%s)\n" "$module" "✓ 成功" "$time"
        else
            printf "${RED}%-${max_module_width}s${NC} %s (%s)\n" "$module" "${RED}✗ 失败${NC}" "$time"
        fi
    done
    
    echo
    
    # --- 统计信息 (保持不变) ---
    local success_count=0
    local failed_count=0
    local failed_modules=()
    
    for i in "${!BUILD_RESULTS_STATUS[@]}"; do
        if [[ "${BUILD_RESULTS_STATUS[i]}" == "SUCCESS" ]]; then
            ((success_count++))
        else
            ((failed_count++))
            failed_modules+=("${BUILD_RESULTS_MODULES[i]}")
        fi
    done
    
    echo "${YELLOW}构建统计:${NC} 总计 $total_modules 个模块, ${GREEN}成功 $success_count${NC}, ${RED}失败 $failed_count${NC}"
    
    if [[ $failed_count -gt 0 ]]; then
        echo
        echo "${RED}失败的模块:${NC}"
        for module in "${failed_modules[@]}"; do
            local description # 省略 get_module_description 的实现
            description=$(get_module_description "$module" 2>/dev/null || echo "")
            if [[ -n "$description" ]]; then
                echo "  ${RED}•${NC} ${BRIGHT_WHITE}$module${NC} ${DARK_GRAY}- $description${NC}"
            else
                echo "  ${RED}•${NC} ${BRIGHT_WHITE}$module${NC}"
            fi
        done
    fi
    
    echo
}

# 记录构建开始
record_build_start() {
    local module="$1"
    BUILD_RESULTS_MODULES+=("$module")
    BUILD_RESULTS_STATUS+=("BUILDING")
    BUILD_RESULTS_TIMES+=("0s")
    BUILD_RESULTS_START_TIMES+=("$(date +%s)")
}

# 更新构建结果
update_build_result() {
    local module="$1"
    local status="$2"
    local end_time="${3:-$(date +%s)}"
    
    # 找到模块在数组中的位置
    for i in "${!BUILD_RESULTS_MODULES[@]}"; do
        if [[ "${BUILD_RESULTS_MODULES[i]}" == "$module" ]]; then
            BUILD_RESULTS_STATUS[i]="$status"
            
            # 计算用时
            local start_time="${BUILD_RESULTS_START_TIMES[i]:-$end_time}"
            local duration=$((end_time - start_time))
            BUILD_RESULTS_TIMES[i]="$(format_build_duration $duration)"
            break
        fi
    done
}

# 格式化持续时间显示
format_build_duration() {
    local seconds=$1
    if [[ $seconds -lt 60 ]]; then
        echo "${seconds}s"
    elif [[ $seconds -lt 3600 ]]; then
        echo "$((seconds / 60))m$((seconds % 60))s"
    else
        echo "$((seconds / 3600))h$(((seconds % 3600) / 60))m$((seconds % 60))s"
    fi
}