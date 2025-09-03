#!/bin/bash
# UI交互层

if ! declare -f print_status >/dev/null 2>&1; then
    source "$(dirname "${BASH_SOURCE[0]}")/../common/utils.sh"
fi
if ! declare -f validate_config >/dev/null 2>&1; then
    source "$(dirname "${BASH_SOURCE[0]}")/../common/config.sh"
fi

# Linus-style: Embedded terminal reset functions - no external dependencies
reset_terminal() {
    # Essential ANSI sequences to clean terminal state
    printf '\033[?25h'      # Show cursor
    printf '\033[0m'        # Reset all attributes (colors, styles)
    printf '\033[?1000l'    # Disable X10 mouse reporting
    printf '\033[?1001l'    # Disable highlight mouse tracking
    printf '\033[?1002l'    # Disable button event tracking
    printf '\033[?1003l'    # Disable any event tracking
    printf '\033[?1004l'    # Disable focus events
    printf '\033[?1005l'    # Disable UTF-8 mouse mode
    printf '\033[?1006l'    # Disable SGR mouse mode
    printf '\033[?1015l'    # Disable Urxvt mouse mode
    printf '\033[?2004l'    # Disable bracketed paste mode
    printf '\033[?47l'      # Exit alternate screen mode
    printf '\033[?1049l'    # Exit alternate screen + cursor save
    printf '\033[c'         # Soft reset terminal
    sync
}

# Linus-style: 激进的鼠标控制字符过滤器 - 专门对付TUI泄漏
filter_mouse_control_chars() {
    if [[ "$FROM_TUI" == true ]]; then
        # 清空stdin中的所有鼠标控制序列
        # 使用超时0.1秒读取并丢弃所有待处理输入
        while IFS= read -r -t 0.1 -s line 2>/dev/null; do
            # 静默丢弃所有输入 - 专门清理鼠标事件
            :
        done
        
        # 额外的stdin清理 - 确保没有缓冲遗留
        if [[ -t 0 ]]; then
            # 如果stdin是终端，进行tcflush等效操作
            exec 0</dev/null
            exec 0</dev/tty
        fi
    fi
}

cleanup_on_error() {
    reset_terminal
    echo "❌ Build failed - terminal state reset"
}

cleanup_on_exit() {
    reset_terminal
    echo "🔧 Terminal state reset"
}

# UI变量
SCRIPT_NAME="$(basename "$0")"
DRY_RUN=false
AUTO_YES=false
FROM_TUI=false
HAD_CONFLICTS=false
BUILD_JOBS=0
SELECTED_MODULES=()

# 隐藏文件路径
LAST_SELECTION_FILE="buildscripts/tui/build/.sunray_last_build_selection"

# 保存模块选择到隐藏文件
save_module_selection() {
    local modules=("$@")
    if [[ ${#modules[@]} -eq 0 ]]; then
        return 0
    fi
    
    # 创建临时文件以确保原子性写入
    local temp_file="${LAST_SELECTION_FILE}.tmp"
    {
        echo "# Sunray构建系统 - 上次模块选择"
        echo "# 生成时间: $(date)"
        echo "# 格式: 每行一个模块名"
        echo ""
        printf '%s\n' "${modules[@]}"
    } > "$temp_file" && mv "$temp_file" "$LAST_SELECTION_FILE"
    
    print_status "已保存模块选择到: $LAST_SELECTION_FILE"
}

# 从隐藏文件读取上次选择的模块
load_last_selection() {
    if [[ ! -f "$LAST_SELECTION_FILE" ]]; then
        print_status "未找到上次选择记录: $LAST_SELECTION_FILE"
        return 1
    fi
    
    local loaded_modules=()
    while IFS= read -r line; do
        # 跳过注释行和空行
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        [[ -z "${line// }" ]] && continue
        
        # 如果行包含多个模块（空格分隔），分割成单独的模块
        if [[ "$line" =~ [[:space:]] ]]; then
            # 使用read -a将空格分隔的字符串分割为数组
            local modules_in_line
            read -ra modules_in_line <<< "$line"
            for module in "${modules_in_line[@]}"; do
                # 清理每个模块名
                local clean_module="${module## }"   # 移除前置空格
                clean_module="${clean_module%% }"   # 移除后置空格
                [[ -n "$clean_module" ]] && loaded_modules+=("$clean_module")
            done
        else
            # 单个模块的情况
            local clean_module="${line## }"   # 移除前置空格
            clean_module="${clean_module%% }" # 移除后置空格
            [[ -n "$clean_module" ]] && loaded_modules+=("$clean_module")
        fi
    done < "$LAST_SELECTION_FILE"
    
    if [[ ${#loaded_modules[@]} -eq 0 ]]; then
        print_warning "上次选择记录为空"
        return 1
    fi
    
    SELECTED_MODULES=("${loaded_modules[@]}")
    print_status "已加载上次选择的 ${#loaded_modules[@]} 个模块: ${loaded_modules[*]}"
    return 0
}

# 显示简短帮助提示
show_help() {
    echo "使用 $SCRIPT_NAME --help 查看完整帮助信息"
}

# 解析命令行参数
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -y|--yes)
                AUTO_YES=true
                shift
                ;;
            -j|--jobs)
                if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
                    BUILD_JOBS="$2"
                    shift 2
                else
                    print_error "选项 $1 需要一个数字参数"
                    exit 1
                fi
                ;;
            -l|--list)
                list_modules
                exit 0
                ;;
            -g|--groups)
                list_groups
                exit 0
                ;;
            --clean)
                clean_build_dirs
                exit 0
                ;;
            --clean-bs)
                clean_buildscripts_dirs
                exit 0
                ;;
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            -s|--same)
                if load_last_selection; then
                    print_status "已加载上次选择的模块"
                else
                    print_error "无法加载上次选择，请手动指定模块或先进行一次构建"
                    exit 1
                fi
                shift
                ;;
            --from-tui)
                FROM_TUI=true
                AUTO_YES=true  # 来自TUI时自动确认
                shift
                ;;
            -*)
                print_error "未知选项: $1"
                echo "使用 $SCRIPT_NAME --help 查看帮助"
                exit 1
                ;;
            *)
                SELECTED_MODULES+=("$1")
                shift
                ;;
        esac
    done

    if [[ $BUILD_JOBS -eq 0 ]]; then
        BUILD_JOBS=$(($(get_cpu_cores) - 1))
        if [[ $BUILD_JOBS -lt 1 ]]; then
            BUILD_JOBS=1
        fi
    fi

    if [[ ${#SELECTED_MODULES[@]} -eq 0 && "$FROM_TUI" != true ]]; then
        show_module_selection_prompt
    elif [[ "$FROM_TUI" == true && ${#SELECTED_MODULES[@]} -gt 0 ]]; then
        # Linus-style: 来自TUI时立即清理鼠标控制字符
        filter_mouse_control_chars
        print_status "接收来自TUI的模块选择: ${SELECTED_MODULES[*]}"
    fi
}

# 模块选择提示
show_module_selection_prompt() {
    echo "${CYAN}Sunray 构建系统${NC}"
    echo
    echo "${YELLOW}请指定要构建的模块:${NC}"
    echo
    echo "${GREEN}快速选择:${NC}"
    echo "  all     - 构建所有模块"
    echo "  groupA  - 构建模块组A"
    echo "  groupB  - 构建模块组B" 
    echo "  groupC  - 构建模块组C"
    echo
    echo "${GREEN}示例:${NC}"
    echo "  $SCRIPT_NAME all"
    echo "  $SCRIPT_NAME groupA groupB"
    echo "  $SCRIPT_NAME moduleA moduleB"
    echo "  $SCRIPT_NAME --list          # 查看所有可用选项"
    echo "  $SCRIPT_NAME --help          # 查看完整帮助"
    echo
    exit 1
}

# 列出所有可用模块
list_modules() {
    print_subtitle "可用模块列表"
    echo
    
    local modules=($(get_all_modules))
    
    for module in "${modules[@]}"; do
        local description=$(get_module_description "$module")
        print_module "$module" "$description"
    done
    echo
}

# 列出所有模块组
list_groups() {
    print_subtitle "可用模块组"
    echo
    
    local groups=($(get_all_groups))
    for group in "${groups[@]}"; do
        local description=$(get_group_description "$group")
        local group_modules=($(get_group_modules "$group"))
        
        print_group "$group" "$description"
        
        for module in "${group_modules[@]}"; do
            echo "    ${BRIGHT_WHITE}$module${NC}"
        done
        echo
        echo
    done
}

# 显示构建计划 - 直接使用old-cli逻辑，添加--from-tui支持，修复换行符问题
show_build_plan() {
    local modules_to_build=("$@")
    
    if [[ ${#modules_to_build[@]} -eq 0 ]]; then
        print_error "没有要构建的模块"
        return 1
    fi
    
    # 清理模块名称中的换行符和其他特殊字符
    local cleaned_modules=()
    for module in "${modules_to_build[@]}"; do
        # 更安全的清理方式：只移除尾部的\n
        local clean_module="${module%\\n}"
        clean_module="${clean_module%\\r}"  
        clean_module="${clean_module## }"   # 移除前置空格
        clean_module="${clean_module%% }"   # 移除后置空格
        [[ -n "$clean_module" ]] && cleaned_modules+=("$clean_module")
    done
    
    local resolved_modules=($(resolve_dependencies "${cleaned_modules[@]}"))
    local build_order=($(get_build_order "${resolved_modules[@]}"))
    
    echo
    echo "${CYAN}=== 构建计划 ===${NC}"
    echo
    echo "${YELLOW}构建配置:${NC}"
    echo "  并行任务数: $BUILD_JOBS"
    echo "  构建模式: $(if [[ "$DRY_RUN" == true ]]; then echo "预览模式"; else echo "执行模式"; fi)"
    echo
    
    echo "${YELLOW}模块构建顺序:${NC}"
    local index=1
    for module in "${build_order[@]}"; do
        local description=$(get_module_description "$module")
        
        printf "%2d. %-30s %s\n" $index "$module" "$description"
        ((index++))
    done
    
    echo
    echo "${YELLOW}总计:${NC} ${#build_order[@]} 个模块"
    
    local conflicts=($(check_module_conflicts "${resolved_modules[@]}"))
    if [[ ${#conflicts[@]} -gt 0 ]]; then
        HAD_CONFLICTS=true
        echo
        echo "${RED}⚠️  发现模块冲突:${NC}"
        for conflict in "${conflicts[@]}"; do
            echo "  - $conflict"
        done
        echo
        
        if ! resolve_module_conflicts conflicts "${resolved_modules[@]}"; then
            return 1
        fi
        
        resolved_modules=($(resolve_dependencies "${SELECTED_MODULES[@]}"))
        build_order=($(get_build_order "${resolved_modules[@]}"))
        
        echo
        echo "${YELLOW}冲突解决后的构建计划:${NC}"
        index=1
        for module in "${build_order[@]}"; do
            local description=$(get_module_description "$module")
            
            printf "%2d. %-30s %s\n" $index "$module" "$description"
            ((index++))
        done
        echo
        echo "${YELLOW}总计:${NC} ${#build_order[@]} 个模块"
    fi
    
    echo
    return 0
}

resolve_module_conflicts() {
    local conflicts_var="$1"; shift; local modules=("$@")
    
    echo "${CYAN}如何处理模块冲突？${NC}"
    echo "  [${BRIGHT_GREEN}m${NC}] 手动选择要构建的模块"
    echo "  [${BRIGHT_YELLOW}a${NC}] 自动移除冲突模块（保留第一个）"
    echo "  [${BRIGHT_RED}n${NC}] 取消构建"
    echo
    
    local attempts=0
    while [[ $attempts -lt 5 ]]; do
        read -p "请选择处理方式 [m/a/n]: " choice || {
            echo "输入结束，取消构建"
            return 1
        }
        
        case "$(echo "$choice" | tr '[:upper:]' '[:lower:]')" in
            m|manual)
                handle_manual_conflict_resolution "${modules[@]}"
                return $?
                ;;
            a|auto)
                handle_auto_conflict_resolution "${modules[@]}"
                return $?
                ;;
            n|no|cancel|"")
                echo "取消构建"
                return 1
                ;;
            *)
                echo "请输入 m（手动）、a（自动）或 n（取消）"
                ((attempts++))
                ;;
        esac
    done
    
    echo "输入尝试次数过多，取消构建"
    return 1
}

handle_manual_conflict_resolution() {
    local modules=("$@")
    echo
    echo "${YELLOW}请选择要构建的模块:${NC}"
    echo
    
    # 找出所有冲突的模块
    local conflicted_modules=()
    for module in "${modules[@]}"; do
        local conflicts_var="CONFIG_modules_${module}_conflicts_with"
        eval "local conflicts_str=\"\$${conflicts_var}\""
        
        if [ -n "$conflicts_str" ]; then
            local conflicting_modules
            # bash/zsh兼容的数组处理
            local saved_ifs="$IFS"
            IFS=','
            conflicting_modules=(${conflicts_str//[[\]\"]/})
            IFS="$saved_ifs"
            
            for conflict_module in "${conflicting_modules[@]}"; do
                conflict_module=$(echo "$conflict_module" | xargs)
                
                # 检查冲突模块是否也在构建列表中
                for check_module in "${modules[@]}"; do
                    if [ "$check_module" = "$conflict_module" ]; then
                        if [[ ! " ${conflicted_modules[@]} " =~ " ${module} " ]]; then
                            conflicted_modules+=("$module")
                        fi
                        if [[ ! " ${conflicted_modules[@]} " =~ " ${conflict_module} " ]]; then
                            conflicted_modules+=("$conflict_module")
                        fi
                    fi
                done
            done
        fi
    done
    
    # 智能冲突解决：让用户逐个选择是否保留冲突的模块，已保留的模块会自动排除其冲突项
    local updated_modules=()
    local excluded_modules=()
    
    for module in "${modules[@]}"; do
        # 如果模块已经被排除，跳过
        if [[ " ${excluded_modules[@]} " =~ " ${module} " ]]; then
            continue
        fi
        
        if [[ " ${conflicted_modules[@]} " =~ " ${module} " ]]; then
            local description=$(get_module_description "$module")
            if confirm "是否保留 ${BRIGHT_WHITE}$module${NC} ($description)？" "n"; then
                updated_modules+=("$module")
                
                # 自动排除与此模块冲突的其他模块
                local conflicts_var="CONFIG_modules_${module}_conflicts_with"
                eval "local conflicts_str=\"\$${conflicts_var}\""
                
                if [ -n "$conflicts_str" ]; then
                    local conflicting_modules
                    local saved_ifs="$IFS"
                    IFS=','
                    conflicting_modules=(${conflicts_str//[[\]\"]/})
                    IFS="$saved_ifs"
                    
                    for conflict_module in "${conflicting_modules[@]}"; do
                        conflict_module=$(echo "$conflict_module" | xargs)
                        
                        # 将冲突模块添加到排除列表
                        if [[ ! " ${excluded_modules[@]} " =~ " ${conflict_module} " ]]; then
                            excluded_modules+=("$conflict_module")
                            echo "  → 自动排除冲突模块: ${BRIGHT_RED}$conflict_module${NC}"
                        fi
                    done
                fi
            fi
        else
            # 非冲突模块直接保留
            updated_modules+=("$module")
        fi
    done
    
    SELECTED_MODULES=("${updated_modules[@]}")
    
    echo
    echo "${GREEN}已更新模块列表，冲突已智能解决${NC}"
    return 0
}

handle_auto_conflict_resolution() {
    local modules=("$@"); local updated_modules=(); local removed_modules=()
    
    for module in "${modules[@]}"; do
        local should_keep=true
        local conflicts_var="CONFIG_modules_${module}_conflicts_with"
        eval "local conflicts_str=\"\$${conflicts_var}\""
        
        if [ -n "$conflicts_str" ]; then
            local conflicting_modules; IFS=',' read -ra conflicting_modules <<< "${conflicts_str//[\[\]\"]/}"
            for conflict_module in "${conflicting_modules[@]}"; do
                conflict_module=$(echo "$conflict_module" | xargs)
                for kept_module in "${updated_modules[@]}"; do
                    if [ "$kept_module" = "$conflict_module" ]; then
                        should_keep=false; removed_modules+=("$module"); break 2
                    fi
                done
            done
        fi
        [ "$should_keep" = true ] && updated_modules+=("$module")
    done
    
    SELECTED_MODULES=("${updated_modules[@]}")
    
    if [[ ${#removed_modules[@]} -gt 0 ]]; then
        echo -e "\\n${YELLOW}自动移除的冲突模块:${NC}"
        for module in "${removed_modules[@]}"; do echo "  - $module"; done
    fi
    
    echo -e "\\n${GREEN}冲突已自动解决${NC}"; return 0
}

confirm() {
    local message="$1"; local default="${2:-n}"
    
    if [[ "$default" == "y" ]]; then
        read -p "$message [Y/n]: " answer
    else
        read -p "$message [y/N]: " answer
    fi
    
    [[ -z "$answer" ]] && answer="$default"
    
    case "$(echo "$answer" | tr '[:upper:]' '[:lower:]')" in
        y|yes) return 0 ;;
        *) return 1 ;;
    esac
}

clean_build_dirs() {
    echo "${CYAN}=== Sunray Build System Cleanup Tool ===${NC}"
    echo
    
    local build_dirs=("build" "devel" ".catkin_workspace" "logs")
    local total_cleaned=0
    
    for dir in "${build_dirs[@]}"; do
        if [[ -d "$dir" ]]; then
            local size=$(du -sh "$dir" 2>/dev/null | cut -f1 || echo "unknown")
            echo "Found build directory: $dir (size: $size)"
            rm -rf "$dir" && { 
                print_success "✓ Removed $dir"; 
                ((total_cleaned++)); 
            } || print_error "✗ Failed to remove $dir"
        fi
    done
    
    # Clean compilation artifacts
    local patterns=("*.o" "*.a" "*.so" "*.dylib" "CMakeCache.txt" "cmake_install.cmake" "Makefile")
    for pattern in "${patterns[@]}"; do
        local files=$(find . -name "$pattern" -type f 2>/dev/null | head -10)
        if [[ -n "$files" ]]; then
            echo "Found $pattern files"
            find . -name "$pattern" -type f -delete 2>/dev/null
            print_success "✓ Removed $pattern files"
            ((total_cleaned++))
        fi
    done
    
    echo -e "\\n${GREEN}Cleanup completed, processed $total_cleaned items${NC}"
}

clean_buildscripts_dirs() {
    echo "${CYAN}=== Buildscripts Cleanup Tool ===${NC}"
    echo
    
    local bs_dirs=("buildscripts/tui/build" "buildscripts/bin" "buildscripts/tui/.cache")
    local total_cleaned=0
    
    for dir in "${bs_dirs[@]}"; do
        if [[ -d "$dir" ]]; then
            local size=$(du -sh "$dir" 2>/dev/null | cut -f1 || echo "unknown")
            echo "Found buildscripts directory: $dir (size: $size)"
            rm -rf "$dir" && { 
                print_success "✓ Removed $dir"; 
                ((total_cleaned++)); 
            } || print_error "✗ Failed to remove $dir"
        fi
    done
    
    # Clean buildscripts compilation artifacts
    local patterns=("buildscripts/**/*.o" "buildscripts/**/CMakeCache.txt" "buildscripts/**/cmake_install.cmake" "buildscripts/**/Makefile")
    for pattern in "${patterns[@]}"; do
        local files=$(find buildscripts -name "$(basename "$pattern")" -type f 2>/dev/null | head -10)
        if [[ -n "$files" ]]; then
            echo "Found $(basename "$pattern") files in buildscripts"
            find buildscripts -name "$(basename "$pattern")" -type f -delete 2>/dev/null
            print_success "✓ Removed $(basename "$pattern") files from buildscripts"
            ((total_cleaned++))
        fi
    done
    
    echo -e "\\n${GREEN}Buildscripts cleanup completed, processed $total_cleaned items${NC}"
}


run_ui_flow() {
    [[ "$FROM_TUI" != true ]] && echo -e "${CYAN}Sunray 构建系统${NC}\\n"
    
    if ! validate_config; then
        print_error "配置验证失败"; return 1
    fi
    
    if [[ ${#SELECTED_MODULES[@]} -eq 0 ]]; then
        print_error "没有指定要构建的模块"; show_help; return 1
    fi
    
    local expanded_modules=()
    for module in "${SELECTED_MODULES[@]}"; do
        case "$module" in
            all) 
                # 使用while循环正确处理换行分隔的模块列表
                while IFS= read -r all_module; do
                    [[ -n "$all_module" ]] && expanded_modules+=("$all_module")
                done <<< "$(get_all_modules)"
                ;;
            *)
                local is_module=$(module_exists "$module" && echo "true" || echo "false")
                local is_group=$(get_group_modules "$module" >/dev/null 2>&1 && echo "true" || echo "false")
                
                if [[ "$is_module" == "true" && "$is_group" == "true" ]]; then
                    HAD_CONFLICTS=true
                    echo -e "\\n${YELLOW}⚠️  发现命名冲突: '$module'${NC}\\n  ${CYAN}模块${NC}: $(get_module_description "$module")\\n  ${CYAN}构建组${NC}: $(get_group_description "$module")\\n"
                    if confirm "是否构建整个 '${BRIGHT_WHITE}$module${NC}' 构建组？" "y"; then
                        while IFS= read -r group_module; do
                            [[ -n "$group_module" ]] && expanded_modules+=("$group_module")
                        done <<< "$(get_group_modules "$module")"
                    else
                        expanded_modules+=("$module")
                    fi
                elif [[ "$is_module" == "true" ]]; then
                    expanded_modules+=("$module")
                elif [[ "$is_group" == "true" ]]; then
                    while IFS= read -r group_module; do
                        [[ -n "$group_module" ]] && expanded_modules+=("$group_module")
                    done <<< "$(get_group_modules "$module")"
                else
                    print_warning "未知模块: $module"
                fi ;;
        esac
    done
    
    # 去重和排序，但保持数组结构
    local temp_array=()
    while IFS= read -r sorted_module; do
        [[ -n "$sorted_module" ]] && temp_array+=("$sorted_module")
    done <<< "$(printf '%s\n' "${expanded_modules[@]}" | sort -u)"
    SELECTED_MODULES=("${temp_array[@]}")
    
    if ! show_build_plan "${SELECTED_MODULES[@]}"; then return 1; fi
    
    [[ "$DRY_RUN" == true ]] && { print_status "预览模式 - 不执行实际构建"; return 0; }
    
    if [[ "$AUTO_YES" == true && "$HAD_CONFLICTS" == false ]]; then
        echo "自动确认开始构建..."
    else
        if ! confirm "开始构建？" "y"; then
            echo "取消构建"; return 2
        fi
    fi
    
    # 保存当前选择到隐藏文件
    if [[ ${#SELECTED_MODULES[@]} -gt 0 ]]; then
        save_module_selection "${SELECTED_MODULES[@]}"
    fi
    
    return 0
}