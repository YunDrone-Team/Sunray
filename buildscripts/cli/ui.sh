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
VERBOSE=false
DRY_RUN=false
AUTO_YES=false
FROM_TUI=false
HAD_CONFLICTS=false
BUILD_JOBS=0
SELECTED_MODULES=()

# 显示帮助信息
show_help() {
    print_header "Sunray 构建系统"
    
    print_help_section "用法"
    echo -e "  ${BRIGHT_WHITE}$SCRIPT_NAME${NC} ${BRIGHT_CYAN}[选项]${NC} ${BRIGHT_YELLOW}[模块...]${NC}"
    echo
    
    print_help_section "选项"
    print_config "-h, --help" "显示此帮助信息"
    print_config "-v, --verbose" "详细输出模式"
    print_config "-y, --yes" "自动确认开始构建（仅在无冲突时生效）"
    print_config "-j, --jobs N" "并行构建任务数 (默认: CPU核心数-1)"
    print_config "-l, --list" "列出所有可用模块"
    print_config "-g, --groups" "列出所有模块组"
    print_config "    --clean" "清理构建目录"
    print_config "    --dry-run" "显示将要执行的构建操作但不执行"
    echo
    
    print_help_section "模块选择"
    print_config "[模块名]" "构建指定模块"
    
    if ! load_modules_config > /dev/null 2>&1; then
        print_config "all" "构建所有模块"
        print_config "control" "控制系统构建"
        print_config "ego" "EGO规划器构建"
        print_config "sim" "仿真环境构建"
        print_config "ugv_control" "地面车控制构建"
    else
        local groups=($(get_all_groups))
        for group in "${groups[@]}"; do
            local description=$(get_group_description "$group")
            print_config "$group" "$description"
        done
    fi
    echo

    print_help_section "示例"
    print_example "$SCRIPT_NAME all" "构建所有模块"
    print_example "$SCRIPT_NAME control" "构建控制系统相关模块"
    print_example "$SCRIPT_NAME ego" "构建EGO规划器相关模块"
    print_example "$SCRIPT_NAME all -j 8" "使用8个并行任务构建所有模块"
    print_example "$SCRIPT_NAME all -y" "构建所有模块并自动确认开始"
    print_example "$SCRIPT_NAME --list" "列出所有可用模块"
    print_example "$SCRIPT_NAME --dry-run sim" "预览仿真环境构建计划"
    echo
}

# 解析命令行参数
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
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
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            --from-tui)
                FROM_TUI=true
                AUTO_YES=true  # 来自TUI时自动确认
                VERBOSE=false  # 减少冗余输出
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
        print_status "接收来自TUI的模块选择: ${SELECTED_MODULES[*]}"
    fi
}

# 模块选择提示
show_module_selection_prompt() {
    echo "${CYAN}Sunray 模块化构建系统${NC}"
    echo
    echo "${YELLOW}请指定要构建的模块:${NC}"
    echo
    echo "${GREEN}快速选择:${NC}"
    echo "  all     - 构建所有模块"
    echo "  uav     - 构建UAV相关模块"
    echo "  ugv     - 构建UGV相关模块"
    echo "  sim     - 构建仿真相关模块"
    echo "  common  - 构建通用模块"
    echo
    echo "${GREEN}示例:${NC}"
    echo "  $SCRIPT_NAME all"
    echo "  $SCRIPT_NAME uav sim"
    echo "  $SCRIPT_NAME --list          # 查看所有可用模块"
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

# 显示构建计划
show_build_plan() {
    local modules_to_build=("$@")
    
    if [[ ${#modules_to_build[@]} -eq 0 ]]; then
        print_error "没有要构建的模块"
        return 1
    fi
    
    local resolved_modules=($(resolve_dependencies "${modules_to_build[@]}"))
    local build_order=($(get_build_order "${resolved_modules[@]}"))
    
    echo
    echo "${CYAN}=== 构建计划 ===${NC}"
    echo
    echo "${YELLOW}构建配置:${NC}"
    echo "  并行任务数: $BUILD_JOBS"
    echo "  构建模式: $(if [[ "$DRY_RUN" == true ]]; then echo "预览模式"; else echo "执行模式"; fi)"
    echo
    
    echo "${YELLOW}模块构建顺序:${NC}"
    for i in "${!build_order[@]}"; do
        local module="${build_order[i]}"
        local description=$(get_module_description "$module")
        
        printf "%2d. %-30s %s\n" $((i+1)) "$module" "$description"
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
        for i in "${!build_order[@]}"; do
            local module="${build_order[i]}"
            local description=$(get_module_description "$module")
            
            printf "%2d. %-30s %s\n" $((i+1)) "$module" "$description"
        done
        echo
        echo "${YELLOW}总计:${NC} ${#build_order[@]} 个模块"
    fi
    
    echo
    return 0
}

# 冲突解决交互
resolve_module_conflicts() {
    local conflicts_var="$1"
    shift
    local modules=("$@")
    
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

# 手动解决冲突
handle_manual_conflict_resolution() {
    local modules=("$@")
    echo
    echo "${YELLOW}请选择要构建的模块:${NC}"
    echo
    
    local conflicted_modules=()
    for module in "${modules[@]}"; do
        local conflicts_var="CONFIG_modules_${module}_conflicts_with"
        local conflicts_str="${!conflicts_var}"
        
        if [ -n "$conflicts_str" ]; then
            local conflicting_modules
            IFS=',' read -ra conflicting_modules <<< "${conflicts_str//[\[\]\"]/}"
            
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
    
    local updated_modules=()
    for module in "${modules[@]}"; do
        if [[ " ${conflicted_modules[@]} " =~ " ${module} " ]]; then
            local description=$(get_module_description "$module")
            if confirm "是否保留 ${BRIGHT_WHITE}$module${NC} ($description)？" "n"; then
                updated_modules+=("$module")
            fi
        else
            updated_modules+=("$module")
        fi
    done
    
    # 检查是否还有剩余的冲突
    local remaining_conflicts=($(check_module_conflicts "${updated_modules[@]}"))
    if [[ ${#remaining_conflicts[@]} -gt 0 ]]; then
        echo
        echo "${RED}仍然存在冲突，请重新选择:${NC}"
        for conflict in "${remaining_conflicts[@]}"; do
            echo "  - $conflict"
        done
        return 1
    fi
    
    SELECTED_MODULES=("${updated_modules[@]}")
    
    echo
    echo "${GREEN}已更新模块列表，冲突已解决${NC}"
    return 0
}

# 自动解决冲突（保留第一个遇到的模块）
handle_auto_conflict_resolution() {
    local modules=("$@")
    local updated_modules=()
    local removed_modules=()
    
    for module in "${modules[@]}"; do
        local should_keep=true
        
        # 检查这个模块是否与已经保留的模块冲突
        local conflicts_var="CONFIG_modules_${module}_conflicts_with"
        local conflicts_str="${!conflicts_var}"
        
        if [ -n "$conflicts_str" ]; then
            local conflicting_modules
            IFS=',' read -ra conflicting_modules <<< "${conflicts_str//[\[\]\"]/}"
            
            for conflict_module in "${conflicting_modules[@]}"; do
                conflict_module=$(echo "$conflict_module" | xargs)
                
                # 检查冲突模块是否已经在保留列表中
                for kept_module in "${updated_modules[@]}"; do
                    if [ "$kept_module" = "$conflict_module" ]; then
                        should_keep=false
                        removed_modules+=("$module")
                        break 2
                    fi
                done
            done
        fi
        
        if [ "$should_keep" = true ]; then
            updated_modules+=("$module")
        fi
    done
    
    SELECTED_MODULES=("${updated_modules[@]}")
    
    if [[ ${#removed_modules[@]} -gt 0 ]]; then
        echo
        echo "${YELLOW}自动移除的冲突模块:${NC}"
        for module in "${removed_modules[@]}"; do
            echo "  - $module"
        done
    fi
    
    echo
    echo "${GREEN}冲突已自动解决${NC}"
    return 0
}

# 确认对话框 - 修复无限循环
confirm() {
    local message="$1"
    local default="${2:-n}"
    
    local attempts=0
    while [[ $attempts -lt 5 ]]; do
        if [[ "$default" == "y" ]]; then
            read -p "$message [Y/n]: " answer || {
                echo "输入结束，使用默认值: $default"
                answer="$default"
            }
        else
            read -p "$message [y/N]: " answer || {
                echo "输入结束，使用默认值: $default"
                answer="$default"
            }
        fi
        
        if [[ -z "$answer" ]]; then
            answer="$default"
        fi
        
        case "$(echo "$answer" | tr '[:upper:]' '[:lower:]')" in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            *) 
                echo "请输入 y 或 n"
                ((attempts++))
                ;;
        esac
    done
    
    echo "输入尝试次数过多，使用默认值: $default"
    [[ "$default" == "y" ]] && return 0 || return 1
}

# 清理构建目录
clean_build_dirs() {
    echo "${CYAN}清理构建目录...${NC}"
    
    local build_dirs=(
        "build"
        "devel"
        ".catkin_workspace"
    )
    
    for dir in "${build_dirs[@]}"; do
        if [[ -d "$dir" ]]; then
            if confirm "删除目录 $dir？"; then
                rm -rf "$dir"
                print_success "已删除 $dir"
            fi
        fi
    done
    
    echo "构建目录清理完成"
}


# 主UI流程
run_ui_flow() {
    if [[ "$FROM_TUI" != true ]]; then
        echo "${CYAN}Sunray 构建系统${NC}"
        echo
    fi
    
    # 检查配置文件
    if ! validate_config; then
        print_error "配置验证失败"
        return 1
    fi
    
    if [[ ${#SELECTED_MODULES[@]} -eq 0 ]]; then
        print_error "没有指定要构建的模块"
        show_help
        return 1
    fi
    
    local expanded_modules=()
    for module in "${SELECTED_MODULES[@]}"; do
        case "$module" in
            all)
                expanded_modules+=($(get_all_modules))
                ;;
            *)
                local is_module=$(module_exists "$module" && echo "true" || echo "false")
                local is_group=$(get_group_modules "$module" >/dev/null 2>&1 && echo "true" || echo "false")
                
                if [[ "$is_module" == "true" && "$is_group" == "true" ]]; then
                    # 发现命名冲突，询问用户意图
                    HAD_CONFLICTS=true
                    echo
                    echo "${YELLOW}⚠️  发现命名冲突: '$module'${NC}"
                    echo "  存在同名的模块和构建组："
                    echo "  ${CYAN}模块${NC}: $(get_module_description "$module")"
                    echo "  ${CYAN}构建组${NC}: $(get_group_description "$module")"
                    echo
                    
                    if confirm "是否构建整个 '${BRIGHT_WHITE}$module${NC}' 构建组？" "y"; then
                        expanded_modules+=($(get_group_modules "$module"))
                    else
                        expanded_modules+=("$module")
                    fi
                elif [[ "$is_module" == "true" ]]; then
                    expanded_modules+=("$module")
                elif [[ "$is_group" == "true" ]]; then
                    expanded_modules+=($(get_group_modules "$module"))
                else
                    print_warning "未知模块: $module"
                fi
                ;;
        esac
    done
    
    SELECTED_MODULES=($(printf '%s\n' "${expanded_modules[@]}" | sort -u))
    
    if ! show_build_plan "${SELECTED_MODULES[@]}"; then
        return 1
    fi
    
    if [[ "$DRY_RUN" == true ]]; then
        print_status "预览模式 - 不执行实际构建"
        return 0
    fi
    
    if [[ "$AUTO_YES" == true && "$HAD_CONFLICTS" == false ]]; then
        echo "自动确认开始构建..."
    else
        if ! confirm "开始构建？" "y"; then
            echo "取消构建"
            return 2  # 返回特殊退出码表示用户取消
        fi
    fi
    
    return 0
}

# 获取UI状态
get_ui_config() {
    cat << EOF
{
    "verbose": $VERBOSE,
    "auto_yes": $AUTO_YES,
    "build_jobs": $BUILD_JOBS,
    "selected_modules": [$(printf '"%s",' "${SELECTED_MODULES[@]}" | sed 's/,$//')],
    "dry_run": $DRY_RUN
}
EOF
}