#!/bin/bash

# UI交互层 - 简化的用户界面和交互逻辑
# 提供命令行参数解析和简洁的用户提示

# 导入工具函数（如果没有被加载的话）
if ! declare -f print_status >/dev/null 2>&1; then
    source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
fi
if ! declare -f validate_config >/dev/null 2>&1; then
    source "$(dirname "${BASH_SOURCE[0]}")/config.sh"
fi

# 全局变量
SCRIPT_NAME="$(basename "$0")"
VERBOSE=false
QUIET=false
SELECTED_MODULES=()
BUILD_JOBS=0
DRY_RUN=false

# 显示帮助信息
show_help() {
    print_header "Sunray 模块化构建系统"
    
    print_help_section "用法"
    echo -e "  ${BRIGHT_WHITE}$SCRIPT_NAME${NC} ${BRIGHT_CYAN}[选项]${NC} ${BRIGHT_YELLOW}[模块...]${NC}"
    echo
    
    print_help_section "选项"
    print_config "-h, --help" "显示此帮助信息"
    print_config "-v, --verbose" "详细输出模式"
    print_config "-q, --quiet" "安静模式（最小输出）"
    print_config "-j, --jobs N" "并行构建任务数 (默认: CPU核心数-1)"
    print_config "-l, --list" "列出所有可用模块"
    print_config "-g, --groups" "列出所有模块组"
    print_config "    --clean" "清理构建目录"
    print_config "    --check-deps" "检查系统依赖项"
    print_config "    --dry-run" "显示将要执行的构建操作但不执行"
    echo
    
    print_help_section "模块选择"
    print_config "[模块名]" "构建指定模块"
    
    # 确保配置已加载并动态显示真实可用的组
    if ! load_modules_config > /dev/null 2>&1; then
        # 如果配置加载失败，显示默认组
        print_config "all" "构建所有模块"
        print_config "control" "控制系统构建"
        print_config "ego" "EGO规划器构建"
        print_config "sim" "仿真环境构建"
        print_config "ugv_control" "地面车控制构建"
    else
        # 配置加载成功，动态显示组
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
            -q|--quiet)
                QUIET=true
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
            --check-deps)
                check_system_dependencies
                exit 0
                ;;
            --dry-run)
                DRY_RUN=true
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

    # 设置默认值 (CPU核心数-1)
    if [[ $BUILD_JOBS -eq 0 ]]; then
        BUILD_JOBS=$(($(get_cpu_cores) - 1))
        if [[ $BUILD_JOBS -lt 1 ]]; then
            BUILD_JOBS=1
        fi
    fi

    # 如果没有指定模块，提示用户
    if [[ ${#SELECTED_MODULES[@]} -eq 0 ]]; then
        show_module_selection_prompt
    fi
}

# 简化的模块选择提示
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
        
        # 用缩进显示模块列表，每行一个模块
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
    
    # 解析依赖关系
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
    
    # 检查冲突
    local conflicts=($(check_module_conflicts "${resolved_modules[@]}"))
    if [[ ${#conflicts[@]} -gt 0 ]]; then
        echo
        echo "${RED}警告: 发现模块冲突:${NC}"
        for conflict in "${conflicts[@]}"; do
            echo "  - $conflict"
        done
        echo
        
        if ! confirm "是否继续构建？"; then
            return 1
        fi
    fi
    
    echo
    return 0
}

# 确认对话框
confirm() {
    local message="$1"
    local default="${2:-n}"
    
    while true; do
        if [[ "$default" == "y" ]]; then
            read -p "$message [Y/n]: " answer
            answer="${answer:-y}"
        else
            read -p "$message [y/N]: " answer
            answer="${answer:-n}"
        fi
        
        case "$(echo "$answer" | tr '[:upper:]' '[:lower:]')" in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            *) echo "请输入 y 或 n" ;;
        esac
    done
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

# 检查系统依赖项
check_system_dependencies() {
    echo "${CYAN}检查系统依赖项...${NC}"
    
    local required_commands=(
        "catkin_make:ROS构建工具"
        "roscore:ROS核心"
        "gcc:C编译器"
        "g++:C++编译器"
        "cmake:CMake构建系统"
        "make:Make构建工具"
    )
    
    local missing_deps=()
    
    for dep in "${required_commands[@]}"; do
        local cmd="${dep%%:*}"
        local desc="${dep##*:}"
        
        if command -v "$cmd" >/dev/null 2>&1; then
            echo "${GREEN}✓${NC} $desc ($cmd)"
        else
            echo "${RED}✗${NC} $desc ($cmd) - 缺失"
            missing_deps+=("$cmd")
        fi
    done
    
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        echo
        echo "${RED}缺失的依赖项:${NC}"
        for dep in "${missing_deps[@]}"; do
            echo "  - $dep"
        done
        echo
        echo "${YELLOW}请安装缺失的依赖项后再运行构建${NC}"
        return 1
    else
        echo
        echo "${GREEN}所有系统依赖项都已安装${NC}"
        return 0
    fi
}

# 主UI流程
run_ui_flow() {
    # 显示简洁的欢迎信息
    if [[ "$QUIET" != true ]]; then
        echo "${CYAN}Sunray 模块化构建系统 v2.0${NC}"
        echo
    fi
    
    # 检查配置文件
    if ! validate_config; then
        print_error "配置验证失败"
        return 1
    fi
    
    # 如果仍然没有选择模块，使用默认
    if [[ ${#SELECTED_MODULES[@]} -eq 0 ]]; then
        print_error "没有指定要构建的模块"
        show_help
        return 1
    fi
    
    # 展开模块组和别名
    local expanded_modules=()
    for module in "${SELECTED_MODULES[@]}"; do
        case "$module" in
            all)
                expanded_modules+=($(get_all_modules))
                ;;
            *)
                # 首先检查是否是个体模块
                if module_exists "$module"; then
                    expanded_modules+=("$module")
                # 然后检查是否是组（类别或虚拟组）
                elif get_group_modules "$module" >/dev/null 2>&1; then
                    expanded_modules+=($(get_group_modules "$module"))
                else
                    print_warning "未知模块: $module"
                fi
                ;;
        esac
    done
    
    SELECTED_MODULES=($(printf '%s\n' "${expanded_modules[@]}" | sort -u))
    
    # 显示构建计划
    if ! show_build_plan "${SELECTED_MODULES[@]}"; then
        return 1
    fi
    
    # 确认构建
    if [[ "$DRY_RUN" == true ]]; then
        print_status "预览模式 - 不执行实际构建"
        return 0
    fi
    
    if [[ "$QUIET" != true ]]; then
        if ! confirm "开始构建？" "y"; then
            echo "取消构建"
            return 0
        fi
    fi
    
    return 0
}

# 获取UI状态
get_ui_config() {
    cat << EOF
{
    "verbose": $VERBOSE,
    "quiet": $QUIET,
    "build_jobs": $BUILD_JOBS,
    "selected_modules": [$(printf '"%s",' "${SELECTED_MODULES[@]}" | sed 's/,$//')],
    "dry_run": $DRY_RUN
}
EOF
}