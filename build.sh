#!/bin/bash
# Sunray 模块化构建系统
# ./build.sh --help # 运行构建脚本

set -e

# 目录设置
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$SCRIPT_DIR"
readonly BUILDSCRIPTS_DIR="$SCRIPT_DIR/buildscripts"

[[ ! -d "$BUILDSCRIPTS_DIR" ]] && {
    echo "❌ 模块化构建系统未找到: $BUILDSCRIPTS_DIR"
    echo "请确保运行了构建系统初始化"
    exit 1
}

# 模块加载
source "$BUILDSCRIPTS_DIR/lib/utils.sh"
source "$BUILDSCRIPTS_DIR/lib/config.sh"
source "$BUILDSCRIPTS_DIR/lib/ui.sh"
source "$BUILDSCRIPTS_DIR/lib/builder.sh"

# 主函数
main() {
    local start_time=$(date +%s)
    
    init_config && parse_arguments "$@" || exit 1
    
    local ui_result
    run_ui_flow; ui_result=$?
    
    case $ui_result in
    esac
    
    [[ "$DRY_RUN" == true ]] && exit 0
    
    print_status "初始化构建环境..."
    init_build_environment "$WORKSPACE_ROOT" || { print_error "构建环境初始化失败"; exit 1; }
    
    print_status "解析模块依赖关系..."
    local resolved_modules=($(resolve_dependencies "${SELECTED_MODULES[@]}"))
    [[ ${#resolved_modules[@]} -eq 0 ]] && { print_error "没有找到要构建的模块"; exit 1; }
    
    echo
    echo "${CYAN}=== 开始构建 ===${NC}"
    echo "构建模块: ${resolved_modules[*]}"
    echo "并行任务: $BUILD_JOBS"
    echo
    
    trap cleanup_build_environment EXIT
    
    if build_modules_parallel "${resolved_modules[@]}"; then
        local total_time=$(($(date +%s) - start_time))
        echo
        echo "${GREEN}🎉 构建完成！${NC}"
        echo "总用时: $(format_duration $total_time)"
        post_build_actions
        return 0
    else
        echo
        echo "${RED}❌ 构建失败！${NC}"
        return 1
    fi
}

# 构建后处理
post_build_actions() {
    print_status "执行构建后处理..."
    
    # ROS工作空间检查
    [[ -f "devel/setup.bash" ]] && {
        print_status "ROS工作空间设置文件已生成: devel/setup.bash"
        echo "使用以下命令设置环境:"
        echo "  ${CYAN}source devel/setup.bash${NC}"
    }
    
    # 快速磁盘空间检查
    local available_gb=$(($(df "$WORKSPACE_ROOT" | awk 'NR==2 {print $4}') / 1024 / 1024))
    [[ $available_gb -lt 1 ]] && {
        print_warning "磁盘空间不足 (剩余 ${available_gb}GB)，建议清理构建缓存"
        print_status "使用以下命令清理: $0 --clean"
    }
}

# 兼容性函数
handle_legacy_arguments() {
    local legacy_args=()
    
    # 检查是否使用了旧的参数格式
    for arg in "$@"; do
        case "$arg" in
            # 旧的模块组名称映射
            "uav_modules"|"UAV"|"uav")
                legacy_args+=("uav")
                ;;
            "ugv_modules"|"UGV"|"ugv")
                legacy_args+=("ugv")
                ;;
            "simulation_modules"|"SIM"|"sim")
                legacy_args+=("sim")
                ;;
            "common_modules"|"common")
                legacy_args+=("common")
                ;;
            "all_modules"|"ALL")
                legacy_args+=("all")
                ;;
            # 保持其他参数不变
            *)
                legacy_args+=("$arg")
                ;;
        esac
    done
    
    # 如果参数发生了变化，显示提示信息
    if [[ "${legacy_args[*]}" != "$*" ]]; then
        print_warning "检测到旧式参数格式，已自动转换"
        print_status "新的参数格式: ${legacy_args[*]}"
    fi
    
    echo "${legacy_args[@]}"
}

# 显示迁移帮助
show_migration_help() {
    cat << EOF
${YELLOW}=== 构建系统迁移指南 ===${NC}

${CYAN}新的构建系统特性:${NC}
• 🔧 模块化配置文件管理
• 🚀 智能依赖解析
• 🎯 交互式模块选择
• 📊 详细的构建报告
• 🔄 并行构建优化

${CYAN}主要变化:${NC}
• 配置文件驱动的构建定义
• 分离的UI交互逻辑
• 改进的错误处理和日志
• 更好的用户体验

${CYAN}旧命令对应关系:${NC}
  旧: ./build.sh uav_modules
  新: ./build.sh uav

  旧: ./build.sh --clean --verbose
  新: ./build.sh --clean -v

${CYAN}交互模式:${NC}
  ./build.sh -i    # 启动交互式界面
  ./build.sh       # 默认也会进入交互模式

${CYAN}获取帮助:${NC}
  ./build.sh --help        # 查看完整帮助
  ./build.sh --list        # 列出所有可用模块
  ./build.sh --profiles    # 列出构建配置文件

EOF
}

# 错误处理
handle_error() {
    local exit_code=$? line_number=$1
    
    echo
    print_error "构建脚本在第 $line_number 行发生错误 (退出码: $exit_code)"
    
    case $exit_code in
        127) 
            print_error "可能是缺少必要的依赖或模块未找到" 
            ;;
        130) 
            print_warning "构建被用户中断" 
            ;;
        *) 
            print_error "构建失败，请查看上方错误信息" 
            ;;
    esac
    
    cleanup_build_environment
    exit $exit_code
}

trap 'handle_error $LINENO' ERR

# 版本信息显示
show_version() {
    echo "Sunray 构建系统"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-}" in
        --version|-V) show_version; exit 0 ;;
        --migration|--migrate) show_migration_help; exit 0 ;;
        --help|-h) show_help; exit 0 ;;
    esac
    
    main $(handle_legacy_arguments "$@")
fi