#!/bin/bash

# Sunray 模块化构建系统 - 主构建脚本
# 基于配置驱动的模块化构建架构
# 作者: 重构自原始构建脚本
# 用法: ./build.sh [选项] [模块...]

set -e  # 任意命令出错则退出

# 脚本目录和根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

# 检查并加载模块化构建系统
BUILDSCRIPTS_DIR="$SCRIPT_DIR/buildscripts"

if [[ ! -d "$BUILDSCRIPTS_DIR" ]]; then
    echo "❌ 模块化构建系统未找到: $BUILDSCRIPTS_DIR"
    echo "请确保运行了构建系统初始化"
    exit 1
fi

# 加载构建系统模块
source "$BUILDSCRIPTS_DIR/lib/utils.sh"
source "$BUILDSCRIPTS_DIR/lib/config.sh"
source "$BUILDSCRIPTS_DIR/lib/ui.sh"
source "$BUILDSCRIPTS_DIR/lib/builder.sh"

# 主函数
main() {
    local start_time=$(date +%s)
    
    # 初始化配置系统
    if ! init_config; then
        print_error "配置系统初始化失败"
        exit 1
    fi
    
    # 解析命令行参数
    parse_arguments "$@"
    
    # 运行UI流程
    if ! run_ui_flow; then
        exit 1
    fi
    
    # 如果是预览模式，直接退出
    if [[ "$DRY_RUN" == true ]]; then
        exit 0
    fi
    
    # 初始化构建环境
    print_status "初始化构建环境..."
    if ! init_build_environment "$WORKSPACE_ROOT"; then
        print_error "构建环境初始化失败"
        exit 1
    fi
    
    # 解析要构建的模块
    print_status "解析模块依赖关系..."
    local resolved_modules=($(resolve_dependencies "${SELECTED_MODULES[@]}"))
    if [[ ${#resolved_modules[@]} -eq 0 ]]; then
        print_error "没有找到要构建的模块"
        exit 1
    fi
    
    # 显示最终的构建计划
    echo
    echo "${CYAN}=== 开始构建 ===${NC}"
    echo "构建模块: ${resolved_modules[*]}"
    echo "并行任务: $BUILD_JOBS"
    echo "配置文件: 默认"
    echo

    # 执行构建
    local build_result=0
    
    # 注册清理函数
    trap cleanup_build_environment EXIT
    
    if build_modules_parallel "${resolved_modules[@]}"; then
        local end_time=$(date +%s)
        local total_time=$((end_time - start_time))
        
        echo
        echo "${GREEN}🎉 构建完成！${NC}"
        echo "总用时: $(format_duration $total_time)"
        
        # 构建后处理
        post_build_actions
        
        build_result=0
    else
        echo
        echo "${RED}❌ 构建失败！${NC}"
        build_result=1
    fi
    
    return $build_result
}

# 构建后处理
post_build_actions() {
    print_status "执行构建后处理..."
    
    # 更新ROS包路径
    if [[ -f "devel/setup.bash" ]]; then
        print_status "ROS工作空间设置文件已生成: devel/setup.bash"
        print_status "使用以下命令设置环境:"
        echo "  ${CYAN}source devel/setup.bash${NC}"
    fi
    
    # 检查磁盘空间
    check_disk_space
}

# 检查磁盘空间
check_disk_space() {
    local available_space=$(df "$WORKSPACE_ROOT" | awk 'NR==2 {print $4}')
    local available_gb=$((available_space / 1024 / 1024))
    
    if [[ $available_gb -lt 1 ]]; then
        print_warning "磁盘空间不足 (剩余 ${available_gb}GB)，建议清理构建缓存"
        print_status "使用以下命令清理: $0 --clean"
    fi
}

# 兼容性函数 - 处理旧式的调用方式
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
    local exit_code=$?
    local line_number=$1
    
    echo
    print_error "构建脚本在第 $line_number 行发生错误 (退出码: $exit_code)"
    
    if [[ $exit_code -eq 127 ]]; then
        print_error "可能是缺少必要的依赖或模块未找到"
        print_status "尝试运行: $0 --check-deps"
    elif [[ $exit_code -eq 130 ]]; then
        print_warning "构建被用户中断"
    else
        print_error "构建失败，请查看上方错误信息"
    fi
    
    # 清理构建环境
    cleanup_build_environment
    
    exit $exit_code
}

# 注册错误处理器
trap 'handle_error $LINENO' ERR

# 显示版本信息
show_version() {
    echo "Sunray 模块化构建系统 v2.0.0"
    echo "基于配置驱动的现代构建架构"
    echo "原始版本兼容，增强的功能和用户体验"
}

# 主入口点
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # 处理特殊的帮助选项
    case "${1:-}" in
        --version|-V)
            show_version
            exit 0
            ;;
        --migration|--migrate)
            show_migration_help
            exit 0
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
    esac
    
    # 处理兼容性参数
    processed_args=($(handle_legacy_arguments "$@"))
    
    # 运行主函数
    main "${processed_args[@]}"
fi