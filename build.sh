#!/bin/bash
# Sunray 模块化构建系统

set -e

# 目录设置
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$SCRIPT_DIR"
readonly BUILDSCRIPTS_DIR="$SCRIPT_DIR/buildscripts"

# 预处理命令行参数，决定界面模式
# 规则：只有无参数或明确指定--tui时才使用TUI界面
# 任何其他参数（包括-s/--same）都直接进入CLI模式
if [[ $# -eq 0 ]]; then
    INTERFACE_MODE="tui"
elif [[ $# -eq 1 && "$1" == "--tui" ]]; then
    INTERFACE_MODE="tui"
    shift  # 移除--tui参数
else
    INTERFACE_MODE="cli"
fi

[[ ! -d "$BUILDSCRIPTS_DIR" ]] && {
    echo "❌ 模块化构建系统未找到: $BUILDSCRIPTS_DIR"
    echo "请确保运行了构建系统初始化"
    exit 1
}
source "$BUILDSCRIPTS_DIR/common/utils.sh"
source "$BUILDSCRIPTS_DIR/common/config.sh" 
source "$BUILDSCRIPTS_DIR/common/builder.sh"

cleanup_on_error() {
    echo "❌ Build failed"
}

cleanup_on_exit() {
    echo "🔧 Build completed"
}

build_tui_if_needed() {
    local tui_binary="$BUILDSCRIPTS_DIR/bin/sunray_tui"
    local tui_src_dir="$BUILDSCRIPTS_DIR/tui"
    local build_dir="$tui_src_dir/build"
    
    local need_build=false
    
    if [[ ! -f "$tui_binary" ]]; then
        print_status "TUI程序不存在，开始编译..."
        need_build=true
    else
        local newest_src=$(find "$tui_src_dir" -path "*/third_party" -prune -o \( -name "*.cpp" -o -name "*.hpp" -o -name "CMakeLists.txt" \) -newer "$tui_binary" -print | head -1)
        if [[ -n "$newest_src" ]]; then
            print_status "检测到源码更新: $(basename "$newest_src")，重新编译TUI程序..."
            need_build=true
        fi
    fi
    
    if [[ "$need_build" == true ]]; then
        mkdir -p "$build_dir" || { print_error "无法创建构建目录: $build_dir"; exit 1; }
        
        print_status "配置CMake..."
        (cd "$build_dir" && cmake ..) || { print_error "CMake配置失败"; exit 1; }
        
        print_status "编译TUI程序..."
        (cd "$build_dir" && make -j10) || { print_error "编译失败\n请检查源码或依赖项"; exit 1; }
        
        print_status "TUI程序编译完成"
    else
        print_status "TUI程序已是最新版本，直接启动"
    fi
    
    if [[ ! -f "$tui_binary" ]]; then
        print_error "编译完成但找不到可执行文件: $tui_binary"; exit 1
    fi
}

case "$INTERFACE_MODE" in
    "cli")
        source "$BUILDSCRIPTS_DIR/cli/ui.sh"
        ;;
    "tui")
        build_tui_if_needed
        exec "$BUILDSCRIPTS_DIR/bin/sunray_tui"
        ;;
esac

main() {
    local start_time=$(date +%s)
    
    init_config && parse_arguments "$@" || exit 1
    
    local ui_result; run_ui_flow; ui_result=$?
    case $ui_result in
        0) 
            # CLI流程成功，继续构建
            ;;
        1) 
            print_error "UI流程失败"
            exit 1
            ;;
        2) 
            print_status "用户取消构建"
            exit 0
            ;;
        *) 
            print_error "未知的UI流程返回值: $ui_result"
            exit 1
            ;;
    esac
    
    [[ "$DRY_RUN" == true ]] && exit 0
    
    print_status "初始化构建环境..."
    init_build_environment "$WORKSPACE_ROOT" || { print_error "构建环境初始化失败"; exit 1; }
    
    print_status "解析模块依赖关系..."
    local resolved_modules=($(resolve_dependencies "${SELECTED_MODULES[@]}"))
    [[ ${#resolved_modules[@]} -eq 0 ]] && { print_error "没有找到要构建的模块"; exit 1; }
    
    echo -e "\n${CYAN}=== 开始构建 ===${NC}\n构建模块: ${resolved_modules[*]}\n并行任务: $BUILD_JOBS\n"
    
    trap cleanup_build_environment EXIT
    
    if build_modules_parallel "${resolved_modules[@]}"; then
        local total_time=$(($(date +%s) - start_time))
        echo -e "\n${GREEN}🎉 构建完成！${NC}\n总用时: $(format_duration $total_time)"
        show_build_results_table
        post_build_actions; return 0
    else
        echo -e "\n${RED}❌ 构建失败！${NC}"
        show_build_results_table
        return 1
    fi
}

post_build_actions() {
    print_status "执行构建后处理..."
    
    [[ -f "devel/setup.bash" ]] && {
        print_status "ROS工作空间设置文件已生成: devel/setup.bash"
        echo "使用以下命令设置环境: ${CYAN}source devel/setup.bash${NC}"
    }
    
    local available_gb=$(($(df "$WORKSPACE_ROOT" | awk 'NR==2 {print $4}') / 1024 / 1024))
    [[ $available_gb -lt 1 ]] && {
        print_warning "磁盘空间不足 (剩余 ${available_gb}GB)，建议清理构建缓存"
        print_status "使用以下命令清理: $0 --clean"
    }
}

# 显示版本信息
show_version() {
    echo "Sunray 构建系统 v2.0"
}

# 显示帮助信息
show_help() {
    cat << EOF
${CYAN}Sunray 构建系统${NC}

${YELLOW}用法:${NC}
  $0                  启动TUI交互式选择
  $0 --tui            同上
  $0 [选项] [组/模块...] CLI模式构建

${YELLOW}选项:${NC}
  -h, --help          显示帮助
  -y, --yes           自动确认
  -j, --jobs N        并行任务数
  -l, --list          列出可用模块
  -g, --groups        列出模块组
  -s, --same          使用上次选择
  --clean             清理构建目录
  --dry-run           预览构建

${YELLOW}示例:${NC}
  $0 all              构建所有模块
  $0 groupA groupB    构建指定组
  $0 moduleA moduleB  构建指定模块
  $0 -s               重复上次构建
  $0 --list           查看所有选项

${YELLOW}说明:${NC}
  • TUI模式不接受参数，提供交互选择
  • 所有参数功能仅对CLI模式生效
  • TUI自动保存选择，CLI可用-s重用

EOF
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # 直接处理特殊参数
    case "${1:-}" in
        --version|-V) show_version; exit 0 ;;
        --help|-h) show_help; exit 0 ;;
    esac
    
    # 执行主函数
    main "$@"
fi
