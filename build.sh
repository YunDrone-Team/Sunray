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

# Ubuntu 依赖检查（cmake 和 build-essential）
check_ubuntu_build_requirements() {
    # 仅在 Ubuntu/Debian 系列上进行检查
    local os_id="" os_like=""
    if [[ -f /etc/os-release ]]; then
        # shellcheck disable=SC1091
        . /etc/os-release
        os_id="${ID:-}"
        os_like="${ID_LIKE:-}"
    fi

    if [[ "$os_id" != "ubuntu" && "$os_like" != *"ubuntu"* && "$os_like" != *"debian"* ]]; then
        return 0
    fi

    local has_cmake=true
    local has_build_essential=true

    if ! command -v cmake >/dev/null 2>&1; then
        has_cmake=false
    fi

    # 优先用 dpkg-query 判断 build-essential 是否安装
    if command -v dpkg-query >/dev/null 2>&1; then
        if ! dpkg-query -W -f='${Status}' build-essential 2>/dev/null | grep -q "install ok installed"; then
            has_build_essential=false
        fi
    else
        # 回退：检查 gcc/g++ 和 make 是否存在
        if ! command -v make >/dev/null 2>&1 || { ! command -v gcc >/dev/null 2>&1 && ! command -v g++ >/dev/null 2>&1; }; then
            has_build_essential=false
        fi
    fi

    if [[ "$has_cmake" == false || "$has_build_essential" == false ]]; then
        print_error "缺少必要依赖项，无法继续构建："
        [[ "$has_cmake" == false ]] && echo "  - cmake 未安装"
        [[ "$has_build_essential" == false ]] && echo "  - build-essential 未安装"
        echo
        echo "请执行以下命令安装："
        if [[ "$has_cmake" == false && "$has_build_essential" == false ]]; then
            echo "  sudo apt update && sudo apt install -y cmake build-essential"
        elif [[ "$has_cmake" == false ]]; then
            echo "  sudo apt update && sudo apt install -y cmake"
        else
            echo "  sudo apt update && sudo apt install -y build-essential"
        fi
        exit 1
    fi
}

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

    tui_binary_matches_host_arch() {
        local binary_info host_arch
        binary_info="$(file -b "$tui_binary" 2>/dev/null || true)"
        host_arch="$(uname -m)"

        case "$host_arch" in
            x86_64|amd64)
                [[ "$binary_info" == *"x86-64"* || "$binary_info" == *"x86_64"* ]]
                ;;
            aarch64|arm64)
                [[ "$binary_info" == *"aarch64"* || "$binary_info" == *"ARM64"* ]]
                ;;
            armv7l|armv7*|armhf)
                [[ "$binary_info" == *"ARM"* && "$binary_info" != *"aarch64"* ]]
                ;;
            *)
                print_warning "无法识别当前架构: $host_arch，跳过TUI架构校验"
                return 0
                ;;
        esac
    }

    configure_tui_cmake() {
        local cache_file="$build_dir/CMakeCache.txt"
        if [[ -f "$cache_file" ]]; then
            local cached_home
            cached_home="$(grep '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$cache_file" 2>/dev/null | cut -d= -f2-)"
            if [[ -n "$cached_home" && "$cached_home" != "$tui_src_dir" ]]; then
                print_warning "检测到旧CMake缓存路径: $cached_home"
                print_status "清理TUI构建缓存并重新配置..."
                rm -rf "$build_dir/CMakeCache.txt" "$build_dir/CMakeFiles"
            fi
        fi
        (cd "$build_dir" && cmake ..)
    }
    
    # Check dependencies first
    print_status "Checking TUI dependencies..."
    if ! "$tui_src_dir/check_dependencies.sh"; then
        print_error "TUI dependencies check failed"
        exit 1
    fi
    
    local need_build=false
    
    if [[ ! -f "$tui_binary" ]]; then
        print_status "TUI程序不存在，开始编译..."
        need_build=true
    elif ! tui_binary_matches_host_arch; then
        print_warning "检测到TUI程序架构与当前系统不匹配，重新编译..."
        print_status "当前系统架构: $(uname -m)"
        print_status "现有TUI程序: $(file -b "$tui_binary" 2>/dev/null || echo unknown)"
        need_build=true
    else
        local newest_src=$(find "$tui_src_dir" -path "*/third_party" -prune -o \( -name "*.cpp" -o -name "*.hpp" -o -name "CMakeLists.txt" \) -newer "$tui_binary" -print | head -1)
        if [[ -n "$newest_src" ]]; then
            print_status "检测到源码更新: $(basename "$newest_src")，重新编译TUI程序..."
            need_build=true
        fi
    fi
    
    if [[ "$need_build" == true ]]; then
        if [[ -f "$tui_binary" ]] && ! tui_binary_matches_host_arch; then
            print_status "删除架构不匹配的旧TUI程序: $tui_binary"
            rm -f "$tui_binary"
        fi

        mkdir -p "$build_dir" || { print_error "无法创建构建目录: $build_dir"; exit 1; }
        
        print_status "配置CMake..."
        if ! configure_tui_cmake; then
            print_warning "CMake配置失败，尝试清理TUI构建目录后重试..."
            rm -rf "$build_dir/CMakeCache.txt" "$build_dir/CMakeFiles"
            configure_tui_cmake || { print_error "CMake配置失败"; exit 1; }
        fi
        
        print_status "编译TUI程序..."
        (cd "$build_dir" && make -j10) || { print_error "编译失败\n请检查源码或依赖项"; exit 1; }
        
        print_status "TUI程序编译完成"
    else
        print_status "TUI程序已是最新版本，直接启动"
    fi
    
    if [[ ! -f "$tui_binary" ]]; then
        print_error "编译完成但找不到可执行文件: $tui_binary"; exit 1
    fi

    if ! tui_binary_matches_host_arch; then
        print_error "TUI程序架构仍与当前系统不匹配: $(file -b "$tui_binary" 2>/dev/null || echo unknown)"
        exit 1
    fi
}

case "$INTERFACE_MODE" in
    "cli")
        source "$BUILDSCRIPTS_DIR/cli/ui.sh"
        ;;
    "tui")
        # 在进入 CLI/TUI 之前先检查 Ubuntu 依赖
        check_ubuntu_build_requirements
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
    # 在实际构建前对 Ubuntu 依赖进行检查
    check_ubuntu_build_requirements
    init_build_environment "$WORKSPACE_ROOT" || { print_error "构建环境初始化失败"; exit 1; }
    
    print_status "解析模块依赖关系..."
    local resolved_modules=($(resolve_dependencies "${SELECTED_MODULES[@]}"))
    [[ ${#resolved_modules[@]} -eq 0 ]] && { print_error "没有找到要构建的模块"; exit 1; }

    print_status "检查互斥模块产物..."
    prepare_mutex_build_environment "${resolved_modules[@]}" || {
        print_error "互斥模块清理失败"
        exit 1
    }
    
    echo -e "\n${CYAN}=== 开始构建 ===${NC}\n构建模块: ${resolved_modules[*]}\n并行任务: $BUILD_JOBS\n"
    
    trap cleanup_build_environment EXIT
    
    if build_modules_parallel "${resolved_modules[@]}"; then
        local total_time=$(($(date +%s) - start_time))
        echo -e "\n${GREEN}${BOLD}🎉 构建完成！${NC}\n总用时: $(format_duration $total_time)"
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
    
    if [[ -f "devel/setup.bash" ]]; then
        print_status "ROS工作空间设置文件已生成: devel/setup.bash"
        # 这里会让后处理阶段能直接使用最新 ROS 环境；但如果用户通过 ./build.sh
        # 启动脚本，子进程无法反向修改当前终端环境。
        # 需要当前终端后续命令立即识别新包时，shell 机制上仍需在终端中 source 一次。
        # shellcheck disable=SC1091
        source "devel/setup.bash"
        print_success "已在构建脚本进程内执行: source devel/setup.bash"
        print_warning "如果后续命令在当前终端执行仍找不到新包，请在该终端执行: source devel/setup.bash"
    fi
    
    local available_gb=$(($(df "$WORKSPACE_ROOT" | awk 'NR==2 {print $4}') / 1024 / 1024))
    if [[ $available_gb -lt 1 ]]; then
        print_warning "磁盘空间不足 (剩余 ${available_gb}GB)，建议清理构建缓存"
        print_status "使用以下命令清理: $0 --clean"
    fi

    return 0
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
  --clean-bs          清理buildscripts目录
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
