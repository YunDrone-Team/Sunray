#!/bin/bash
# Sunray 模块化构建系统
# ./build.sh --help # 运行构建脚本

set -e

# 目录设置
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$SCRIPT_DIR"
readonly BUILDSCRIPTS_DIR="$SCRIPT_DIR/buildscripts"

# 界面模式设置
INTERFACE_MODE="cli"  # 默认使用CLI模式

[[ ! -d "$BUILDSCRIPTS_DIR" ]] && {
    echo "❌ 模块化构建系统未找到: $BUILDSCRIPTS_DIR"
    echo "请确保运行了构建系统初始化"
    exit 1
}

# Linus-style: Robust terminal reset - no queries, no pollution
reset_terminal() {
    # Option 1: Use tput reset (most reliable, portable)
    if command -v tput >/dev/null 2>&1; then
        tput reset
        return
    fi
    
    # Option 2: Manual ANSI sequences (fallback, no queries)
    printf '\033[?25h'      # Show cursor
    printf '\033[0m'        # Reset all attributes (colors, styles)
    printf '\033[?1000l'    # Disable X10 mouse reporting
    printf '\033[?1001l'    # Disable highlight mouse tracking
    printf '\033[?1002l'    # Disable button event tracking
    printf '\033[?1003l'    # Disable any event tracking
    printf '\033[?1004l'    # Disable focus events
    printf '\033[?1005l'    # Disable UTF-8 mouse mode
    printf '\033[?1006l'    # Disable SGR mouse mode
    printf '\033[?1007l'    # Disable alternate scroll mode
    printf '\033[?1015l'    # Disable Urxvt mouse mode
    printf '\033[?1016l'    # Disable SGR-Pixels mode
    printf '\033[?2004l'    # Disable bracketed paste mode
    printf '\033[?47l'      # Exit alternate screen mode
    printf '\033[?1049l'    # Exit alternate screen + cursor save
    # 注意：移除 '\033[c' 因为它会触发终端查询响应 ^[[?1;2c
    
    # Flush terminal input buffer to clear any pending control sequences
    if [[ -t 0 ]]; then
        read -t 0.1 -N 1000 >/dev/null 2>&1 || true
    fi
}

# Gemini建议：无论脚本如何退出都执行终端清理的安全网
cleanup() {
    reset_terminal
}

# 关键：注册trap确保任何退出情况都会清理终端状态
trap cleanup EXIT INT TERM

# 检查是否指定了界面模式参数
for arg in "$@"; do
    case "$arg" in
        --cli)
            INTERFACE_MODE="cli"
            ;;
        --tui)
            INTERFACE_MODE="tui"
            ;;
    esac
done

# 模块加载
source "$BUILDSCRIPTS_DIR/common/utils.sh"
source "$BUILDSCRIPTS_DIR/common/config.sh" 
source "$BUILDSCRIPTS_DIR/common/builder.sh"

# Linus-style: Robust terminal reset - no queries, no pollution
reset_terminal() {
    # Option 1: Use tput reset (most reliable, portable)
    if command -v tput >/dev/null 2>&1; then
        tput reset
        return
    fi
    
    # Option 2: Manual ANSI sequences (fallback, no queries)
    printf '\033[?25h'      # Show cursor
    printf '\033[0m'        # Reset all attributes (colors, styles)
    printf '\033[?1000l'    # Disable X10 mouse reporting
    printf '\033[?1001l'    # Disable highlight mouse tracking
    printf '\033[?1002l'    # Disable button event tracking
    printf '\033[?1003l'    # Disable any event tracking
    printf '\033[?1004l'    # Disable focus events
    printf '\033[?1005l'    # Disable UTF-8 mouse mode
    printf '\033[?1006l'    # Disable SGR mouse mode
    printf '\033[?1007l'    # Disable alternate scroll mode
    printf '\033[?1015l'    # Disable Urxvt mouse mode
    printf '\033[?1016l'    # Disable SGR-Pixels mode
    printf '\033[?2004l'    # Disable bracketed paste mode
    printf '\033[?47l'      # Exit alternate screen mode
    printf '\033[?1049l'    # Exit alternate screen + cursor save
    # 注意：移除 '\033[c' 因为它会触发终端查询响应 ^[[?1;2c
    
    # Flush terminal input buffer to clear any pending control sequences
    if [[ -t 0 ]]; then
        read -t 0.1 -N 1000 >/dev/null 2>&1 || true
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

# TUI智能编译函数 - Linus式简单解决方案
build_tui_if_needed() {
    local tui_binary="$BUILDSCRIPTS_DIR/bin/sunray_tui"
    local tui_src_dir="$BUILDSCRIPTS_DIR/tui"
    local build_dir="$tui_src_dir/build"
    
    # 检查DEBUG环境变量，决定构建模式
    local debug_mode="OFF"
    if [[ "${DEBUG:-0}" == "1" ]]; then
        debug_mode="ON"
        print_status "🐛 DEBUG模式已启用 - 将编译带有调试面板的TUI"
        tui_binary="${tui_binary}_debug"  # 使用不同的二进制名称
    fi
    
    # 检查是否需要编译
    local need_build=false
    
    # 1. 二进制不存在 - 必须编译
    if [[ ! -f "$tui_binary" ]]; then
        print_status "TUI程序不存在，开始编译..."
        need_build=true
    else
        # 2. 检查源码是否更新 - Linus式：只检查我们的源码，不包含第三方库
        local newest_src=$(find "$tui_src_dir" -path "*/third_party" -prune -o \( -name "*.cpp" -o -name "*.hpp" -o -name "CMakeLists.txt" \) -newer "$tui_binary" -print | head -1)
        if [[ -n "$newest_src" ]]; then
            print_status "检测到源码更新: $(basename "$newest_src")，重新编译TUI程序..."
            need_build=true
        fi
        
        # 3. 检查DEBUG模式变化 - 如果之前是release现在要debug，或相反
        if [[ "$debug_mode" == "ON" && -f "$BUILDSCRIPTS_DIR/bin/sunray_tui" && ! -f "$tui_binary" ]]; then
            print_status "检测到DEBUG模式变化，重新编译..."
            need_build=true
        elif [[ "$debug_mode" == "OFF" && -f "${BUILDSCRIPTS_DIR}/bin/sunray_tui_debug" && ! -f "$tui_binary" ]]; then
            print_status "检测到RELEASE模式变化，重新编译..."
            need_build=true
        fi
    fi
    
    # 执行编译
    if [[ "$need_build" == true ]]; then
        # 创建构建目录
        mkdir -p "$build_dir" || {
            print_error "无法创建构建目录: $build_dir"
            exit 1
        }
        
        # CMake配置 - 根据DEBUG环境变量设置
        print_status "配置CMake... (DEBUG模式: $debug_mode)"
        (cd "$build_dir" && cmake .. -DSUNRAY_DEBUG_ENABLED="$debug_mode") || {
            print_error "CMake配置失败"
            exit 1
        }
        
        # Make编译
        if [[ "$debug_mode" == "ON" ]]; then
            print_status "🔨 编译TUI程序 (包含调试面板和事件追踪)..."
        else
            print_status "🔨 编译TUI程序 (发布版本)..."
        fi
        (cd "$build_dir" && make -j"$(nproc 2>/dev/null || echo 4)") || {
            print_error "编译失败"
            print_error "请检查源码或依赖项"
            exit 1
        }
        
        # 重命名二进制文件以区分debug和release版本
        if [[ "$debug_mode" == "ON" ]]; then
            local source_binary="$BUILDSCRIPTS_DIR/bin/sunray_tui"
            local debug_binary="$BUILDSCRIPTS_DIR/bin/sunray_tui_debug"
            if [[ -f "$source_binary" ]]; then
                mv "$source_binary" "$debug_binary"
                print_status "✅ DEBUG版本TUI程序编译完成 (包含调试面板)"
                print_status "💡 运行时调试面板默认开启，按 'D' 键切换显示"
            else
                print_error "找不到编译生成的二进制文件: $source_binary"
                exit 1
            fi
        else
            print_status "✅ TUI程序编译完成"
        fi
    else
        if [[ "$debug_mode" == "ON" ]]; then
            print_status "🐛 DEBUG版本TUI程序已是最新版本，直接启动"
        else
            print_status "TUI程序已是最新版本，直接启动"
        fi
    fi
    
    # 最终检查
    if [[ ! -f "$tui_binary" ]]; then
        print_error "编译完成但找不到可执行文件: $tui_binary"
        exit 1
    fi
}

# 根据界面模式加载对应的UI
case "$INTERFACE_MODE" in
    "cli")
        source "$BUILDSCRIPTS_DIR/cli/ui.sh"
        ;;
    "tui")
        # TUI模式：智能编译和启动
        build_tui_if_needed
        
        # 根据DEBUG环境变量选择正确的二进制文件
        TUI_BINARY="$BUILDSCRIPTS_DIR/bin/sunray_tui"
        if [[ "${DEBUG:-0}" == "1" ]]; then
            TUI_BINARY="${TUI_BINARY}_debug"
        fi
        
        exec "$TUI_BINARY" "$@"
        ;;
esac

# 主函数
main() {
    local start_time=$(date +%s)
    
    # 检查是否指定了界面模式参数
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
        
        # Linus风格：构建失败时清理终端状态，解决控制字符污染
        if declare -f cleanup_on_error >/dev/null 2>&1; then
            cleanup_on_error
        elif declare -f reset_terminal >/dev/null 2>&1; then
            reset_terminal
            echo "🔧 终端状态已重置"
        fi
        
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
        --cli) shift; main "$@" ;;
        --tui) shift; main "$@" ;;
        *) main "$@" ;;
    esac
fi