#!/bin/bash
# 工具函数库

# 颜色定义
if [[ -z "${COLORS_INITIALIZED:-}" ]]; then
    if [[ -t 1 ]]; then
        RED=$(tput setaf 1)
        GREEN=$(tput setaf 2)
        YELLOW=$(tput setaf 3)
        BLUE=$(tput setaf 4)
        PURPLE=$(tput setaf 5)
        CYAN=$(tput setaf 6)
        WHITE=$(tput setaf 7)
        
        BRIGHT_RED=$(tput bold; tput setaf 1)
        BRIGHT_GREEN=$(tput bold; tput setaf 2)
        BRIGHT_YELLOW=$(tput bold; tput setaf 3)
        BRIGHT_BLUE=$(tput bold; tput setaf 4)
        BRIGHT_PURPLE=$(tput bold; tput setaf 5)
        BRIGHT_CYAN=$(tput bold; tput setaf 6)
        BRIGHT_WHITE=$(tput bold; tput setaf 7)
        
        BOLD=$(tput bold)
        DIM=$(tput dim)
        UNDERLINE=$(tput smul)
        NC=$(tput sgr0)
    else
        RED='' GREEN='' YELLOW='' BLUE='' PURPLE='' CYAN='' WHITE=''
        BRIGHT_RED='' BRIGHT_GREEN='' BRIGHT_YELLOW='' BRIGHT_BLUE=''
        BRIGHT_PURPLE='' BRIGHT_CYAN='' BRIGHT_WHITE=''
        BOLD='' DIM='' UNDERLINE='' NC=''
    fi
    COLORS_INITIALIZED=1
fi

# 状态颜色
SUCCESS_COLOR="${BRIGHT_GREEN}"
ERROR_COLOR="${BRIGHT_RED}"
WARNING_COLOR="${BRIGHT_YELLOW}"
INFO_COLOR="${BRIGHT_CYAN}"
DEBUG_COLOR="${DIM}${CYAN}"

# 时间戳获取
get_timestamp() {
    printf '%(%Y-%m-%d %H:%M:%S)T\n' -1
}

# 输出函数
print_debug() {
    [[ "${DEBUG:-0}" == "1" ]] && printf '%b[调试] %s%b\n' "${DEBUG_COLOR}" "$1" "${NC}" >&2
}

print_status() {
    printf '%b[状态] %s%b\n' "${INFO_COLOR}" "$1" "${NC}"
}

print_success() {
    printf '%b[成功] %s%b\n' "${SUCCESS_COLOR}" "$1" "${NC}"
}

print_warning() {
    printf '%b[警告] %s%b\n' "${WARNING_COLOR}" "$1" "${NC}"
}

print_error() {
    printf '%b[错误] %s%b\n' "${ERROR_COLOR}" "$1" "${NC}" >&2
}

print_info() {
    printf '%b[信息] %s%b\n' "${INFO_COLOR}" "$1" "${NC}"
}

# 标题打印
print_header() {
    local message="$1"
    echo
    echo -e "${BOLD}${BRIGHT_CYAN}=========================================${NC}"
    echo -e "${BOLD}${BRIGHT_WHITE}  $message${NC}"
    echo -e "${BOLD}${BRIGHT_CYAN}=========================================${NC}"
    echo
}

print_subtitle() {
    echo -e "${SUBTITLE_COLOR}${BOLD}>> $1${NC}"
}

print_separator() {
    echo -e "${DIM}${CYAN}────────────────────────────────────────${NC}"
}

print_double_separator() {
    echo -e "${BRIGHT_PURPLE}═══════════════════════════════════════${NC}"
}

print_highlight() {
    echo -e "${BOLD}${BRIGHT_YELLOW}重点: $1${NC}"
}

# 模块打印
print_module() {
    local module="$1"
    local description="$2"
    
    printf "  ${BOLD}${WHITE}%-30s${NC} ${DIM}%-40s${NC}\n" "$module" "$description"
}

# 构建进度显示
print_build_progress() {
    local current="$1"
    local total="$2"
    local module="$3"
    
    local percentage=$((current * 100 / total))
    local filled=$((percentage / 5))
    local empty=$((20 - filled))
    
    printf "\r${BOLD}${CYAN}构建进度: ${NC}"
    for ((i=0; i<filled; i++)); do printf "${GREEN}█${NC}"; done
    for ((i=0; i<empty; i++)); do printf "${DIM}░${NC}"; done
    printf " ${BOLD}${BRIGHT_BLUE}%d/%d${NC} ${UNDERLINE}%s${NC}" "$current" "$total" "$module"
}

# 构建结果总结
print_build_summary() {
    local total="$1"
    local successful="$2" 
    local failed="$3"
    
    echo
    print_double_separator
    echo -e "${BOLD}${BRIGHT_CYAN}构建结果总结${NC}"
    print_double_separator
    
    if [[ $failed -eq 0 ]]; then
        echo -e "${BOLD}${BRIGHT_GREEN}所有模块构建成功！${NC}"
    else
        echo -e "${BOLD}${BRIGHT_RED}部分模块构建失败${NC}"
    fi
    
    echo
    echo -e "${BRIGHT_CYAN}统计信息:${NC}"
    echo -e "   ${BRIGHT_GREEN}成功: ${successful}${NC}"
    echo -e "   ${BRIGHT_RED}失败: ${failed}${NC}"
    echo -e "   ${BRIGHT_BLUE}总计: ${total}${NC}"
    print_double_separator
}

# 组显示
print_group() {
    local group_name="$1"
    local description="$2"
    
    local group_color
    case "$group_name" in
        "all") group_color="$BOLD$BRIGHT_GREEN" ;;
        "control") group_color="$BOLD$BRIGHT_BLUE" ;;
        "ego") group_color="$BOLD$BRIGHT_PURPLE" ;;
        "sim") group_color="$BOLD$BRIGHT_CYAN" ;;
        "ugv_control") group_color="$BOLD$BRIGHT_YELLOW" ;;
        *) group_color="$BOLD$BRIGHT_WHITE" ;;
    esac
    
    echo -e "${group_color}$group_name${NC} ${BRIGHT_WHITE}$description${NC}"
}

# 模块构建开始
print_build_start() {
    local module="$1"
    
    echo -e "${BOLD}${BRIGHT_CYAN}[ 开始构建: $module ]${NC}"
}

# 模块构建完成
print_build_complete() {
    local module="$1"
    local success="$2"
    local duration="$3"
    
    if [[ "$success" == "true" ]]; then
        echo -e "${BRIGHT_GREEN}${BOLD}[完成] $module${NC} ${DIM}(耗时: ${duration}s)${NC}"
    else
        echo -e "${BRIGHT_RED}${BOLD}[失败] $module${NC} ${DIM}(耗时: ${duration}s)${NC}"
    fi
}

# 配置显示
print_config() {
    local key="$1"
    local value="$2"
    
    printf "${BRIGHT_PURPLE}%-20s${NC}: ${BRIGHT_WHITE}%s${NC}\n" "$key" "$value"
}

# 帮助信息标题
print_help_section() {
    local section="$1"
    echo -e "${BOLD}${BRIGHT_BLUE}$section${NC}"
}

# 命令示例
print_example() {
    local command="$1"
    local description="$2"
    
    echo -e "  ${BRIGHT_GREEN}$command${NC} ${DIM}# $description${NC}"
}

# 错误详情
print_error_details() {
    local error_msg="$1"
    echo -e "${BOLD}${BRIGHT_RED}错误详情: $error_msg${NC}"
}

# 时间格式化
format_duration() {
    local duration="$1"
    local h=$((duration / 3600))
    local m=$(((duration % 3600) / 60))
    local s=$((duration % 60))
    
    (( h > 0 )) && printf '%dh %dm %ds' "$h" "$m" "$s" && return
    (( m > 0 )) && printf '%dm %ds' "$m" "$s" && return
    printf '%ds' "$s"
}

# 估算剩余时间
estimate_remaining_time() {
    local start_time="$1"
    local current_progress="$2"
    local total_progress="$3"
    
    if [ "$current_progress" -eq 0 ]; then
        echo "估算中..."
        return
    fi
    
    local current_time=$(date +%s)
    local elapsed=$((current_time - start_time))
    local rate=$(echo "scale=2; $current_progress / $elapsed" | bc -l)
    local remaining_work=$((total_progress - current_progress))
    local estimated_remaining=$(echo "$remaining_work / $rate" | bc -l | cut -d. -f1)
    
    format_duration "$estimated_remaining"
}

# 安全创建目录
safe_mkdir() {
    local dir="$1"
    local mode="${2:-755}"
    
    if [ ! -d "$dir" ]; then
        print_debug "Creating directory: $dir"
        mkdir -p "$dir"
        chmod "$mode" "$dir"
    fi
}

# 安全删除目录
safe_rmdir() {
    local dir="$1"
    local force="${2:-false}"
    
    if [ ! -d "$dir" ]; then
        return 0
    fi
    
    if [ "$force" = "true" ]; then
        print_debug "Forcefully removing directory: $dir"
        rm -rf "$dir"
    else
        print_debug "Safely removing directory: $dir"
        rmdir "$dir" 2>/dev/null || {
            print_warning "Directory not empty, skipping: $dir"
            return 1
        }
    fi
}

# 检查磁盘空间
check_disk_space() {
    local path="${1:-.}"
    local required_mb="${2:-1024}"
    
    local available_kb=$(df "$path" | awk 'NR==2 {print $4}')
    local available_mb=$((available_kb / 1024))
    
    if [ "$available_mb" -lt "$required_mb" ]; then
        print_warning "磁盘空间不足: 需要 ${required_mb}MB，可用 ${available_mb}MB"
        return 1
    fi
    
    return 0
}

# 字符串去除空白
trim() {
    local str="$1"
    echo "$str" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//'
}

# 字符串转小写
to_lower() {
    echo "$1" | tr '[:upper:]' '[:lower:]'
}

# 字符串转大写
to_upper() {
    echo "$1" | tr '[:lower:]' '[:upper:]'
}

# 检查字符串是否为数字
is_number() {
    local str="$1"
    [[ $str =~ ^[0-9]+$ ]]
}

# 数组包含检查
array_contains() {
    local element="$1"
    shift
    local IFS='|'
    [[ "|$*|" == *"|$element|"* ]]
}

# CPU核心数获取
get_cpu_cores() {
    if [[ -z "${_CPU_CORES_CACHE:-}" ]]; then
        if command -v nproc >/dev/null 2>&1; then
            _CPU_CORES_CACHE=$(nproc)
        elif [[ -r /proc/cpuinfo ]]; then
            _CPU_CORES_CACHE=$(grep -c '^processor' /proc/cpuinfo)
        else
            _CPU_CORES_CACHE=1
        fi
    fi
    echo "${_CPU_CORES_CACHE}"
}

# 获取内存信息
get_memory_info() {
    if [ -r /proc/meminfo ]; then
        awk '/^MemTotal:/ { printf "%.0f\n", $2/1024 }' /proc/meminfo
    else
        echo "unknown"
    fi
}

# 命令存在检查
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# 检查ROS环境
check_ros_environment() {
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS环境未初始化，请先运行: source /opt/ros/\$ROS_DISTRO/setup.bash"
        return 1
    fi
    
    if ! command_exists catkin_make; then
        print_error "catkin_make 命令未找到，请确保ROS开发工具已安装"
        return 1
    fi
    
    print_debug "ROS环境检查通过: $ROS_DISTRO"
    return 0
}

# 设置错误处理
setup_error_handling() {
    trap 'handle_error $? $LINENO "$BASH_COMMAND"' ERR
    trap 'cleanup_on_exit' EXIT INT TERM
}

# 错误处理函数
handle_error() {
    local exit_code="$1"
    local line_number="$2"
    local command="$3"
    
    print_error "脚本执行失败:"
    print_error "  退出码: $exit_code"
    print_error "  行号: $line_number"
    print_error "  命令: $command"
    
    cleanup_on_exit
    exit "$exit_code"
}

# 退出时清理函数
cleanup_on_exit() {
    print_debug "执行清理操作..."
    
    if [ -n "$TEMP_FILES" ]; then
        for temp_file in $TEMP_FILES; do
            [ -f "$temp_file" ] && rm -f "$temp_file"
        done
    fi
    
    if [ -n "$TEMP_DIRS" ]; then
        for temp_dir in $TEMP_DIRS; do
            [ -d "$temp_dir" ] && rm -rf "$temp_dir"
        done
    fi
}

# 询问用户确认
ask_confirmation() {
    local message="$1"
    local default="${2:-n}"
    local response
    
    while true; do
        if [ "$default" = "y" ]; then
            read -p "${message} [Y/n]: " response
            response=${response:-y}
        else
            read -p "${message} [y/N]: " response
            response=${response:-n}
        fi
        
        case $(to_lower "$response") in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            *) print_warning "请输入 y/yes 或 n/no" ;;
        esac
    done
}

# 选择菜单
show_menu() {
    local title="$1"
    shift
    local options=("$@")
    
    echo
    print_header "$title"
    
    for i in "${!options[@]}"; do
        echo "${CYAN}$((i+1)))${NC} ${options[i]}"
    done
    
    echo
    read -p "请选择 (1-${#options[@]}): " choice
    
    if is_number "$choice" && [ "$choice" -ge 1 ] && [ "$choice" -le "${#options[@]}" ]; then
        echo $((choice - 1))
    else
        echo -1
    fi
}