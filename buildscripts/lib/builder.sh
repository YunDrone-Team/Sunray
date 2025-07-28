#!/bin/bash
# 构建引擎

source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
source "$(dirname "${BASH_SOURCE[0]}")/config.sh"

# 构建状态变量
TOTAL_MODULES=0
COMPLETED_MODULES=0
FAILED_MODULES=0
BUILD_START_TIME=0
BUILD_JOBS=0

# 构建环境初始化
init_build_environment() {
    local workspace_root="$1"
    
    print_debug "初始化构建环境: $workspace_root"
    
    cd "$workspace_root" || { print_error "无法进入工作目录: $workspace_root"; return 1; }
    
    if [[ -z "$BUILD_JOBS" ]]; then
        BUILD_JOBS=$(($(get_cpu_cores) - 1))
        [[ $BUILD_JOBS -lt 1 ]] && BUILD_JOBS=1
    fi
    
    export BUILD_JOBS ROS_WORKSPACE="$workspace_root"
    BUILD_START_TIME=$(date +%s)
    
    print_debug "构建环境初始化完成 (并行任务: $BUILD_JOBS)"
    return 0
}

# 单模块构建
build_module() {
    local module="$1"
    local build_jobs="${2:-$BUILD_JOBS}"
    local start_time=$(date +%s)
    
    print_status "构建模块: $module"
    
    local source_path=$(get_module_config "$module" "source_path")
    local build_path=$(get_module_config "$module" "build_path")
    
    [[ ! -d "$source_path" ]] && { print_error "模块源代码路径不存在: $source_path"; return 1; }
    
    print_debug "源代码路径: $source_path"
    
    if build_catkin_module "$module" "$build_jobs"; then
        local duration=$(($(date +%s) - start_time))
        ((COMPLETED_MODULES++))
        print_success "模块构建成功: $module (用时: $(format_duration $duration))"
        return 0
    else
        ((FAILED_MODULES++))
        print_error "模块构建失败: $module"
        return 1
    fi
}

# catkin构建
build_catkin_module() {
    local module="$1"
    local build_jobs="$2"
    
    print_debug "使用catkin构建模块: $module"
    
    local source_path=$(get_module_config "$module" "source_path")
    local build_path=$(get_module_config "$module" "build_path")
    
    local build_cmd="catkin_make -j$build_jobs --source $source_path --build $build_path"
    print_debug "执行构建命令: $build_cmd"
    
    eval "$build_cmd" && {
        print_debug "catkin构建成功: $module"
        return 0
    } || {
        print_debug "catkin构建失败: $module"
        return 1
    }
}

# 串行构建模块
build_modules_sequential() {
    local modules=("$@")
    
    [[ ${#modules[@]} -eq 0 ]] && { print_error "没有模块需要构建"; return 1; }
    
    TOTAL_MODULES=${#modules[@]}
    COMPLETED_MODULES=0
    FAILED_MODULES=0
    
    print_status "开始串行构建 $TOTAL_MODULES 个模块..."
    
    local failed_modules=()
    local module_index=1
    
    for module in "${modules[@]}"; do
        printf "\r%b[进度] %d/%d%b" "${BRIGHT_BLUE}" "$module_index" "$TOTAL_MODULES" "${NC}"
        
        if build_module "$module"; then
            printf "\r%b[成功] %d/%d: %s%b\n" "${BRIGHT_GREEN}" "$module_index" "$TOTAL_MODULES" "$module" "${NC}"
        else
            printf "\r%b[失败] %d/%d: %s%b\n" "${BRIGHT_RED}" "$module_index" "$TOTAL_MODULES" "$module" "${NC}"
            failed_modules+=("$module")
        fi
        
        ((module_index++))
    done
    
    echo
    
    if [[ ${#failed_modules[@]} -eq 0 ]]; then
        print_success "所有模块构建成功！"
        return 0
    else
        print_error "构建失败的模块:"
        printf '%s\n' "${failed_modules[@]}" | sed 's/^/  - /'
        return 1
    fi
}

# 并行构建模块
build_modules_parallel() {
    local modules=("$@")
    
    print_status "开始构建模块（使用 $BUILD_JOBS 并行任务）..."
    
    build_modules_sequential "${modules[@]}"
}

# 清理构建环境
cleanup_build_environment() {
    print_debug "清理构建环境..."
    unset BUILD_JOBS ROS_WORKSPACE BUILD_START_TIME
    print_debug "构建环境清理完成"
}

