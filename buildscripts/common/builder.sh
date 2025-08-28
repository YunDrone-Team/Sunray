#!/bin/bash
# 构建引擎

source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
source "$(dirname "${BASH_SOURCE[0]}")/config.sh"
source "$(dirname "${BASH_SOURCE[0]}")/build_results.sh"

# 构建状态变量
TOTAL_MODULES=0
COMPLETED_MODULES=0
FAILED_MODULES=0
BUILD_START_TIME=0
BUILD_JOBS=0

# 构建结果记录 - 这些变量现在在build_results.sh中管理
BUILD_RESULTS_MODULES=()
BUILD_RESULTS_STATUS=()
BUILD_RESULTS_TIMES=()
BUILD_RESULTS_START_TIMES=()
BUILD_ORDER=()

# 构建环境初始化
init_build_environment() {
    local workspace_root="$1"
    
    echo "初始化构建环境: $workspace_root"
    
    cd "$workspace_root" || { print_error "无法进入工作目录: $workspace_root"; return 1; }
    
    if [[ -z "$BUILD_JOBS" ]]; then
        BUILD_JOBS=$(($(get_cpu_cores) - 1))
        [[ $BUILD_JOBS -lt 1 ]] && BUILD_JOBS=1
    fi
    
    export BUILD_JOBS ROS_WORKSPACE="$workspace_root"
    BUILD_START_TIME=$(date +%s)
    
    echo "构建环境初始化完成 (并行任务: $BUILD_JOBS)"
    return 0
}

# 记录构建结果
record_build_result() {
    # 这个函数保留用于向后兼容，但实际功能已移至build_results.sh
    local module="$1"
    local status="$2"
    local time="$3"
    
    # 调用新的结果记录函数
    update_build_result "$module" "$status"
}

# 单模块构建
build_module() {
    local module="$1"
    local build_jobs="${2:-$BUILD_JOBS}"
    local start_time=$(date +%s)
    
    # 记录构建顺序和开始时间
    BUILD_ORDER+=("$module")
    record_build_start "$module"
    
    print_status "构建模块: $module"
    
    local source_path=$(get_module_config "$module" "source_path")
    local build_path=$(get_module_config "$module" "build_path")
    
    [[ ! -d "$source_path" ]] && { 
        print_error "模块源代码路径不存在: $source_path"
        update_build_result "$module" "FAILED" "$(date +%s)"
        ((FAILED_MODULES++))
        return 1
    }
    
    echo "源代码路径: $source_path"
    
    if build_catkin_module "$module" "$build_jobs"; then
        update_build_result "$module" "SUCCESS" "$(date +%s)"
        ((COMPLETED_MODULES++))
        print_success "模块构建成功: $module"
        return 0
    else
        update_build_result "$module" "FAILED" "$(date +%s)"
        ((FAILED_MODULES++))
        print_error "模块构建失败: $module"
        return 1
    fi
}

# catkin构建
build_catkin_module() {
    local module="$1"
    local build_jobs="$2"
    
    echo "使用catkin构建模块: $module"
    
    local source_path=$(get_module_config "$module" "source_path")
    local build_path=$(get_module_config "$module" "build_path")
    
    # 分两步：1) catkin_make初始化 2) 直接调用make
    echo "执行构建命令: catkin_make -j$build_jobs --source $source_path --build $build_path"
    echo "================================"
    
    set +e  # 临时禁用错误退出
    
    # 第一步：让catkin_make做初始化（生成Makefile等）
    catkin_make --source "$source_path" --build "$build_path" --cmake-args
    local cmake_exit_code=$?
    
    if [[ $cmake_exit_code -ne 0 ]]; then
        echo "================================"
        echo "catkin构建失败: $module (cmake configuration failed)"
        set -e
        return 1
    fi
    
    # 第二步：直接调用make，它的退出码是可靠的
    cd "$build_path" && make -j"$build_jobs"
    local make_exit_code=$?
    cd - > /dev/null
    
    set -e  # 恢复错误退出
    
    echo "================================"
    echo "[DEBUG] make exit code: $make_exit_code"
    
    if [[ $make_exit_code -eq 0 ]]; then
        echo "catkin构建成功: $module"
        return 0
    else
        echo "catkin构建失败: $module (make failed with exit code: $make_exit_code)"
        return 1
    fi
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

# 显示构建结果表格 - 重定向到新模块
show_build_results_table() {
    # 调用build_results.sh中的函数
    display_build_results_table
}

