#!/bin/bash

# 构建引擎 - 核心构建执行逻辑
# 简化版本，兼容bash 3.2

# 导入依赖
source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
source "$(dirname "${BASH_SOURCE[0]}")/config.sh"

# 全局构建状态
TOTAL_MODULES=0
COMPLETED_MODULES=0
FAILED_MODULES=0

# 初始化构建环境
init_build_environment() {
    local workspace_root="$1"
    
    print_debug "Initializing build environment..."
    print_debug "Workspace: $workspace_root"
    
    # 设置工作目录
    cd "$workspace_root" || {
        print_error "无法进入工作目录: $workspace_root"
        return 1
    }
    
    # 设置构建环境变量 - 使用UI中设置的BUILD_JOBS
    if [[ -z "$BUILD_JOBS" ]]; then
        local default_jobs=$(($(get_cpu_cores) - 1))
        export BUILD_JOBS="$default_jobs"
    else
        export BUILD_JOBS
    fi
    export ROS_WORKSPACE="$workspace_root"
    
    print_debug "Build environment initialized successfully"
    return 0
}

# 构建单个模块
build_module() {
    local module="$1"
    local build_jobs="${2:-$BUILD_JOBS}"
    
    print_status "构建模块: $module"
    
    # 获取模块配置
    local source_path=$(get_module_config "$module" "source_path")
    local build_path=$(get_module_config "$module" "build_path")
    
    if [[ ! -d "$source_path" ]]; then
        print_error "模块源代码路径不存在: $source_path"
        return 1
    fi
    
    # 记录开始时间
    local start_time=$(date +%s)
    
    print_debug "源代码路径: $source_path"
    
    # 执行构建
    if build_catkin_module "$module" "$build_jobs"; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        ((COMPLETED_MODULES++))
        print_success "模块构建成功: $module (用时: $(format_duration $duration))"
        return 0
    else
        ((FAILED_MODULES++))
        print_error "模块构建失败: $module"
        return 1
    fi
}

# 使用catkin构建模块
build_catkin_module() {
    local module="$1"
    local build_jobs="$2"
    
    print_debug "使用catkin构建模块: $module"
    
    # 获取源路径和构建路径
    local source_path=$(get_module_config "$module" "source_path")
    local build_path=$(get_module_config "$module" "build_path")
    
    # 构建命令 - 使用原来的--source和--build方式
    local build_cmd="catkin_make -j$build_jobs --source $source_path --build $build_path"
    
    print_debug "执行构建命令: $build_cmd"
    
    # 直接执行构建，输出到终端
    if eval "$build_cmd"; then
        print_debug "catkin构建成功: $module"
        return 0
    else
        print_debug "catkin构建失败: $module"
        return 1
    fi
}

# 串行构建所有模块
build_modules_sequential() {
    local modules=("$@")
    
    if [[ ${#modules[@]} -eq 0 ]]; then
        print_error "没有模块需要构建"
        return 1
    fi
    
    TOTAL_MODULES=${#modules[@]}
    print_status "开始串行构建 $TOTAL_MODULES 个模块..."
    
    local failed_modules=()
    
    for module in "${modules[@]}"; do
        if build_module "$module"; then
            print_status "进度: $COMPLETED_MODULES/$TOTAL_MODULES"
        else
            failed_modules+=("$module")
        fi
    done
    
    # 输出构建结果
    echo
    if [[ ${#failed_modules[@]} -eq 0 ]]; then
        print_success "所有模块构建成功！"
        return 0
    else
        print_error "构建失败的模块:"
        for module in "${failed_modules[@]}"; do
            print_error "  - $module"
        done
        return 1
    fi
}

# 并行构建模块（简化版本）
build_modules_parallel() {
    local modules=("$@")
    
    # 简单实现：串行构建，使用设置的并行任务数
    print_status "开始构建模块（使用 $BUILD_JOBS 并行任务）..."
    
    build_modules_sequential "${modules[@]}"
}

# 清理构建环境
cleanup_build_environment() {
    print_debug "清理构建环境..."
    
    # 清理临时变量
    unset BUILD_JOBS ROS_WORKSPACE
    
    print_debug "构建环境清理完成"
}

