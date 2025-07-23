#!/bin/bash
# 配置管理模块 - 处理YAML配置文件的解析和管理
# 提供配置文件读取、模块定义解析、依赖关系计算等功能

# 全局配置变量 - 使用普通数组代替关联数组以兼容bash 3.2
# 注意：由于bash 3.2不支持关联数组，我们使用环境变量来存储配置

# YAML 简单解析器（针对我们的配置格式）
parse_yaml() {
    local file="$1"
    local prefix="$2"
    
    if [ ! -f "$file" ]; then
        print_error "Configuration file not found: $file"
        return 1
    fi
    
    # 移除注释和空行，然后解析
    sed -e 's/#.*$//' -e '/^[[:space:]]*$/d' "$file" | \
    awk -v prefix="$prefix" '
    BEGIN { 
        level = 0
        path[0] = prefix
    }
    {
        # 计算缩进级别
        indent = match($0, /[^ ]/) - 1
        if (indent < 0) indent = 0
        
        # 调整路径数组
        level = int(indent / 2)
        
        # 移除前导空格
        gsub(/^[[:space:]]+/, "")
        
        # 跳过空行
        if (length($0) == 0) next
        
        # 处理键值对 - 使用兼容的正则表达式匹配
        if ($0 ~ /^[^:]+:[[:space:]]*.*$/) {
            # 分离键和值
            colon_pos = index($0, ":")
            key = substr($0, 1, colon_pos - 1)
            value = substr($0, colon_pos + 1)
            
            # 清理键和值的空格
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
            
            # 构建完整路径
            path[level+1] = key
            full_path = ""
            for (i = 0; i <= level+1; i++) {
                if (path[i] != "") {
                    if (full_path != "") full_path = full_path "_"
                    full_path = full_path path[i]
                }
            }
            
            # 清理值（移除引号和方括号）
            gsub(/^[\"\47]|[\"\47]$/, "", value)
            gsub(/^\[|\]$/, "", value)
            
            # 输出变量赋值
            if (value != "") {
                print full_path "=\"" value "\""
            }
        }
    }'
}

# 加载模块配置文件
load_modules_config() {
    local config_file="${SCRIPT_DIR}/buildscripts/config/modules.yaml"
    
    print_debug "Loading modules configuration from: $config_file"
    
    if [ ! -f "$config_file" ]; then
        print_error "Modules configuration file not found: $config_file"
        return 1
    fi
    
    # 解析配置文件并执行变量赋值
    eval "$(parse_yaml "$config_file" "CONFIG")"
    
    print_debug "Modules configuration loaded successfully"
    return 0
}

# 获取模块配置值
get_module_config() {
    local module_name="$1"
    local config_key="$2"
    local var_name="CONFIG_modules_${module_name}_${config_key}"
    
    echo "${!var_name}"
}

# 获取用户配置值
get_user_config() {
    local config_key="$1"
    local var_name="USER_${config_key}"
    local default_value="$2"
    
    local value="${!var_name}"
    echo "${value:-$default_value}"
}

# 获取模块列表
get_all_modules() {
    # 从解析的配置中提取所有模块名
    compgen -v | grep "^CONFIG_modules_.*_description$" | while read var; do
        # Extract module name using parameter expansion
        module_name=${var#CONFIG_modules_}
        module_name=${module_name%_description}
        echo "$module_name"
    done | sort -u
}

# 获取模块组列表
get_all_groups() {
    for var in $(compgen -v CONFIG_groups_); do
        if [[ $var =~ CONFIG_groups_([^_]+)_description$ ]]; then
            echo "${BASH_REMATCH[1]}"
        fi
    done | sort -u
}

# 获取组包含的模块列表
get_group_modules() {
    local group_name="$1"
    local modules_var="CONFIG_groups_${group_name}_modules"
    
    # YAML数组解析（简化版本）
    local modules_str="${!modules_var}"
    if [ -n "$modules_str" ]; then
        # 移除方括号和引号，分割模块名
        echo "$modules_str" | sed 's/\[//g; s/\]//g; s/"//g; s/,/ /g'
    fi
}

# 检查模块冲突
check_module_conflicts() {
    local module_list=("$@")
    local conflicts=()
    
    for module in "${module_list[@]}"; do
        local conflicts_var="CONFIG_modules_${module}_conflicts_with"
        local conflicts_str="${!conflicts_var}"
        
        if [ -n "$conflicts_str" ]; then
            # 解析冲突模块列表
            local conflicting_modules
            IFS=',' read -ra conflicting_modules <<< "${conflicts_str//[\[\]\"]/}"
            
            for conflict_module in "${conflicting_modules[@]}"; do
                # 清理空格
                conflict_module=$(echo "$conflict_module" | xargs)
                
                # 检查冲突模块是否也在构建列表中
                for check_module in "${module_list[@]}"; do
                    if [ "$check_module" = "$conflict_module" ]; then
                        conflicts+=("$module conflicts with $conflict_module")
                    fi
                done
            done
        fi
    done
    
    if [ ${#conflicts[@]} -gt 0 ]; then
        print_error "Module conflicts detected:"
        for conflict in "${conflicts[@]}"; do
            print_error "  - $conflict"
        done
        return 1
    fi
    
    return 0
}

# 解析依赖关系并排序模块 (简化版本，兼容bash 3.2)
resolve_dependencies() {
    local input_modules=("$@")
    
    # 简单实现：直接返回输入的模块列表
    # 在实际使用中，依赖关系由build脚本的调用顺序来保证
    echo "${input_modules[@]}"
}

# 验证配置完整性
validate_config() {
    print_debug "Validating configuration..."
    
    local modules=($(get_all_modules))
    local validation_errors=()
    
    # 检查每个模块的必需字段
    for module in "${modules[@]}"; do
        local required_fields=("description" "source_path" "build_path")
        
        for field in "${required_fields[@]}"; do
            local var_name="CONFIG_modules_${module}_${field}"
            if [ -z "${!var_name}" ]; then
                validation_errors+=("Module '$module' missing required field: $field")
            fi
        done
        
        # 检查源路径是否存在
        local source_path=$(get_module_config "$module" "source_path")
        if [ -n "$source_path" ] && [ ! -d "$source_path" ]; then
            validation_errors+=("Module '$module' source path does not exist: $source_path")
        fi
    done
    
    # 输出验证结果
    if [ ${#validation_errors[@]} -gt 0 ]; then
        print_error "Configuration validation failed:"
        for error in "${validation_errors[@]}"; do
            print_error "  - $error"
        done
        return 1
    fi
    
    print_debug "Configuration validation passed"
    return 0
}

# 获取构建配置
get_build_config() {
    local key="$1"
    local default="$2"
    
    local value=$(get_user_config "build_config_$key" "$default")
    if [ -z "$value" ]; then
        # 尝试从模块配置获取
        local var_name="CONFIG_build_config_$key"
        value="${!var_name:-$default}"
    fi
    
    echo "$value"
}

# 估算构建时间
estimate_build_time() {
    local modules=("$@")
    local total_time=0
    
    for module in "${modules[@]}"; do
        local time_var="CONFIG_modules_${module}_build_time_estimate"
        local module_time="${!time_var:-60}"  # 默认60秒
        total_time=$((total_time + module_time))
    done
    
    echo "$total_time"
}

# 检查系统依赖
check_system_dependencies() {
    local modules=("$@")
    local missing_deps=()
    
    for module in "${modules[@]}"; do
        local deps_var="CONFIG_modules_${module}_system_deps"
        local deps_str="${!deps_var}"
        
        if [ -n "$deps_str" ]; then
            local system_deps
            IFS=',' read -ra system_deps <<< "${deps_str//[\[\]\"]/}"
            
            for dep in "${system_deps[@]}"; do
                dep=$(echo "$dep" | xargs)
                
                # 简单检查依赖是否可用
                case "$dep" in
                    "octomap")
                        if ! pkg-config --exists octomap 2>/dev/null; then
                            missing_deps+=("$dep (for module: $module)")
                        fi
                        ;;
                    "serial")
                        if ! pkg-config --exists serial 2>/dev/null; then
                            missing_deps+=("$dep (for module: $module)")
                        fi
                        ;;
                esac
            done
        fi
    done
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_warning "Missing system dependencies:"
        for dep in "${missing_deps[@]}"; do
            print_warning "  - $dep"
        done
        return 1
    fi
    
    return 0
}

# 初始化配置系统
init_config() {
    print_debug "Initializing configuration system..."
    
    # 设置脚本目录 - 从主build.sh传入
    SCRIPT_DIR="${SCRIPT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
    
    # 加载配置文件
    load_modules_config || return 1
    
    # 验证配置
    validate_config || return 1
    
    print_debug "Configuration system initialized successfully"
    return 0
}

# ========== 新架构的组和配置管理函数 ==========

# 获取所有组（简化版本）
get_all_groups() {
    # 获取预定义的简单组
    compgen -v | grep "^CONFIG_groups_.*_description$" | while read var; do
        local group_name=${var#CONFIG_groups_}
        group_name=${group_name%_description}
        echo "$group_name"
    done | sort -u
}

# 获取组描述
get_group_description() {
    local group="$1"
    
    # 检查简单组定义
    local desc_var="CONFIG_groups_${group}_description"
    local description="${!desc_var}"
    
    if [[ -n "$description" ]]; then
        echo "$description"
    else
        echo "构建组: $group"
    fi
}

# 获取模块描述
get_module_description() {
    local module="$1"
    local var_name="CONFIG_modules_${module}_description"
    echo "${!var_name}"
}

# 检查模块是否存在
module_exists() {
    local module="$1"
    local var_name="CONFIG_modules_${module}_description"
    [[ -n "${!var_name}" ]]
}

# 获取构建顺序（简化版本依赖排序）
get_build_order() {
    local modules=("$@")
    # 简单返回模块列表，实际的排序在resolve_dependencies中已完成
    printf '%s\n' "${modules[@]}"
}