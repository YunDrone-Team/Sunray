# !/bin/bash

# Sunray 统一构建脚本
# 所有 Sunray 模块的统一构建脚本，支持多核构建
# 作者: 从现有构建脚本自动生成
# 用法: ./build.sh --help

set -e  # 任意命令出错则退出

# 颜色定义，提升可读性
RED=$(tput setaf 1)
GREEN=$(tput setaf 2)
YELLOW=$(tput bold; tput setaf 3)
BLUE=$(tput setaf 4)
PURPLE=$(tput setaf 5)
CYAN=$(tput setaf 6)
WHITE=$(tput bold; tput setaf 7)
NC=$(tput sgr0) # 无颜色

# 全局变量
START_TIME=$(date +%s)
BUILD_COUNT=0
FAILED_BUILDS=()
SUCCESSFUL_BUILDS=()
JOBS=$(nproc --ignore=1)  # 使用核数减一的并行任务数
CLEAN_BUILD=false
VERBOSE=false
CLEAN_ONLY=false

# 打印状态信息（蓝色）
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# 打印成功信息（绿色）
print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# 打印警告信息（黄色）
print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# 打印错误信息（红色）
print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 打印标题（紫色）
print_header() {
    echo -e "${PURPLE}=== $1 ===${NC}"
}

# 显示帮助信息
show_help() {
    cat << EOF
${WHITE}Sunray 统一构建脚本${NC}

${CYAN}用法:${NC}
    ./build.sh [选项]

${CYAN}选项说明:${NC}
    --all         构建所有模块（默认）
    --uav         仅构建无人机（UAV）控制模块
    --ugv         仅构建地面无人车（UGV）控制模块  
    --sim         仅构建仿真模块
    --ego         使用 EGO 规划器（排除 FUEL）
    --fuel        使用 FUEL 规划器（排除 EGO）
    --clean       构建前清理构建目录
    --clean-only  仅清理所有构建目录，不进行构建
    --verbose     显示详细的 catkin_make 输出
    --jobs N      设置并行任务数（默认: $(nproc --ignore=1)）
    --help        显示本帮助信息

${CYAN}使用示例:${NC}
    ./build.sh                      # 构建所有模块
    ./build.sh --uav --sim          # 构建 UAV 与仿真模块
    ./build.sh --clean --ego        # 清理后构建 EGO 模块
    ./build.sh --clean-only          # 仅清理所有构建目录
    ./build.sh --jobs 4 --verbose   # 使用4个任务并显示详细输出

${CYAN}模块分组:${NC}
    ${GREEN}通用模块:${NC} sunray_common, sunray_tutorial, sunray_media
    ${GREEN}UAV 模块:${NC} sunray_uav_control, vrpn_client_ros, sunray_planner_utils
    ${GREEN}UGV 模块:${NC} sunray_ugv_control, turn_on_wheeltec_robot
    ${GREEN}仿真模块:${NC} sunray_simulator, simulator_utils, gazebo_plugin
    ${GREEN}通信模块:${NC} sunray_communication_bridge
    ${GREEN}编队模块:${NC} sunray_formation
    ${GREEN}规划器模块:${NC} ego-planner-swarm 或 FUEL（二者互斥）

EOF
}

# 构建单个模块并处理异常
build_module() {
    local module_name="$1"
    local source_path="$2"
    local build_path="$3"
    local description="$4"
    
    BUILD_COUNT=$((BUILD_COUNT + 1))
    
    print_header "Building Module $BUILD_COUNT: $module_name"
    print_status "$description"
    print_status "Source: $source_path"
    print_status "Build dir: $build_path"
    print_status "Using $JOBS parallel jobs"
    
    if [ "$CLEAN_BUILD" = true ]; then
        if [ -d "$build_path" ]; then
            print_status "Cleaning build directory: $build_path"
            rm -rf "$build_path"
        fi
    fi
    
    mkdir -p "$build_path"
    
    local start_module_time=$(date +%s)
    
    local catkin_cmd="catkin_make --source $source_path --build $build_path -j$JOBS"
    
    if [ "$VERBOSE" = true ]; then
        print_status "Executing: $catkin_cmd"
        if $catkin_cmd; then
            local end_module_time=$(date +%s)
            local module_duration=$((end_module_time - start_module_time))
            print_success "$module_name built successfully in ${module_duration}s"
            SUCCESSFUL_BUILDS+=("$module_name (${module_duration}s)")
        else
            print_error "$module_name build failed!"
            FAILED_BUILDS+=("$module_name")
            return 1
        fi
    else
        print_status "Building... (use --verbose for detailed output)"
        if $catkin_cmd > /dev/null 2>&1; then
            local end_module_time=$(date +%s)
            local module_duration=$((end_module_time - start_module_time))
            print_success "$module_name built successfully in ${module_duration}s"
            SUCCESSFUL_BUILDS+=("$module_name (${module_duration}s)")
        else
            print_error "$module_name build failed!"
            print_error "Run with --verbose to see detailed error output"
            FAILED_BUILDS+=("$module_name")
            return 1
        fi
    fi
    
    echo
}

# 检查模块源路径是否存在
check_source() {
    local source_path="$1"
    if [ ! -d "$source_path" ]; then
        print_warning "Source directory not found: $source_path"
        return 1
    fi
    return 0
}

# 构建通用模块（大多数构建所需）
build_common_modules() {
    print_header "Building Common Modules"
    
    if check_source "General_Module/sunray_common"; then
        build_module "sunray_common" "General_Module/sunray_common" "build/sunray_common" "Common utilities and libraries"
    fi
    
    if check_source "General_Module/sunray_tutorial"; then
        build_module "sunray_tutorial" "General_Module/sunray_tutorial" "build/sunray_tutorial" "Tutorial and example code"
    fi
    
    if check_source "General_Module/sunray_media"; then
        build_module "sunray_media" "General_Module/sunray_media" "build/sunray_media" "Media processing utilities"
    fi
    
    if check_source "Comunication_Module/sunray_communication_bridge"; then
        build_module "sunray_communication_bridge" "Comunication_Module/sunray_communication_bridge" "build/sunray_communication_bridge" "Communication bridge module"
    fi
}

# 构建 UAV 控制模块
build_uav_modules() {
    print_header "Building UAV Control Modules"
    
    if check_source "General_Module/sunray_uav_control"; then
        build_module "sunray_uav_control" "General_Module/sunray_uav_control" "build/sunray_uav_control" "UAV control system"
    fi
    
    if check_source "External_Module/vrpn_client_ros"; then
        build_module "vrpn_client_ros" "External_Module/vrpn_client_ros" "build/vrpn_client_ros" "VRPN client for ROS"
    fi
    
    if check_source "General_Module/sunray_planner_utils"; then
        build_module "sunray_planner_utils" "General_Module/sunray_planner_utils" "build/sunray_planner_utils" "Path planning utilities"
    fi
}

# 构建 UGV 控制模块
build_ugv_modules() {
    print_header "Building UGV Control Modules"
    print_warning "UGV modules require octomap and serial dependencies"
    
    if check_source "General_Module/sunray_ugv_control"; then
        build_module "sunray_ugv_control" "General_Module/sunray_ugv_control" "build/sunray_ugv_control" "UGV control system"
    fi
    
    if check_source "External_Module/turn_on_wheeltec_robot"; then
        build_module "turn_on_wheeltec_robot" "External_Module/turn_on_wheeltec_robot" "build/turn_on_wheeltec_robot" "Wheeltec robot activation"
    fi
}

# 构建仿真模块
build_simulation_modules() {
    print_header "Building Simulation Modules"
    
    if check_source "Simulation/sunray_simulator"; then
        build_module "sunray_simulator" "Simulation/sunray_simulator" "build/sunray_simulator" "Main simulation framework"
    fi
    
    if check_source "Simulation/simulator_utils"; then
        build_module "simulator_utils" "Simulation/simulator_utils" "build/simulator_utils" "Simulation utilities"
    fi
    
    if check_source "Simulation/gazebo_plugin"; then
        build_module "gazebo_plugin" "Simulation/gazebo_plugin" "build/gazebo_plugin" "Custom Gazebo plugins"
    fi
}

# 构建 EGO 规划器（与 FUEL 冲突）
build_ego_planner() {
    print_header "Building EGO Planner"
    print_warning "EGO planner conflicts with FUEL - they cannot be built together"
    
    if check_source "External_Module/ego-planner-swarm"; then
        build_module "ego-planner" "External_Module/ego-planner-swarm" "build/ego-planner" "EGO planner for swarm navigation"
    fi
}

# 构建 FUEL 规划器（与 EGO 冲突）
build_fuel_planner() {
    print_header "Building FUEL Planner"  
    print_warning "FUEL planner conflicts with EGO - they cannot be built together"
    
    if check_source "External_Module/FUEL"; then
        build_module "FUEL" "External_Module/FUEL" "build/FUEL" "FUEL fast autonomous exploration"
    fi
}

# 构建编队模块
build_formation_modules() {
    print_header "Building Formation Modules"
    
    if check_source "sunray_formation"; then
        build_module "sunray_formation" "sunray_formation" "build/sunray_formation" "Multi-agent formation control"
    fi
}

# 清理所有构建目录
clean_all_build_dirs() {
    print_header "Cleaning All Build Directories"
    local dirs=(
        "build/sunray_common"
        "build/sunray_tutorial"
        "build/sunray_media"
        "build/sunray_communication_bridge"
        "build/sunray_uav_control"
        "build/vrpn_client_ros"
        "build/sunray_planner_utils"
        "build/sunray_ugv_control"
        "build/turn_on_wheeltec_robot"
        "build/sunray_simulator"
        "build/simulator_utils"
        "build/gazebo_plugin"
        "build/ego-planner"
        "build/FUEL"
        "build/sunray_formation"
    )
    for dir in "${dirs[@]}"; do
        if [ -d "$dir" ]; then
            print_status "Removing $dir"
            rm -rf "$dir"
        fi
    done
    print_success "All build directories cleaned."
}

# 打印构建总结
print_summary() {
    local end_time=$(date +%s)
    local total_duration=$((end_time - START_TIME))
    local minutes=$((total_duration / 60))
    local seconds=$((total_duration % 60))
    
    print_header "Build Summary"
    
    echo -e "${WHITE}Total build time:${NC} ${minutes}m ${seconds}s"
    echo -e "${WHITE}Parallel jobs used:${NC} $JOBS"
    echo -e "${WHITE}Total modules processed:${NC} $BUILD_COUNT"
    echo
    
    if [ ${#SUCCESSFUL_BUILDS[@]} -gt 0 ]; then
        print_success "Successfully built modules (${#SUCCESSFUL_BUILDS[@]}):"
        for build in "${SUCCESSFUL_BUILDS[@]}"; do
            echo -e "  ${GREEN}✓${NC} $build"
        done
        echo
    fi
    
    if [ ${#FAILED_BUILDS[@]} -gt 0 ]; then
        print_error "Failed builds (${#FAILED_BUILDS[@]}):"
        for build in "${FAILED_BUILDS[@]}"; do
            echo -e "  ${RED}✗${NC} $build"
        done
        echo
        print_error "Build completed with errors. Check the output above for details."
        exit 1
    else
        print_success "All builds completed successfully!"
    fi
}

# 主程序逻辑
main() {
    local build_all=true
    local build_uav=false
    local build_ugv=false
    local build_sim=false
    local build_ego=false
    local build_fuel=false
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --help)
                show_help
                exit 0
                ;;
            --all)
                build_all=true
                shift
                ;;
            --uav)
                build_all=false
                build_uav=true
                shift
                ;;
            --ugv)
                build_all=false
                build_ugv=true
                shift
                ;;
            --sim)
                build_all=false
                build_sim=true
                shift
                ;;
            --ego)
                build_all=false
                build_ego=true
                shift
                ;;
            --fuel)
                build_all=false
                build_fuel=true
                shift
                ;;
            --clean)
                CLEAN_BUILD=true
                shift
                ;;
            --clean-only)
                CLEAN_ONLY=true
                shift
                ;;
            --verbose)
                VERBOSE=true
                shift
                ;;
            --jobs)
                if [[ -n $2 ]] && [[ $2 =~ ^[0-9]+$ ]]; then
                    JOBS=$2
                    shift 2
                else
                    print_error "Invalid number of jobs: $2"
                    exit 1
                fi
                ;;
            *)
                print_error "Unknown option: $1"
                print_status "Use --help for usage information"
                exit 1
                ;;
        esac
    done
    
    # 如果仅清理，执行清理后退出
    if [ "$CLEAN_ONLY" = true ]; then
        clean_all_build_dirs
        exit 0
    fi
    
    # 检查 EGO 与 FUEL 冲突
    if [ "$build_ego" = true ] && [ "$build_fuel" = true ]; then
        print_error "Cannot build both EGO and FUEL planners - they conflict!"
        print_status "Choose either --ego or --fuel, not both"
        exit 1
    fi
    
    # 显示构建配置信息
    print_header "Sunray Unified Build Script"
    print_status "Build configuration:"
    print_status "  Parallel jobs: $JOBS"
    print_status "  Clean build: $CLEAN_BUILD"
    print_status "  Verbose output: $VERBOSE"
    
    if [ "$build_all" = true ]; then
        print_status "  Mode: Build ALL modules (default EGO planner)"
    else
        local modes=()
        [ "$build_uav" = true ] && modes+=("UAV")
        [ "$build_ugv" = true ] && modes+=("UGV") 
        [ "$build_sim" = true ] && modes+=("Simulation")
        [ "$build_ego" = true ] && modes+=("EGO Planner")
        [ "$build_fuel" = true ] && modes+=("FUEL Planner")
        print_status "  Mode: Build selected modules (${modes[*]})"
    fi
    echo
    
    # 根据配置执行构建
    if [ "$build_all" = true ]; then
        build_common_modules
        build_uav_modules
        build_simulation_modules
        build_ego_planner  # 默认使用 EGO 规划器进行完整构建
        build_formation_modules
        # UGV 模块默认注释，因需要额外依赖
        print_warning "UGV modules skipped (use --ugv to include them)"
    else
        # 构建特定模块时始终先构建通用模块
        if [ "$build_uav" = true ] || [ "$build_ugv" = true ] || [ "$build_sim" = true ]; then
            build_common_modules
        fi
        
        [ "$build_uav" = true ] && build_uav_modules
        [ "$build_ugv" = true ] && build_ugv_modules
        [ "$build_sim" = true ] && build_simulation_modules
        [ "$build_ego" = true ] && build_ego_planner
        [ "$build_fuel" = true ] && build_fuel_planner
    fi
    
    print_summary
}

# 运行主程序，传入所有参数
main "$@"