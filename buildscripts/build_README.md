# Sunray 构建系统使用指南

## 快速开始

```bash
# 基本构建
./build.sh all                  # 构建所有模块
./build.sh control              # 构建控制组
./build.sh ego sim ugv_control  # 构建多个组

# 常用选项
./build.sh --list               # 列出所有模块
./build.sh --groups             # 列出所有组
./build.sh --dry-run control    # 预览构建
./build.sh --clean              # 清理构建目录
./build.sh all -j 8             # 8线程并行构建
```

## Debug 模式

```bash
# 启用 debug 输出
DEBUG=1 ./build.sh control

# 详细输出模式
./build.sh -v control
```

## 添加新模块

编辑 `buildscripts/config/modules.yaml`：

```yaml
modules:
  your_module:
    name: "your_module"
    description: "模块描述"
    source_path: "path/to/module"
    build_path: "build/your_module"
    dependencies: ["sunray_common"]  # 可选
```

验证：

```bash
./build.sh --list | grep your_module
./build.sh --dry-run your_module
```

## 添加新模块组

编辑 `buildscripts/config/modules.yaml` 的 groups 部分：

```yaml
groups:
  your_group:
    name: "组名称"
    description: "组描述"
    modules: ["module1", "module2", "module3"]
```

验证：

```bash
./build.sh --groups | grep your_group
./build.sh --dry-run your_group
```

## 环境变量

- `DEBUG=1` - 启用调试输出
- `BUILD_JOBS=N` - 设置并行任务数
- `VERBOSE=1` - 详细输出
- `QUIET=1` - 安静模式

## 故障排除

**模块未找到**：检查 `modules.yaml` 中是否定义
**构建失败**：查看终端错误信息，启用 `DEBUG=1`
**依赖错误**：检查 `dependencies` 配置

## 系统架构

```
buildscripts/
├── config/modules.yaml    # 模块和组定义
├── lib/
│   ├── builder.sh         # 构建执行
│   ├── config.sh          # 配置管理  
│   ├── ui.sh              # 用户界面
│   └── utils.sh           # 工具函数
└── build_README.md        # 本文档
```

## 工作原理

### 命令解析流程

当你运行 `./build.sh --list` 时，系统按以下步骤处理：

1. **build.sh** 启动 → 加载 `buildscripts/lib/ui.sh`
2. **ui.sh** 中的 `parse_arguments()` 函数解析 `--list` 参数
3. 匹配到 `--list` 选项后，调用 `list_modules()` 函数
4. `list_modules()` 调用 `config.sh` 中的 `get_all_modules()` 
5. `get_all_modules()` 从 `modules.yaml` 读取所有模块定义
6. 通过 `print_module()` 格式化输出每个模块信息

### 关键函数调用链

```bash
# --list 命令
build.sh → ui.sh:parse_arguments() → ui.sh:list_modules() 
→ config.sh:get_all_modules() → utils.sh:print_module()

# --groups 命令  
build.sh → ui.sh:parse_arguments() → ui.sh:list_groups()
→ config.sh:get_all_groups() → utils.sh:print_group()

# 构建命令
build.sh → ui.sh:parse_arguments() → config.sh:resolve_modules_and_dependencies()
→ builder.sh:build_modules_sequential() → builder.sh:build_catkin_module()
```

### 配置文件解析

`modules.yaml` 通过简单的 AWK 脚本解析：
- 读取 YAML 结构转换为 bash 变量
- 模块信息存储为 `CONFIG_modules_<name>_<field>` 格式
- 组信息存储为 `CONFIG_groups_<name>_<field>` 格式

### 依赖解析算法

1. 从命令行获取目标模块/组
2. 递归展开所有依赖模块
3. 使用拓扑排序确定构建顺序
4. 检测并报告循环依赖

---

**需要更多帮助？** 查看设计理念：`buildscripts/BUILD_SYSTEM_PHILOSOPHY.md`
