# Sunray TUI Debug Mode

## 🐛 使用 DEBUG 模式

启用调试面板和事件追踪的完整命令：

```bash
DEBUG=1 ./build.sh --tui
```

## 🔍 Debug 功能特性

### 编译时特性
- **零成本调试宏**: 生产版本中所有debug代码被优化为空操作
- **调试符号**: Debug版本包含 `-g -O0` 编译选项  
- **独立二进制**: Debug和Release版本使用不同的可执行文件

### 运行时特性  
- **实时调试面板**: 按 `D` 键切换调试面板显示/隐藏
- **事件追踪**: 记录所有按键、焦点变化、状态更新
- **构建过程跟踪**: 详细记录TUI→CLI切换过程
- **时间戳记录**: 精确到毫秒的事件时间记录

## 📋 使用方法对比

| 模式 | 命令 | 二进制文件 | 大小 | 功能 |
|------|------|------------|------|------|
| 普通模式 | `./build.sh --tui` | `sunray_tui` | ~729KB | 标准TUI界面 |
| 调试模式 | `DEBUG=1 ./build.sh --tui` | `sunray_tui_debug` | ~4.4MB | TUI + 调试面板 |

## 🔧 调试面板操作

在TUI运行时：
1. **开启调试面板**: 按 `D` 键
2. **关闭调试面板**: 再次按 `D` 键  
3. **查看最近事件**: 调试面板显示最近8条事件记录
4. **事件包含**:
   - 按键记录 (KEY: ArrowUp pressed)
   - 焦点变化 (FOCUS: GROUPS -> MODULES) 
   - 状态更新 (STATE: Cursor modules_cursor=2)
   - 构建激活 (BUILD: Button activated)

## 🎯 故障排除

Debug模式特别适用于解决：
- **"按Enter键无响应"** - 查看按键事件是否被正确捕获
- **焦点切换问题** - 追踪Tab键和焦点状态变化
- **TUI→CLI切换失败** - 监控构建请求和TUI退出过程
- **动画卡顿** - 观察事件处理频率和状态更新

## 🏗️ 编译配置

CMake配置会根据DEBUG环境变量自动调整：

```cmake
# DEBUG=1时
option(SUNRAY_DEBUG_ENABLED "Enable comprehensive debug logging and panel" ON)
add_compile_definitions(SUNRAY_DEBUG_ENABLED)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g -O0)

# 普通模式时  
option(SUNRAY_DEBUG_ENABLED "Enable comprehensive debug logging and panel" OFF)
set(CMAKE_BUILD_TYPE Release)
```

## 💡 开发建议

- **开发时**: 使用 `DEBUG=1 ./build.sh --tui` 进行调试
- **测试时**: 使用普通 `./build.sh --tui` 验证生产行为  
- **部署时**: 确保使用普通模式以获得最佳性能