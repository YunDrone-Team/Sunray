# TUI 渲染层解耦方案（tui_render.cpp 拆分）

目标
- 降低 tui_render.cpp 体积与复杂度，按“职责单一、命名统一”的原则拆分。
- 稳定对外接口（UIRenderer 与 UIState 保持不变），内部按模块化组织。
- 明确事件路由（键盘/鼠标/按钮面板）与渲染职责边界，方便后续维护与增量修改。

现状痛点
- 文件过大、函数过多，逻辑交叉（渲染/事件/布局/调试/业务混杂）。
- 局部状态耦合，改动一处易牵动多处，难以回归。
- 键盘/鼠标事件与组件事件顺序耦合在一处，容易产生回归（如 Tab/Space/Enter）。

拆分原则
- 保留对外头文件 `buildscripts/tui/include/tui_render.hpp` 与类名 UIRenderer。
- 以“功能域”为单位将 UIRenderer 的成员函数实现按 .cpp 拆分；每个 .cpp 只包含相关函数实现，不新增外部可见 API。
- 命名统一：tui_render_*.cpp；内部辅助放匿名命名空间或 `namespace sunray_tui::detail`。

拟议文件结构
- 入口与组件
  - `tui_render_component.cpp`：create_component() 及构建顶层界面骨架（标题、双栏容器、描述/详情拼装、按钮行、调试/按键指南拼装）。
  - `tui_render_buttons.cpp`：按钮行（组件化）逻辑与渲染 transform、按钮面板焦点、高亮策略、异步清理触发（trigger_clear_build_clean）。

- 事件处理
  - `tui_render_keyboard.cpp`：键盘事件主路由（Tab/Shift+Tab、Enter、Space、Arrows、Esc/Q/C 等），面板切换、按钮面板左右切换与回车触发。
  - `tui_render_mouse.cpp`：鼠标事件主路由（移动/点击/滚轮）、Hover 计算、点击命中至组/模块。

- 布局与映射
  - `tui_render_layout.cpp`：rebuild_dual_column_coordinate_mapping()、calculate_dynamic_column_boundaries()、左右栏尺寸/可视高度计算。
  - `tui_render_cells.cpp`：render_group_item() / render_module_item() 等单元渲染。

- 详情与调试
  - `tui_render_details.cpp`：update_details_on_hover()（根据 hover/selection 生成描述与详情文本）。
  - `tui_render_debug.cpp`：render_debug_window() / calculate_debug_content_lines()；render_key_guide() / calculate_key_guide_content_lines()。

- 公共/内部工具（必要时）
  - `tui_render_util.cpp`：小型格式化/字符串/裁剪/配色工具（仅限渲染层内部使用）。
  - 不额外新增对外头文件；如多个 .cpp 共享内部声明，集中到 `buildscripts/tui/include/tui_render_internal.hpp`，只被上述 .cpp 引用。

命名与包含规范
- 统一包含顺序：本模块头 → 公共依赖头（ftxui/dom, ftxui/component, std）→ 本地其他子模块头。
- 禁止在 .hpp 暴露“仅用于实现”的结构（走 internal 头，或匿名命名空间内定义）。
- 所有渲染 transform 均为“纯样式”函数，输入来源统一由 UIRenderer 成员读取，避免跨模块修改状态。

事件路由架构（稳定版）
- 鼠标事件：先交给按钮组件（若在按钮内）；若未消费，再交给双栏鼠标路由（组/模块 / 滚轮滚动）。
- 键盘事件：先交给双栏键盘路由（Tab/Shift+Tab 面板切换；Arrows 导航；Space/Enter 业务）；若未消费，再交给按钮组件（允许键盘激活按钮）。
- 按键面板：UIState 维护 `build_button_focused` + `button_focus_index`（0=开始构建，1=清除构建）；
  - 当 `build_button_focused` 为 true 时，左右键修改 `button_focus_index`；
  - Enter 根据 `button_focus_index` 触发对应按钮；
  - 悬停/聚焦高亮统一在 buttons transform 中通过 `build_button_focused` 与 Hoverable 标志实现。

CMake 拆分增量计划
- 第一阶段：只移动实现，不改任何对外接口与业务逻辑（低风险投产）。
  - 在 `buildscripts/tui/CMakeLists.txt` 的 `add_executable(sunray_tui ...)` 源文件列表中，追加新 .cpp 文件；逐步从 tui_render.cpp 挪出对应实现（方法体）至新文件。
  - 每次只挪一个域（例如先 `tui_render_buttons.cpp`），编译运行，验证行为不变。
- 第二阶段：把“详情/调试/布局”各域方法迁至相应 .cpp，删掉 tui_render.cpp 中重复实现。
- 第三阶段：清理无用工具函数；将内部复用的私有声明收敛至 internal 头。

迁移顺序建议（每步可回退）
1) `tui_render_buttons.cpp`：按钮 transform + 触发（含异步清理）。
2) `tui_render_keyboard.cpp`：键盘事件集中路由；修复 Tab/Enter 问题后稳定。
3) `tui_render_mouse.cpp`：鼠标路线与 Hover 管理；双栏命中函数只保留必要入口。
4) `tui_render_layout.cpp`：rebuild_dual_column_coordinate_mapping 与边界计算。
5) `tui_render_cells.cpp`：组/模块单元渲染；颜色/符号/行内布局统一化。
6) `tui_render_debug.cpp` / `tui_render_details.cpp`：调试窗口与详情区域。
7) `tui_render_component.cpp`：create_component 净化为“拼装页面骨架 + 调用子模块渲染”。

回归点与验收
- 键盘：Tab/Shift+Tab 在 左栏 → 右栏 → 按键面板 三者循环；左右切换按钮；Enter 行为正确；Space 在左栏批量 toggle。
- 鼠标：Hover/点击命中、滚轮滚动；按钮悬停高亮包含括号。
- UI：按钮 [ ] 内宽固定（当前 6），文本居中；运行/成功/失败配色与动画保持。
- 清理：按钮触发清理、EXIT_CODE 解析成功；失败详情自动换行展示。

涉及文件（最终形态示例）
- include
  - tui_render.hpp（保留）
  - tui_render_internal.hpp（仅实现用，若需要）
- src
  - tui_render_component.cpp
  - tui_render_buttons.cpp
  - tui_render_keyboard.cpp
  - tui_render_mouse.cpp
  - tui_render_layout.cpp
  - tui_render_cells.cpp
  - tui_render_details.cpp
  - tui_render_debug.cpp
  - tui_render_util.cpp（可选）

后续演进（可选）
- 将调试/按键指南抽象为独立小组件，进一步隔离样式与数据源。
- 把“按钮面板”做成一个小的 FTXUI Container，支持键盘在内部自然路由（目前已手动路由）。
- 把“清理/构建”后台调用封装到一个小的 runner（独立于渲染层），渲染层只消费状态。

备注
- 拆分为“多 .cpp 共享同一类”的做法是 C++ 常见实践，保持 UIRenderer 的单一对外入口同时显著降低单文件复杂度。
- 迁移过程每步都能单独编译运行，减少合并风险。


