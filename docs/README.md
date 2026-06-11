# Sunray 二次开发手册

这个目录只包含文档文件，不修改工程源码。

- 推荐入口：打开 `docs/index.html`
- 正文文件：`docs/content/*.md`
- 样式文件：`docs/assets/style.css`
- 渲染脚本：`docs/assets/app.js`

手册面向“有一点 ROS 基础、没有 PX4 经验”的学生，重点解释 Sunray 的二次开发路径：

1. 先理解仓库结构和启动链路。
2. 再从 `sunray_tutorial` 示例复制并修改任务逻辑。
3. 通过 `sunray_msgs` 的控制话题接入 UAV/UGV 控制节点。
4. 需要定位、规划、视觉、编队、通信时，再进入对应模块。

文档参考了仓库当前结构编写，并对以下包做了较深入说明：

- `General_Module/sunray_common`
- `General_Module/sunray_uav_control`
- `General_Module/sunray_tutorial`
- `General_Module/sunray_planner_utils`
- `External_Module/ego-planner-swarm`

同时整理了 `scripts_sim`、`scripts_exp`、`scripts_swarm`、`server` 和 `tests/production` 中的快速启动方式。这些脚本既可以手动运行，也可以作为地面站快速启动的命令模板。

## 维护说明

正文最多拆成 8 个 Markdown 文件，位于 `docs/content/`。`docs/index.html` 仍是入口页，浏览器会用本地 `marked` 库解析并渲染 Markdown。

为了支持直接双击 `index.html` 打开，`docs/assets/doc-data.js` 保存了一份 Markdown 内容镜像。修改 `docs/content/*.md` 后，需要同步更新 `doc-data.js`，否则双击打开时仍会显示旧内容；通过静态服务打开时会优先读取相对路径下的 Markdown 文件。
