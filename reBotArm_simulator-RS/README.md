# reBot Arm B601-RS 网页控制台

该目录提供 B601-RS 的 Three.js 网页模型、ROS 2 控制面板和可选的
MotorBridge 高级控制界面。它与仓库中的 `rebotarm_ros2_RS` 使用同一份 RS
URDF/STL；目录内的 `description` 只作为网页被单独复制时的离线后备。

## 当前能力

| 能力 | 状态 |
| --- | --- |
| RS Three.js/URDF/STL | 可用 |
| 可切换腕部相机 URDF | 无相机 / RealSense D405 / RealSense D435i / Orbbec Gemini 2 |
| RS ROS 2 Fake Driver | 可用，命名空间 `/rebotarm_rs` |
| RS 真机 ROS 2 Controller | 可用，命名空间 `/rebotarm` |
| 新版 `JointPosVelCmd`/`JointMitCmd` | 已适配 |
| RS 夹爪宽度与电机角度换算 | 已适配，71.5 mm ↔ 5 rad |
| RS MuJoCo 动力学 | 可用，启动脚本默认打开 Viewer 与 physics 模式 |
| 从零点执行当前姿态 | 可用，默认回零与目标两段各 2 秒 |
| 目标鬼影生命周期 | 到位或反馈等待超时后自动隐藏 |
| 三色物体抓取环境 | 可用，含俯视相机、检测、IK 与物理抬升验证 |
| MCP/Text 抓取 Agent | 可用，MCP 默认 `http://127.0.0.1:8081/mcp` |
| MotorBridge 直连 | 保留；不能替代物理急停和后端安全层 |

## 启动网页

```bash
cd reBotArm_simulator-RS
npm start
```

浏览器访问 `http://localhost:3002`。

控制台顶部的“腕部相机模型”可以切换三种相机及其支架，默认无相机；刷新后恢复上次
成功选择的型号。切换保留关节姿态、夹爪和示教状态，相机随腕部运动，目标残影同步显示
对应装配。加载失败时保留原装配，可以重新选择重试。

安装位置来自 `xiehuangbao888/rebot_visual_grasp` 的三份装配 Xacro；相机本体和光学
坐标系使用 RealSense / Orbbec 描述包，资源已随项目保存，无需在浏览器运行 ROS 或 Xacro。
这个选择器更改网页模型；ROS 相机图像话题仍由已有连接配置决定。
来源、安装参数和 URDF 重新生成方法见
[腕部相机资源说明](public/models/wrist-cameras/README.md)。

## GitHub Pages

GitHub Pages 地址：

```text
https://yang-ci.github.io/ReBot_Arm_DigitalTwin_RS/rs-console/
```

Pages 构建使用 `npm run build:pages -- ../web_mujoco/dist/rs-console`，会
复用当前仓库中的 ROS URDF、RS 手臂 STL 和夹爪 STL，输出纯静态页面；现有
`web_mujoco` 根页面保持不变。也可在本地只生成 RS 静态站点：

```bash
cd reBotArm_simulator-RS
npm run build:pages
```

注意：GitHub Pages 是 HTTPS，浏览器会阻止页面连接明文 `ws://<机器人IP>:9090`。
从 Pages 使用真机推动示教时，rosbridge 需要通过 `wss://` 提供；本地
`npm start` 页面则可以直接使用 `ws://`。AI 助手的 Node 代理同样只在本地服务
中可用，Pages 上模型、ROS 面板和示教导入导出功能不受影响。

## 启动安全仿真

先在仓库根目录构建 ROS 2 工作区：

```bash
./scripts/setup_rs_workspace.sh
```

然后启动 RS Fake Driver、MuJoCo 动力学 Viewer、抓取 Agent 和 rosbridge：

```bash
./scripts/start_rs_sim.sh
```

网页填写 `ws://<Ubuntu-IP>:9090`，选择“RS 仿真（/rebotarm_rs）”。收到
`fake_rs_pos_vel` 状态后，网页会自动允许仿真控制。

网页自然语言助手另开终端启动：

```bash
export DASHSCOPE_API_KEY="你的 Key"
./scripts/start_rs_text_agent.sh
```

## RS 真机

真机使用 SocketCAN `can0`，默认控制模式为 MIT。真机脚本设有显式保护：

```bash
export REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE
./scripts/start_rs_hardware.sh
```

同时启动真机、Fake Driver 和 rosbridge：

```bash
export REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE
./scripts/start_rs_dual.sh
```

执行前必须确认 `can0` 配置正确、机械臂工作区无人、物理急停可用。网页真机
模式不会自动打开控制锁。

### 推动真机示教

1. 启动 RS 真机 ROS 2 Controller 与 rosbridge，网页选择“RS 真机”。
2. 连接 rosbridge，收到 `/rebotarm/joint_states` 后打开控制锁。
3. 选择“完整路径（按原时间回放）”或“仅最终位置（3 秒到位）”，再点击
   “推动真机示教”。网页会请求进入重力补偿，成功后开始记录。
4. 用手轻推机械臂完成需要的轨迹，再点击“结束真机示教”。网页会先退出
   重力补偿，成功后自动执行安全回零。
5. 点击“回放”执行，或点击“导出 / 下载 JSON”保存；之后可用“导入 JSON”
   恢复同一条轨迹。

“完整路径”模式的数据来自 `/joint_states` 的原始 `position`，不做滤波、死区、抽稀或平滑；
时间优先使用 `header.stamp` 的整数纳秒，缺失时才回退到浏览器时钟。导出为
JSON，JavaScript number 与整数纳秒时间戳原样保留。回放同样使用原始 waypoint
和整数纳秒间隔，仅在正式轨迹前加入“当前姿态到示教起点”的安全引导段。
录制期间网页滑块、TCP 拖拽和回放命令会被拒绝，避免和人工推动互相抢控制权。
“仅最终位置”模式只保留结束示教前收到的最后一组关节位置；回放复用 IK
运动模块的平滑关节插值，从当前反馈姿态用固定 3 秒运动到该位置。

导入兼容 `rebotarm_ros_waypoints_v1` 与 `rebotarm_rs_teach_v1`。新格式会校验
关节名、点位数量、数值、时间单调性以及 `ros_stamp` 与 `time_from_start`
的一致性，避免导入被篡改或精度受损的数据。

## ROS 2 接口约定

- 关节位置速度：`/<namespace>/joints/<joint>/cmd/pos_vel`
- 关节 MIT：`/<namespace>/joints/<joint>/cmd/mit`
- 夹爪位置速度：`/<namespace>/gripper/cmd/pos_vel`
- 状态：`/<namespace>/joint_states`、`arm_status`、`gripper/state`
- 网页内部夹爪单位是米；ROS 2 夹爪电机位置单位是弧度。
