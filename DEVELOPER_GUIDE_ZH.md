# reBot Arm B601-RS 完整开发者手册

## 1. 适用范围与安全边界

本工程同时包含网页、ROS 2、RS 真机驱动、MuJoCo、视觉和抓取 Agent。仿真与真机使用
不同命名空间：仿真默认 `/rebotarm_rs`，真机默认 `/rebotarm`。

真机操作前必须满足：急停可触达、机械臂工作空间无人、CAN 连接稳定、机械零位已标定、
负载已确认。第一次验证请降低网页 `vlim`，并一次只移动一个关节。`set_zero` 会修改电机
零点，不是普通“回零”按钮，只有机械结构确实放在标定姿态时才能执行。

## 2. 支持环境

- Ubuntu 24.04 + ROS 2 Jazzy（当前主要验证环境）；
- Ubuntu 22.04 + ROS 2 Humble（脚本支持，需自行完成真机回归）；
- Python 3.12 或 3.10；
- Node.js 18 及以上；
- RS 真机使用 SocketCAN `can0`，比特率 1 Mbps；
- 浏览器建议 Chrome/Chromium 或 Firefox 的当前稳定版。

## 3. 工程结构

```text
ReBot_Arm_DigitalTwin_RS/
├── README.md                         # 快速开始和入口索引
├── DEVELOPER_GUIDE_ZH.md             # 本开发手册
├── DM_RS_COMPARISON_ZH.md            # DM/RS 模式、速度与优化对比
├── DATA_FLOW_RS_ZH.md                # RS 端到端数据流
├── setup.sh                          # 一键依赖安装、拉取资源、构建
├── rebotarm                           # 统一启动/检查入口
├── requirements-rs-hardware.txt      # RS SDK Python 依赖
├── requirements-rs-mujoco.txt        # MuJoCo/Agent Python 依赖
├── scripts/
│   ├── setup_rs_workspace.sh         # 固定上游版本、打补丁、colcon 构建
│   ├── rs_env.sh                     # 统一加载 ROS、venv、workspace
│   ├── start_rs_sim.sh               # Fake Driver + MuJoCo + Agent + rosbridge
│   ├── start_rs_hardware.sh          # 真机独占、安全替换旧实例
│   ├── start_rs_dual.sh              # 真机/仿真双命名空间开发
│   └── start_rs_text_agent.sh        # 自然语言 Agent HTTP 服务
├── rebotarm_ros2_RS/
│   ├── src/rebotarmcontroller/        # 真机驱动、状态机、动作、Fake Driver
│   ├── src/rebotarm_msgs/             # 自定义 msg/srv/action
│   ├── src/rebotarm_bringup/          # 配置、URDF、launch、RViz
│   ├── src/rebotarm_mujoco_rs/        # RS MuJoCo 同步、场景、相机和任务
│   ├── src/rebotarm_agent/            # MCP 抓取工具与文本 Agent
│   ├── src/rebotarm_moveit_config/    # MoveIt 配置
│   └── third_party/                   # setup 自动拉取，不作为主仓库源码维护
├── reBotArm_simulator-RS/             # Three.js 网页、rosbridge 与代理服务器
├── patches/                            # 对固定上游 SDK 的可复现补丁
└── vendor_overrides/                   # RS MJCF/网格的本仓库覆盖文件
```

## 4. 一键安装

```bash
cd /home/robot/ReBot_Arm_DigitalTwin_RS
./setup.sh --check        # 只检查，不改系统
./setup.sh --yes          # 安装缺失依赖、拉取固定资源、构建
./rebotarm doctor         # 安装后的复查
```

安装脚本不会删除或重置已存在的用户配置和 third-party 工作树；缺失时才拉取，但仍会
检查并应用本工程经过验证的补丁和资源覆盖。主要动作：

1. 检查 Ubuntu、Python、Node.js、SocketCAN 工具和 ROS 2；
2. 缺少 ROS 软件源时安装官方 `ros2-apt-source`；
3. 安装 ROS desktop、rosbridge、MoveIt、构建工具和 `can-utils`；
4. 创建 `rebotarm_ros2_RS/.venv`；
5. 拉取并固定 `reBotArm_control_py` 和 RS MuJoCo 上游提交；
6. 应用 `patches/` 与 `vendor_overrides/`；
7. 通过 rosdep 安装包依赖并执行 `colcon build --symlink-install`；
8. 创建网页 `.env`（仅当不存在），检查 Node 环境。

仅重建 ROS 工作区可运行：

```bash
./scripts/setup_rs_workspace.sh
```

`setup_rs_workspace.sh` 是 `setup.sh` 调用的下层构建脚本。它假定 ROS 2 已经安装好，只负责：

- 创建 `rebotarm_ros2_RS/.venv` 并安装 RS 真机、MuJoCo 和 Agent 的 Python 依赖；
- 拉取固定版本的 RS 控制 SDK 与 RS MuJoCo 模型；
- 应用 `patches/` 和 `vendor_overrides/`；
- 执行 rosdep 和 `colcon build --symlink-install`。

新机器应优先运行根目录的 `./setup.sh --yes`，因为它还会安装 ROS、Node、SocketCAN 等
系统依赖。只有系统依赖已经齐全，需要重新生成/构建 ROS 工作区时，才直接运行
`setup_rs_workspace.sh`。

修改 Python 包以后通常只需：

```bash
source scripts/rs_env.sh
cd rebotarm_ros2_RS
colcon build --symlink-install
```

## 5. 启动仿真

一条命令启动 Fake Driver、MuJoCo 窗口、物体场景、相机、检测、MCP Agent 和 rosbridge：

```bash
./rebotarm start rs_sim
```

另开终端启动网页：

```bash
./rebotarm start web
```

打开 `http://localhost:3002`，选择 `/rebotarm_rs`。

常用仿真变量：

```bash
# 无图形窗口，适合自动测试
REBOTARM_MUJOCO_VIEWER=false ./rebotarm start rs_sim

# 只验证状态同步，不验证动力学
REBOTARM_MUJOCO_MODE=kinematic ./rebotarm start rs_sim

# 不启动 MCP Agent
REBOTARM_START_AGENT=false ./rebotarm start rs_sim

# 清理陈旧仿真进程再启动
./rebotarm start rs_sim --force
```

`physics` 是默认模式。它验证重力、接触和抓取；`kinematic` 直接跟随关节位置，不能用于
判断控制增益、抓取力或碰撞稳定性。

## 6. 启动真机

### 6.1 配置 SocketCAN

确认 USB-CAN 设备已识别后：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
candump can0
```

`candump` 用于只读观察总线；确认有帧后按 `Ctrl+C` 退出。若接口进入 `BUS-OFF`，先检查
两端终端电阻、波特率、接地和供电，不要靠重复重启控制器掩盖硬件问题。

### 6.2 启动控制器

```bash
cd /home/robot/ReBot_Arm_DigitalTwin_RS
REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE \
  ./rebotarm start rs
```

另开终端运行网页，然后选择 `/rebotarm`：

```bash
./rebotarm start web
```

真机控制器必须用 `Ctrl+C` 结束。若误用 `Ctrl+Z`，再次运行同一启动命令会恢复旧进程、
请求安全退出、清理僵尸 Fast DDS 共享内存，再启动新实例。默认允许旧实例 35 秒完成回零；
可通过 `REBOTARM_REPLACE_TIMEOUT` 修改。脚本只清理确认属于旧控制器的资源，不会批量
终止无关 ROS 进程。

## 7. 真机操作顺序

推荐顺序：

1. 启动驱动，检查网页状态为已连接、无电机错误；
2. 先用 0.2–0.4 rad/s 小范围测试各关节方向；
3. 再启用关节滑块或 TCP 拖拽；
4. 重力补偿前托住可能下坠的负载；
5. 退出时点击安全回零，再失能；也可勾选断开连接时回零再失能；
6. 最后用 `Ctrl+C` 终止驱动。

点击失能时，如果机械臂不在零点附近，控制器会先进入 `SAFE_HOMING`。回零验证失败时
电机保持使能，并向网页返回失败，避免机械臂在非零姿态突然掉落。

## 8. 统一命令行

```bash
./rebotarm doctor
./rebotarm start web
./rebotarm start rs_sim [--force]
REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE ./rebotarm start rs
./rebotarm status
./rebotarm stop
```

`stop` 只停止由 `start web` 登记的后台进程。前台仿真和真机进程使用其终端中的
`Ctrl+C` 安全结束；尤其不要对真机直接 `kill -9`，除非正常安全退出已经失败。

### `start_rs_dual.sh` 的用途

这是开发调试工具，会在同一个 ROS Domain 中同时启动：

- RS 真机驱动，命名空间 `/rebotarm`；
- RS Fake Driver，命名空间 `/rebotarm_rs`；
- 一个 rosbridge，默认端口 9090。

它用于对比同一网页/ROS 客户端在真机和 Fake Driver 上的接口、方向和状态差异。它不会
启动网页服务器，也不会启动完整 MuJoCo 动力学、视觉或 Agent，因此不能替代
`./rebotarm start rs_sim`。普通真机操作不需要使用 dual；同时连真机时仍要求硬件确认变量，
并且必须在网页中确认选中的命名空间，避免把测试命令发到真机。

## 9. ROS 接口

以下用真机命名空间 `/rebotarm` 表示；仿真时替换为 `/rebotarm_rs`。

### 9.1 状态话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/rebotarm/joint_states` | `sensor_msgs/msg/JointState` | 六轴和两个手指的统一状态 |
| `/rebotarm/joints/<name>/state` | `rebotarm_msgs/msg/JointMotorState` | 单电机位置、速度、力矩、状态码 |
| `/rebotarm/gripper/state` | `JointMotorState` | 夹爪电机原始角度状态 |
| `/rebotarm/control_target` | `JointState` | 最终控制目标 |
| `/rebotarm/control_reference` | `JointState` | 在线平滑参考；`effort` 携带参考加速度 |
| `/rebotarm/arm_status` | `rebotarm_msgs/msg/ArmStatus` | 模式、使能、状态机和错误码，TRANSIENT_LOCAL |

### 9.2 低层命令

```text
/rebotarm/joints/<joint1..joint6>/cmd/mit      rebotarm_msgs/msg/JointMitCmd
/rebotarm/joints/<joint1..joint6>/cmd/pos_vel  rebotarm_msgs/msg/JointPosVelCmd
/rebotarm/gripper/cmd/mit                       rebotarm_msgs/msg/JointMitCmd
/rebotarm/gripper/cmd/pos_vel                   rebotarm_msgs/msg/JointPosVelCmd
```

网页真机模式默认使用 MIT。需要自行发布时，先检查 `arm_status.state_machine`；不要在
`GRAVITY_COMP`、`SAFE_HOMING` 或运行中轨迹上强行抢占。

### 9.3 服务

```text
/rebotarm/enable
/rebotarm/disable
/rebotarm/safe_home
/rebotarm/set_zero
/rebotarm/gravity_compensation/start
/rebotarm/gravity_compensation/stop
/rebotarm/gravity_compensation/status
/rebotarm/move_to_pose_ik
/rebotarm/gripper/set
/rebotarm/gripper/open
/rebotarm/gripper/close
```

示例：

```bash
source scripts/rs_env.sh
ros2 service call /rebotarm/gravity_compensation/status std_srvs/srv/Trigger '{}'
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger '{}'
```

### 9.4 动作

```text
/rebotarm/follow_joint_trajectory  control_msgs/action/FollowJointTrajectory
/rebotarm/gripper/command          control_msgs/action/GripperCommand
/rebotarm/move_to_pose             rebotarm_msgs/action/MoveToPose
```

真机 `FollowJointTrajectory` 会自动把过短段延长到不超过 0.60 rad/s。调用端必须等待动作
结果或真实反馈，不能只按原始请求时长播放本地动画。

## 10. MuJoCo 与 Agent 接口

```text
/rebotarm_rs/mujoco/joint_states
/rebotarm_rs/mujoco/object_states
/rebotarm_rs/mujoco/overhead_rgb/image_raw
/rebotarm_rs/vision/color_blocks/detections
/rebotarm_rs/mujoco/reset
/rebotarm_rs/mujoco/record/start
/rebotarm_rs/mujoco/record/stop
/rebotarm_rs/mujoco/record/replay
/rebotarm_rs/mujoco/record/clear
```

MCP 默认地址：`http://127.0.0.1:8081/mcp`。不配置 LLM 也可以直接使用检测、IK、关节、
抓取工具。启用自然语言网页：

```bash
export DASHSCOPE_API_KEY='你的 Key'
export REBOTARM_LLM_MODEL='qwen-plus'
./scripts/start_rs_text_agent.sh
```

默认 Dashboard 为 `http://localhost:8082`。密钥只放环境变量或本地 `.env`，不要提交。

## 11. 关键参数与修改位置

### 真机控制

文件：`rebotarm_ros2_RS/src/rebotarm_bringup/config/rebotarm_hardware.yaml`

```yaml
rate: 125
control:
  arm_control_mode: mit
  mit_kp: [80, 150, 150, 50, 50, 50]
  mit_kd: [5, 10, 10, 5, 4, 4]
  stream_acceleration_limit: 4.0
  stream_jerk_limit: 30.0
  stream_natural_frequency: 8.0
gravity_compensation:
  kp: 2.0
  kd: 1.0
```

真机启动脚本另外设置 `joint_state_rate=60 Hz` 和
`hardware_feedback_poll_rate=20 Hz`。后者才是同步读取 RobStride 电机参数并刷新缓存的
频率；125 Hz MIT 实时循环从缓存取测量角计算重力前馈，不允许在实时路径里同步查询 CAN。
可分别用 `REBOTARM_JOINT_STATE_RATE`、`REBOTARM_HARDWARE_FEEDBACK_POLL_RATE` 临时覆盖。

先记录 `/control_target`、`/control_reference` 和 `/joint_states`，再改增益。一次只改一组
参数，并记录负载、关节、速度、电流/温度和结果。网页动画滞后不应该通过提高 MIT 增益解决。

### 网页

- 命令频率、反馈滤波、滑块阻尼：
  `reBotArm_simulator-RS/public/js/ros/rebot-ros-ui.js`
- TCP IK、自适应 DLS、场景和动画：
  `reBotArm_simulator-RS/public/js/rebot-sim.js`
- 默认 UI 数值：`reBotArm_simulator-RS/public/index.html`
- 服务器端模型参数：`reBotArm_simulator-RS/server.js`

### MuJoCo

- 同步频率与 PD：`rebotarm_ros2_RS/src/rebotarm_mujoco_rs/rebotarm_mujoco_rs/mujoco_sync.py`
- launch 默认参数：`rebotarm_ros2_RS/src/rebotarm_mujoco_rs/launch/mujoco_rs.launch.py`
- 任务/回放：`rebotarm_ros2_RS/src/rebotarm_mujoco_rs/rebotarm_mujoco_rs/task_server.py`
- MuJoCo 模型资源：`rebotarm_ros2_RS/src/rebotarm_mujoco_rs/models/`
- 控制 SDK 源码：`rebotarm_ros2_RS/third_party/reBotArm_control_py/`

这些内容由主仓库直接跟踪，不保留嵌套 `.git`。MuJoCo 只合入实际使用的模型 XML 和 STL，
不依赖上游算法包。修改 SDK、模型或网格后直接随主仓库提交。
