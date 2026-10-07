# RS 控制台 Leader 遥操作：ROS 集成说明

日期：2026-10-07。分支：`feat/rs-console-leader-teleop`。
基线：`b85fc7621ac85ad6dd1bc707d5ad069fb6173b22`；保留工作区既存改动。
状态：网页、ROS 服务处理与 mock 流程已实现；Ubuntu ROS 编译和真实 USB/CAN 验证待公司测试。

## 1. 集成路线

[Wiki](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/) 的 RS 遥操作使用 `seeed_b601_rs_follower` 与 `rebot_arm_102_leader`，实际实现来自 [Arm102 leader 插件](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102) 和 [B601 RS follower 插件](https://github.com/Seeed-Projects/lerobot-robot-seeed-b601)。
[Seeed-Projects/lerobot](https://github.com/Seeed-Projects/lerobot) 当前内置 `rebot_102_leader` 使用不同方向/夹爪表示，内置 `rebot_b601_follower` 是 DM 路径，不能与本次 Wiki RS 配对混用。

本次复用 Wiki 的 FashionStar UART SDK、零位处理和映射；从臂继续由已有 ROS controller 唯一持有 CAN，使用原有 125 Hz MIT 平滑和重力前馈。主臂 UART 在可终止的独立子进程运行，避免探测/读取阻塞 CAN 输出。
无需安装整套 LeRobot/PyTorch，只增加可选 `motorbridge-smart-servo==0.0.4`、`pyserial==3.5`。不能同时运行直接控制同一 can0 的 `lerobot-teleoperate`。

相较初期独立 ROS 包/目标话题方案，已实现版本将会话与整帧目标放在现有 controller 内，UART worker 负责隔离；不增加 ROS 节点启动命令。

## 2. 操作与映射

**连接 ROS → 扫描 → 连接预览 → 解锁主臂 → 摆好零位并闭合夹爪 → 确认零位 → 开始跟随。**

- 扫描 ROS 主机串口并探测 ID 0–6，报告完整/部分响应，支持手动端口。扫描与连接预览只读。
- 解锁只对主臂卸力，不设置零点、不移动从臂，便于摆放校准姿态。
- 校准须勾选确认，逐舵机调用 unlock、set_origin_point、reset_multi_turn；不会修改 RS 从臂零点。
- 默认相对跟随，开始/继续时锁存两臂当前位置；首帧保持从臂姿态。
- 绝对跟随初始六关节偏差超过 0.15 rad，或参与跟随的夹爪偏差超过 0.3 rad，拒绝启动。
- 暂停保持位置并保留会话/心跳；停止保持位置并释放控制；断开再关闭 UART。
- 校准仅限当前连接。故障/超时后不能自动继续；网页刷新/重连不接管旧会话。

`sync_monitor([0,1,2,3,4,5,6])` 返回度数；七项须都存在、有限且 reliable=True。SDK 返回缓存 reliable=False 不刷新有效采样时间。

| 舵机 ID | RS 目标 | 方向/比例 |
|---|---|---|
| 0、1 | joint1、joint2 | +1、+1 |
| 2、3、4 | joint3、joint4、joint5 | −1、−1、−1 |
| 5 | joint6 | +1 |
| 6 | gripper 电机角度 | +6 |

相对六轴目标：`q_start + radians(leader - leader_start) * direction`。
相对夹爪：`gripper_start + radians((leader_gripper - leader_gripper_start) * 6)`。
绝对模式省去初始偏移。限位取现有 Pinocchio RS URDF，夹爪端点取硬件配置（通常闭合 0、张开 5 rad）。夹爪电机角度不能与网页宽度/两指 joint_states 混用。
ROS 参数 leader_joint_directions 为六项 ±1，用于装配方向修正；目标夹紧到限位，每帧超过 25° 的跳变、缺项、NaN/Infinity、陈旧/倒序帧会拒绝。

## 3. 控制权与超时

启动需从臂已使能、MIT、IDLE，没有其他运动，夹爪处于位置保持。整帧目标在同一驱动锁内更新，速度 0.05–0.6 rad/s，默认 0.3；夹爪六倍换算，封顶 1.5 rad/s。
网页待发命令与回放在启动前取消，手势跟随须先关闭。活动会话后端拒绝其他单关节、夹爪、轨迹/IK、回零、卸力等写入；网页普通运动入口同步阻止，停止后恢复。错误页面 session 不能暂停/停止原页面。

| 检查 | 当前值/行为 |
|---|---|
| UART 目标采样频率 | 30 Hz，显示实际频率 |
| 有效帧年龄 / host 样本超时 | 200 ms |
| CAN 循环样本兜底超时 | 300 ms |
| 网页心跳 | 250 ms；超过 1 s 保持并结束会话 |
| 从臂缓存刷新检查 | 超过 500 ms 拒绝启动/更新 |
| ROS 状态发布 | 20 Hz；网页 100 ms 节流 |

暂停/故障清除旧 arm/gripper 流目标、速度和加速度，保持缓存当前位置。读错误立即结束活动跟随，worker 卡住由样本超时处理。控制锁关闭、切离 ROS 面板、页面退出请求停止，断线由后端心跳兜底。
从臂缓存刷新检查沿用已有 CAN SDK，不等于已验证每个电机包的真实年龄；公司需实测反馈有效性。

## 4. ROS 接口与代码

真机命名空间 `/rebotarm`，仿真 `/rebotarm_rs`。

- `/<ns>/leader/control`：LeaderControl，operation 为 scan/connect/unlock/calibrate/start/pause/resume/stop/disconnect/heartbeat。请求含 port、session_id、speed、follow_gripper、absolute、confirm_zero；响应含 success、message、session_id、devices。
- `/<ns>/leader/status`：LeaderStatus，含状态、原因、端口、会话、校准/跟随/暂停标记、样本年龄/频率、七项主臂角度和六轴目标。
- LeaderDevice：端口、描述、序列号、响应 ID、完整性、探测错误。

| 文件 | 用途 |
|---|---|
| leader_policy.py | 映射、样本校验、会话、watchdog |
| leader_worker.py、leader_teleop.py | 隔离 UART 与 ROS 生命周期 |
| hardware_manager.py | 整帧目标、占用、保持、实时超时检查 |
| motor_passthrough.py、ros_actions.py、ros_services.py | 旧入口仲裁与异常清理 |
| fake_leader_hardware.py、fake_rs_driver.py | 同一会话协议，mock 仅在仿真开放 |
| public/js/ros/rebot-leader-ui.js、css/leader-teleop.css | 卡片、状态、心跳、中英文 |
| index.html、rebot-ros-ui.js、service-worker.js | 入口、冲突阻止、缓存更新 |
| requirements-rs-leader.txt、scripts/setup_rs_leader.sh | 可选依赖、接口/controller 重建 |

controller 文件在 rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/；网页在 reBotArm_simulator-RS/public/。

## 5. 验证范围

15 项遥操作策略/集成测试通过，覆盖方向、相对首帧、夹爪比例、限位/跳变、SDK 缓存拒绝、错误/陈旧/重复帧、mock 识别、真实驱动整帧原子更新/旧入口拒绝、worker 被杀、暂停/心跳超时。原有平滑/轨迹 8 项与反馈缓存 10 项测试通过。官方 0.0.4 Windows wheel 导入/API 检查通过。

真实网页经本机 WebSocket 到同一 LeaderTeleop 实现（ROS Node/消息替身、mock UART），完成扫描、预览、解锁、校准、跟随、心跳、暂停、继续、停止、控制锁变更及网页断线后的保持；无页面 JS 异常，桌面/手机布局截图已检查。
当前 Windows 工作区没有编译 ROS、没有真机验收；电机方向、机械零位、实际采样频率和停止时延仍待公司验证。

安装、测试顺序和排障见 [LEADER_TELEOP_TEST_ZH.md](LEADER_TELEOP_TEST_ZH.md)。

## 6. Leader 模型预览

连接后在 Leader 卡片内显示 FashionStar 官方浏览器控制台的 **Star Arm 102-LD** 模型（九个 STL，含手柄和指环）。来源为 [官方控制台](https://fashionstar.com.hk/wiki/zh/software/robot-arm/data/102-web-controller/Browser_SDK/) 的 LD 模型目录；原始 URDF、逐文件 URL/哈希和来源说明保存在 `public/models/leader-arm102/`。模型资源约 5 MB，使用本地路径并加入 PWA 缓存。

`public/js/ros/leader-model.js` 复用 THREE / URDFLoader，消费已有 LeaderStatus 的七轴角度。显示使用官方 LD 方向映射（J4 反向），手柄倍率 1、左右指环相反运动，与从臂的夹爪 ×6 映射独立。适配 URDF 仅修正本地网格路径、按官方配置启用 J5、添加右指环 mimic。显示不截断 URDF 限位，不发送 ROS 指令。

连接并进入 ROS 面板时按需加载；读取到有效样本时更新姿态，样本超过 200 ms 或网页断线时冻结最后姿态并提示，主动断开后隐藏。支持拖动旋转、滚轮/双指缩放、方向键及重置视角。模型颜色为本地显示材质。

新增 `node --test scripts/leader-model.test.js` 四项检查，覆盖角度方向/手柄倍率、整圈归一化、无效输入拒绝、九个网格引用及来源哈希。浏览器检查九个网格全部加载、八个可动关节角度、断线冻结/断开隐藏、桌面与手机布局；本机 mock 全流程通过且无资源加载错误。官方映射也要求实物零位与方向核对，当前没有完成真机姿态验收。
