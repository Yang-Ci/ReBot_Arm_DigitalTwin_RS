# B601-DM 与 B601-RS 控制系统对比

本文以当前两个本地工程的实际源码和配置为准：

- DM：`/home/robot/ReBot_Arm_DigitalTwin_DM`
- RS：`/home/robot/ReBot_Arm_DigitalTwin_RS`

这里的“频率”必须分层理解。网页最多发送 60 Hz，并不代表电机只以 60 Hz 控制；RS
控制器会在收到新目标后，用自己的 125 Hz 实时循环持续生成并发送 MIT 指令。

## 1. 核心差异总览

| 项目 | DM 版本 | RS 版本 |
|---|---|---|
| 电机/总线 | 达妙电机，串口 CAN 桥，通常为 `/dev/ttyACM0` | 睿尔曼 RobStride，Linux SocketCAN，默认 `can0`、1 Mbps |
| 机械臂默认控制 | `POS_VEL` | `MIT` |
| 驱动控制环 | 500 Hz | 125 Hz |
| 网页连续命令 | 约 22.2 Hz（45 ms 节流） | 最高 60 Hz（16.67 ms 节流） |
| 网页关节速度默认值 | 1.2 rad/s，可调 0.05–3.0 | 1.2 rad/s，可调 0.05–1.5 |
| 真机轨迹安全速度 | 主要由 DM 的 `vlim` 与轨迹时间决定 | 对关节路径按 0.60 rad/s 强制延长时长 |
| 在线目标平滑 | 主要依赖 POS_VEL 和目标保持 | 125 Hz 三阶限加加速度参考：限速度、限加速度、限 jerk |
| 真机网页动画 | 以命令镜像为主 | 真机反馈为实线模型，命令仅作为半透明目标影子 |
| 反馈降噪 | 没有 RS 这套完整网页反馈滤波链 | 自适应低通 + 死区 + 反馈插值；不是卡尔曼滤波 |
| TCP IK | 固定 DLS 阻尼 0.035 | 奇异点附近自适应 DLS 阻尼 0.018–0.075 |
| 安全回零 | 没有独立的完整 `SAFE_HOMING` 状态 | 独立安全回零状态、清理旧目标、到位校验、失败不失能 |
| 重力补偿 | MIT + 重力/积分补偿，并带末端静止锁定 | MIT + 重力前馈，目标持续跟随当前测量位置，启动/重启更保守 |
| MuJoCo 模型 | DM 专用 MJCF 与执行器参数 | RS 专用 MJCF、网格、场景与碰撞参数，不复用 DM 结构 |

## 2. 各运行状态下的控制模式

### 2.1 RS 真机状态机

| 状态 | 电机模式 | 更新频率 | 可接受的新命令 | 行为与速度约束 |
|---|---|---:|---|---|
| `IDLE` | MIT 保持 | 125 Hz | 低层目标、轨迹、重力补偿请求 | 保持当前位置；使能时不会先跳到零点 |
| `LOWLEVEL_STREAMING` | MIT | 125 Hz | 网页关节/TCP 连续目标 | 网页最高 60 Hz 改目标；控制器以 125 Hz 生成连续参考。网页 `vlim` 默认 1.2 rad/s |
| `TRAJ_RUNNING` | MIT | 125 Hz | 默认拒绝与轨迹冲突的低层命令 | `FollowJointTrajectory` 以 100 Hz 更新单调三次位置/速度参考；路径按最大 0.60 rad/s 延长 |
| `GRAVITY_COMP` | MIT + 重力力矩前馈 | 125 Hz | 拒绝轨迹、网页低层与夹爪命令 | `kp=2`、`kd=1`；每周期以缓存测量角为柔顺目标，可直接手动拖动 |
| `SAFE_HOMING` | MIT | 125 Hz | 拒绝外部运动命令 | 先以 3.0 rad/s 关闭夹爪，再执行机械臂安全回零；清理旧网页目标 |
| `DISABLED`（概念状态） | 电机失能 | 无主动控制环 | 运动命令会因硬件未使能而失败/无效 | 代码中的 `state_machine` 字段失能后回到 `IDLE`；是否使能要同时看 `ArmStatus.enabled`。只有在零点附近才直接失能，否则先安全回零 |
| `SET_ZERO` 过程 | 停止 MIT 循环后写零点 | 非连续状态 | 拒绝运动命令 | 完成后保持失能，必须人工重新使能 |

RS 的“网页 60 Hz”和“电机 125 Hz”之间不是简单重复上一帧。每个网页目标都会更新
在线轨迹的终点，125 Hz 循环继续从当前参考状态向新目标推进，所以 TCP 拖拽时可以连续
重定向，而不需要等待上一段动画结束。

### 2.2 DM 真机状态机

| 状态 | 电机模式 | 更新频率 | 主要行为 |
|---|---|---:|---|
| `IDLE` | 默认 POS_VEL 保持 | 500 Hz 管理/状态循环 | 等待低层命令或动作请求 |
| `LOWLEVEL_STREAMING` | POS_VEL | 网页约 22.2 Hz 发目标 | 每条消息携带位置与 `vlim`，主要由电机 POS_VEL 模式完成运动 |
| `TRAJ_RUNNING` | 由动作执行器管理 | 轨迹插值循环 | 根据轨迹点推进关节目标 |
| `GRAVITY_COMP` | MIT + 重力/积分补偿 | 500 Hz | `kp=7`、`kd=0.8`；使用动力学与末端速度判断拖动/静止锁定 |
| 失能/回零 | 没有 RS 当前这套独立 `SAFE_HOMING` 仲裁 | — | 逻辑相对直接，旧目标隔离和回零验证弱于 RS |

DM 的 SDK 默认 POS_VEL 速度上限中，关节 1–3 为 5 rad/s、关节 4–6 为 3 rad/s、
夹爪为 3 rad/s；网页为了真机安全，默认只发送 1.2 rad/s。不要把 SDK 极限值当成推荐
操作速度。

## 3. 控制与观测频率

| 环节 | DM | RS | 说明 |
|---|---:|---:|---|
| 真机主控制循环 | 500 Hz | 125 Hz | 最终发往电机的实时循环 |
| 网页连续命令上限 | 22.2 Hz | 60 Hz | 用户输入经过浏览器节流后的 ROS 命令率 |
| `FollowJointTrajectory` 参考更新 | 实现相关 | 100 Hz | RS 每 10 ms 更新连续位置和速度参考，底层为 125 Hz |
| 同步硬件反馈查询 | 实现相关 | 20 Hz | RS 查询 RobStride 状态并刷新缓存，不在实时环内查询 |
| ROS 关节状态发布 | 100 Hz | 60 Hz | RS 状态帧从缓存发布，不等同于 CAN 查询率 |
| 浏览器真机 `/joint_states` 接收 | 取决于旧网页 | 最高 12.5 Hz | RS rosbridge 订阅 `throttle_rate=80 ms` |
| 浏览器 MuJoCo 状态接收 | — | 最高 25 Hz | `throttle_rate=40 ms` |
| 浏览器绘制 | 通常显示器刷新率 | 通常显示器刷新率 | `requestAnimationFrame`，常见为 60 Hz |
| Fake Driver 更新/状态 | — | 100 Hz | 关节最大 1.0 rad/s，夹爪电机最大 2.0 rad/s |
| MuJoCo 物理同步 | DM 专用配置 | 250 Hz | RS 默认动力学模式 |
| MuJoCo 物体状态 | — | 30 Hz | Agent 与网页场景使用 |
| 俯视相机 | — | 8 Hz | 图像流本身不需要 60 Hz |
| 颜色检测 | — | 10 Hz | 输出目标检测结果 |

网页收到真机反馈的频率低于 Three.js 绘制频率，因此 RS 在相邻测量值之间使用约
32–120 ms 的显示插值。这只影响画面，不改变发给电机的目标，也不会把预测姿态当作
真机姿态。

## 4. RS 在线平滑的实际算法

### 4.1 命令侧：三阶限 jerk 参考

RS 当前不是五次多项式 Minimum Jerk，也不是预先生成一整条固定 S 曲线。它是可在线
重定向的临界阻尼二阶吸引器，并在外层增加加速度和 jerk 限制：

```text
期望加速度 = ω² × (目标位置 - 参考位置) - 2ω × 参考速度
加速度      = clamp(期望加速度, ±4 rad/s²)
加速度变化  = clamp(本次变化, ±30 rad/s³ × dt)
参考速度    = clamp(参考速度 + 加速度 × dt, ±vlim)
参考位置    = 参考位置 + 参考速度 × dt
```

当前 `ω=8 rad/s`，积分子步最大 5 ms，长时间调度停顿按最多 100 ms 处理。优势是网页每
16.67 ms 改一次目标时无需重启整段轨迹；代价是它不是严格规定总时长的 S 曲线。

### 4.2 反馈侧：先降噪，再插值显示

RS 网页使用类似 One Euro 的自适应低通，而非卡尔曼滤波：

1. 先以 1 Hz 截止频率估计关节速度；
2. 基础位置截止频率为 3 Hz；
3. 运动越快，按 `beta=4` 提高截止频率，减少快速运动的延迟；
4. 小于 0.0025 rad（约 0.143°）的关节变化作为反馈死区；
5. 夹爪使用 0.25 mm 的反馈死区；
6. 最后在浏览器绘制帧之间做 32–120 ms 插值。

这条链路专门处理编码器量化、CAN/ROS 到达间隔和网页视觉抖动。若仍看到小幅“过冲再
纠正”，应先记录原始电机反馈、滤波后反馈和参考目标：原始反馈也过冲属于真机闭环；
只有网页模型过冲通常属于显示滤波/插值。

### 4.3 网页滑块阻尼

关节 1–6 默认阻尼为 30 ms，可在网页设为 0–300 ms。每个绘制帧使用：

```text
alpha = 1 - exp(-dt / damping_ms)
输出 = 输出 + alpha × (手指目标 - 输出)
```

它让用户拖六轴滑块时不至于每一个像素都立刻变成关节命令。它不是电机阻尼，也不改变
MIT 的 `kd`。六轴关节还带 1° 的输入死区；松手后最迟在
`max(120 ms, 4 × damping)` 内提交最终位置。

J7 夹爪不使用这条弧度阻尼链。网页以 0–71.5 mm 开口宽度表达目标，滑块不再量化为
1 mm 台阶；每个浏览器绘制帧只发送最新输入，松手时立即提交最终宽度；仅在发布时换算
成 0–5 rad 的夹爪电机目标。它没有输入死区，但保留合法行程夹紧。电机侧仍由 125 Hz MIT 循环
执行，最大目标变化速度为 5 rad/s。

## 5. MIT、POS_VEL 与重力前馈

### 5.1 为什么 RS 使用 MIT

RS 网页发送 `JointMitCmd`，但消息里的目标速度在控制器中被解释为本次在线目标的最大
移动速度。真正逐周期发送的 `q、dq、kp、kd、tau` 由 125 Hz 控制器统一生成。当前增益：

```yaml
kp: [80, 150, 150, 50, 50, 50]
kd: [5, 10, 10, 5, 4, 4]
```

关节 1 从早期 RS 配置的 `50/3` 提高到 `80/5`，主要用于克服基座关节静摩擦和减少
低速跟随滞后。继续增大增益可能带来振动、电流和温升，不应只为了让网页动画更快而调高。

### 5.2 正常运动有没有重力前馈

- RS 正常网页/轨迹运动：MIT 位置速度闭环，并用 20 Hz 刷新的测量缓存计算模型重力前馈；
  实时发送循环不会同步读取 CAN 参数。
- RS 重力补偿状态：叠加 Pinocchio 计算的重力力矩前馈。
- MuJoCo 动力学：使用 `qfrc_bias` 与 PD 控制，这是仿真内部的重力/偏置补偿。

因此，若普通关节运动存在持续静差，应先看负载、摩擦、MIT 增益和电机反馈，不要误以为
网页低通能解决力矩不足。

## 6. TCP 拖拽与 DLS

DM 网页 IK 使用固定 DLS 阻尼 0.035。RS 保留 DLS，并按接近奇异点的程度在
0.018–0.075 之间自适应调整；增益为 12，浏览器求解器的关节速度上限为 2.8 rad/s，
末端误差容差 1.5 mm。

真机拖拽的实际数据路径是“网页求 IK → 连续发送各关节 MIT 目标”，不是浏览器直接给
电机发笛卡尔位置。`/mujoco/target_pose` 主要作为目标/任务侧话题；当前 RS 拖拽能够跟随
真机，依赖的是求解后的关节流。这样做也使奇异点限速、关节限位和 125 Hz 在线平滑都在
进入电机前生效。

## 7. 夹爪单位和对称性

真机控制正常但网页模型看起来超行程，根因通常是把三个不同量当成同一单位：

| 量 | 范围 | 用途 |
|---|---:|---|
| 网页/任务夹爪开口宽度 | 0–0.0715 m | 用户命令语义 |
| RS 夹爪电机角度 | 0–5 rad | 真机 SDK 命令 |
| ROS 单指状态 | 0–0.045 m | 状态发布器把 0–5 rad 映射成每指最大 45 mm |
| 网页单指几何归一化行程 | 0–0.05 m | Three.js 对两指使用相同视觉上限，避免原 URDF 两侧上限不一致 |

ROS 状态发布器会把 0–5 rad 的夹爪电机反馈换算成两个手指关节各自的线位移。两个关节
名称保留为 `gripper_joint1`、`gripper_joint2`；左右对称由 URDF 的轴、mimic/几何方向和
映射共同定义，而不是必须把其中一个 ROS 数值写成负数。

## 8. RS 相对 DM 已完成的优化

1. 使用 RS 专用 URDF、MJCF、网格、碰撞体和较薄桌面场景；网页可勾选是否显示场景。
2. 从 DM 串口 CAN/POS_VEL 改为 SocketCAN/MIT，并建立 RS 专用消息接口。
3. 将网页目标率提高到 60 Hz，同时用 125 Hz 在线限 jerk 参考隔离网络抖动。
4. 增加自适应反馈低通、死区、显示插值和“真实姿态/目标影子”分离。
5. 修正夹爪命令宽度、电机角度和视觉行程的换算，避免超行程与左右不对称。
6. TCP 拖拽使用自适应 DLS，靠近奇异点时自动加大阻尼。
7. 动作轨迹按 0.60 rad/s 自动扩展时间，防止短时长请求导致真机突动。
8. 增加完整安全回零仲裁：先清旧目标、夹爪保守闭合、机械臂回零、验证后才失能。
9. 重力补偿支持状态查询、幂等二次启动和非零位启动，切模式时先锁住测量位置。
10. 真机启动脚本能识别上一次运行或被 `Ctrl+Z` 暂停的控制器，安全终止并清理 Fast DDS
    共享内存后再启动。
11. MuJoCo 增加 250 Hz 动力学、物体状态、俯视相机、颜色检测、抓取任务和 MCP Agent。
12. 将阻塞式 RobStride 参数查询移出实时 MIT 环：20 Hz 刷新测量缓存，60 Hz 发布状态，
    125 Hz 只做轨迹推进、缓存重力前馈和命令发送；滑块命令 QoS 只保留最新一条。

## 9. 当前没有采用的算法

- 没有卡尔曼滤波：当前为自适应一阶低通。
- 没有严格五次 Minimum Jerk：在线流使用限 jerk 临界阻尼参考；仿真假轨迹使用三次
  smoothstep `3u²-2u³`。
- 没有把 DLS 放在底层电机控制器中：DLS 只用于网页 TCP IK。
- 普通真机位置运动也有缓存模型重力前馈；它与低刚度、目标随反馈移动的
  `GRAVITY_COMP` 示教状态是两回事。

这些不是遗漏的同义词。只有在采集日志证明现有方法无法满足误差、延迟或噪声指标后，
才应该引入状态估计器或更复杂的轨迹规划器。

## 10. 参数来源

RS 主要来源：

- `rebotarm_ros2_RS/src/rebotarm_bringup/config/rebotarm_hardware.yaml`
- `rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`
- `rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/motion_profiles.py`
- `rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py`
- `rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py`
- `rebotarm_ros2_RS/src/rebotarm_mujoco_rs/rebotarm_mujoco_rs/mujoco_sync.py`
- `rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/fake_rs_driver.py`
- `reBotArm_simulator-RS/public/js/ros/rebot-ros-ui.js`
- `reBotArm_simulator-RS/public/js/rebot-sim.js`

DM 主要来源：

- `/home/robot/ReBot_Arm_DigitalTwin_DM/reBotArmController_ROS2-main/src/rebotarm_bringup/config/rebotarm_hardware.yaml`
- `/home/robot/ReBot_Arm_DigitalTwin_DM/reBotArm_simulator-DM/public/js/ros/rebot-ros-ui.js`
- `/home/robot/ReBot_Arm_DigitalTwin_DM/DATA_FLOW_ZH.md`
- `/home/robot/ReBot_Arm_DigitalTwin_DM/PROJECT_ARCHITECTURE_ZH.md`
