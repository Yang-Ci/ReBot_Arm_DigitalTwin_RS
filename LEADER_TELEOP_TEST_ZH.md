# 2026-10-08 公司测试：Arm102 Leader → RS ROS 控制台

分支：`feat/rs-console-leader-teleop`。命令在公司 Ubuntu 的仓库根目录执行。

## 1. 安装与重建

停止旧 RS controller / 仿真 / rosbridge 后执行：

```bash
bash scripts/setup_rs_leader.sh
```

脚本复用 ROS .venv，安装 UART SDK/pyserial，重建 rebotarm_msgs、rebotarmcontroller。
如果 Ubuntu 从未搭建工作区，先运行 `bash scripts/setup_rs_workspace.sh`，再运行上面命令。
主臂 USB 必须连接 ROS 主机；ROS 在虚拟机时要透传 USB。网页电脑的 COM 口不能代替 ROS 主机串口。

新终端检查：

```bash
source scripts/rs_env.sh
python -c 'from motorbridge_smart_servo import FashionStarServo; from serial.tools import list_ports; print([(p.device,p.description) for p in list_ports.comports()])'
ros2 interface show rebotarm_msgs/srv/LeaderControl
```

支持 /dev/ttyUSB0、/dev/ttyACM0、/dev/serial/by-id/...。权限不足时加入 dialout，再重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

## 2. 无硬件流程（可选）

终端一：

```bash
REBOTARM_MUJOCO_VIEWER=false ./rebotarm start rs_sim
```

终端二：

```bash
./rebotarm start web
```

网页选 ROS / RS 仿真，连接 rosbridge，扫描后选 mock。连接、解锁、确认零位、校准、开始，观察主臂角度与从臂反馈，再测暂停、继续、停止。mock 仅在 FakeRsDriver 开放，真机拒绝 mock。

## 3. 真机启动

停止仿真，按现有方法配置 can0 并检查从臂状态。终端一：

```bash
REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE ./rebotarm start rs
```

终端二：

```bash
./rebotarm start web
```

网页选 ROS / RS 真机（/rebotarm），连接、打开 ROS 控制锁，确保从臂已使能、IDLE。夹爪若卸力/辅助，先恢复位置保持。关闭手势跟随；若先前有滑块/轨迹动作，先停止，再用现有“使能”恢复空闲保持。
主臂/从臂留出运动空间，不要同时运行直接控制同一 can0 的 LeRobot 遥操作。

## 4. 首轮测试

1. Leader 遥操作卡片留空串口，点“扫描主臂”；完整设备应响应 ID 0–6。选中后连接，也可手动填 /dev/serial/by-id/...。
2. 连接后卡片自动显示官方 Arm102 / 102-LD 模型、七项角度与采样率，此时从臂保持。模型可拖动旋转、滚轮缩放，手机支持双指缩放；“重置视角”只调整显示视角。
3. 扶稳主臂，点“解锁主臂”，按 [Wiki](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/) 的 Arm102 校准姿态摆好零位、闭合夹爪。勾选确认，再点“确认并设置主臂零位”。这是主臂校准，不要用 RS 从臂电机设零代替。先逐轴小幅移动主臂，对照实物核对模型的零位、J1–J6 方向及 J7 指环开合；模型按官方 LD 显示映射，手柄比例为 1，实际方向和零位仍需这一步验收。
4. 速度设 **0.1 rad/s**，取消“跟随夹爪”，保持“绝对跟随”未勾选。
5. 点“开始跟随”，首帧从臂不应跳到主臂绝对姿态。从小幅 J6 开始，再逐一小幅测 J1–J5 的方向、反馈、限位。
6. 暂停后移动主臂，从臂不应跟随。继续应以此时两臂重新对齐，不追赶暂停期间的偏移。
7. 停止并释放控制后测试原滑块。六轴确认后再开启夹爪跟随，小幅验证 J7 方向和端点。
8. 测网页断线/拔主臂 USB：应停止目标推进并报告故障；模型冻结最后有效姿态并提示样本过期。重连不能自动跟随，须明确恢复、校准、开始；主动断开主臂后模型隐藏。

暂停/停止是位置保持。现有卸力包含安全回零，遥操作占用期间会被拒绝；先停止并释放控制，再使用卸力。

## 5. 排障

| 现象 | 检查 |
|---|---|
| 等待 ROS 主臂服务 | 接口是否编译；controller/rosbridge 是否重启；命名空间；强制刷新网页 |
| 没有串口 | USB 主机/透传、供电、dialout；手动填端口；清空旧端口再扫全部 |
| 模型加载失败 | 强制刷新，检查 public/models/leader-arm102/urdf/leader.urdf 与 meshes 中九个 STL 是否部署完整；模型使用本地资源 |
| 模型方向/零位与实物不同 | 对照官方校准姿态重新校准，逐轴记录差异；不要修改从臂映射来修正模型显示 |
| 缺 ID | 应有 0–6 七个舵机；供电、总线接线、1 Mbps |
| probe timed out | 权限、其他程序占用、SDK 缺失、设备不响应 |
| uncalibrated / samples stale | 重新预览、解锁、校准；观察 Hz/延迟，不自动放宽超时 |
| follower is busy | 停轨迹/手势/重力补偿，恢复 IDLE；夹爪恢复位置保持 |
| absolute poses differ | 用默认相对模式；绝对模式先对齐 |
| reliable=False / servo missing | 本次未响应，缓存角度被拒绝；检查 USB/总线 |
| browser heartbeat timed out | 页面关闭、连接断开、后台节流；显式校准开始 |
| another page owns session | 原页面停止；关闭该页后等心跳超时释放 |

查看真实状态：

```bash
source scripts/rs_env.sh
ros2 topic echo /rebotarm/leader/status
```

记录七项角度、采样 Hz/延迟、方向、夹爪端点、暂停/拔 USB/网页断线时延及终端报错。本地覆盖 mock 和控制策略，真实硬件尚未验收。
