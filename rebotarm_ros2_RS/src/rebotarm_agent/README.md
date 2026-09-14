# reBotArm RS Agent

该包把 DM 版本的 MCP/Text Agent 接入 B601-RS 仿真接口。默认命名空间是
`/rebotarm_rs`，运动工具默认保持锁定；仓库的 `start_rs_sim.sh` 只在安全仿真中
使用 `motion_mode:=allow`。

## RS 适配

- Agent 对外继续使用夹爪开度 `0~90 mm`。
- ROS RS 驱动内部使用夹爪电机位置 `0~5 rad`，Agent 会自动换算。
- `pick_color` 使用 RS 专用 TCP 偏移、俯视姿态和抓取高度。
- 抓取结果不是只看轨迹是否完成；MuJoCo 物体必须实际抬升至少 35 mm。
- 颜色目标来自 `/rebotarm_rs/vision/color_blocks/detections`。

## 启动

完整仿真会自动启动 MCP Server：

```bash
cd /path/to/ReBot_Arm_DigitalTwin_RS
./scripts/start_rs_sim.sh
```

MCP 地址：

```text
http://127.0.0.1:8081/mcp
```

手动启动时：

```bash
source scripts/rs_env.sh
ros2 launch rebotarm_agent rebotarm_mcp.launch.py \
  arm_namespace:=rebotarm_rs motion_mode:=allow
```

启动供网页使用的 Text Agent/Dashboard：

```bash
export DASHSCOPE_API_KEY="..."
export REBOTARM_LLM_MODEL="qwen-plus"
./scripts/start_rs_text_agent.sh
```

Dashboard 默认地址是 `http://localhost:8082`。

## 主要工具

- `diagnose_ros`：检查抓取链路的 topic、service 和 action。
- `detect_blocks`：读取红、蓝、黄三种物体。
- `pick_color`：执行打开、接近、下降、闭合、抬升及物理验证。
- `set_gripper_opening_mm`：以毫米控制 RS 夹爪。
- `ik_check`、`move_to_pose`、`move_joints`：IK 和运动控制。
- `record_start`、`record_stop`、`record_replay`：仿真动作录制回放。

不启动 Text Agent 也可以通过任意 MCP 客户端直接调用这些工具。
