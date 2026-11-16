# 采摘机器人数据监测上位机

独立的 ROS 2 监测包。服务在 Jetson 上运行，电脑浏览器查看。只订阅机器人数据并调用参数查询服务；没有速度发布器、动作客户端、复位或启动任务功能，也不占用底盘串口。

## 可以查看什么

| 功能 | 数据来源与含义 |
| --- | --- |
| 四轮实测速度、编码器增量、速度曲线 | `/base/wheel_state`，A 左后 / B 左前 / C 右前 / D 右后 |
| 各轮起步延迟、最慢轮、起步时间差 | 从静止开始，按指令接收时刻与轮速阈值估计，显示当次航向变化 |
| 换算目标轮速 | `/base/route_command.velocity` 或 `/base/cmd_vel` 加底盘尺寸计算；不是 STM32 实际目标/PWM |
| 航向、底盘速度、超时、路线段、到位 | `/base/state`、`/base/stage_feedback`、`/base/odom`、`/base/imu` |
| 四向超声波及 valid | `/base/ultrasonic` 原消息，不擅自把无效/超量程换算成实际距离 |
| 视觉停车、树干误差、果实目标 | `/base/vision_stop_state`、`/vision/row_observation`、`/vision/harvest_targets`、`/vision/mango_target`、`/vision/observations` |
| 机械臂关节反馈 | `/joint_states` 原始消息 |
| 告警、频率、数据新鲜度、节点/话题列表 | `/rosout` WARN 及以上、订阅接收频率、ROS 图发现 |
| 自动记录 | JSONL 原始消息、起步事件 CSV；网页可导出最近 100 条事件 |

任意其他已发现话题可用 `--topic /your/topic` 增加，只显示/记录原始消息。默认不会订阅相机图像，以免增加图像传输和推理负担。自定义消息根据 ROS 图动态加载：未开启视觉时也能启动；接口环境不匹配会在页面报告。

## 先在当前 Windows 电脑看演示

在包含 `ros2_ws` 的目录打开 PowerShell：

```powershell
python ros2_ws/src/robot_monitor/scripts/robot_monitor.py --demo --no-record
```

浏览器打开 `http://127.0.0.1:8765`。不需要 ROS。页面顶部明确标注“模拟数据”；每 6 秒模拟一次第 4 段横移，故意让 A 左后轮晚约 250 ms 起转。Ctrl+C 退出。

## 放进实际 ROS 工作空间

将整个 `robot_monitor` 目录放到实际工作空间的 `ros2_ws/src/robot_monitor/` 下；不要放进已有 `robot_chassis` 包里面。

当前 Windows 文件目录可从 WSL 读取；下面的路径仅用于将本次新增包复制到本地 WSL，目标目录已有文件时先停止并核对，避免覆盖：

```bash
cd /home/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
test ! -e src/robot_monitor && cp -R \
  '/mnt/c/Users/Lenovo/Desktop/shh链接codex/ros2_ws/src/robot_monitor' src/
chmod +x src/robot_monitor/scripts/robot_monitor.py
```

Jetson 则需要先同步这个新增目录到自己的 `ros2_ws/src/`。WSL 编译通过不等于 Jetson Humble 已验证。

## Jetson 启动真实数据监测

在 Jetson 的普通 ROS 终端运行（使用系统 Python，而非缺少 ROS Python 模块的独立视觉虚拟环境）：

```bash
cd /home/wrm/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
# 已有接口包时先加载它，新增监测包本身不要求重新编译底盘。
source install/setup.bash
colcon build --packages-select robot_monitor --symlink-install
source install/setup.bash

ros2 run robot_monitor robot_monitor.py \
  --host 0.0.0.0 --port 8765 \
  --chassis-config "$PWD/src/robot_chassis/config/chassis.yaml"
```

`--host 0.0.0.0` 允许同一局域网电脑连接。电脑浏览器打开 `http://10.148.44.94:8765`（IP 以当前 Jetson 地址为准）。这是无账号的局域网只读页面，不要将端口转发到公网。默认不加 `--host` 时仅本机访问。

也可不编译，直接运行同一源文件（仍然需要 source 正确的 ROS 与接口包）：

```bash
python3 src/robot_monitor/scripts/robot_monitor.py \
  --host 0.0.0.0 \
  --chassis-config "$PWD/src/robot_chassis/config/chassis.yaml"
```

或者使用 launch：

```bash
ros2 launch robot_monitor monitor.launch.py \
  host:=0.0.0.0 port:=8765 \
  chassis_config:="$PWD/src/robot_chassis/config/chassis.yaml"
```

监测节点不会替你启动 `chassis.launch.py`。先让监测运行，再按原来的流程由你启动底盘/任务。订阅会每 2 秒发现新话题，因此节点先后启动均可。ROS_DOMAIN_ID、RMW 及网络配置应与被监测的节点一致；最省事是在 Jetson 上运行监测服务，只把网页传给电脑。

退出监测用 Ctrl+C。**关闭监测不会向机器人发停车命令**；任务仍在运行时，请使用原任务的停止方式或实体急停。

## 怎么判断左后轮是不是晚起步

1. 底盘静止时先打开监测，确认四轮 `valid=true`、数据未过期，至少等 0.2 秒。
2. 按原流程运行你要测试的横移。面板只观察，不触发运动。
3. 看“起步事件”最新一行，确认路线段和时间。A/B/C/D 的值是从收到非零指令到首次观察到轮速达到阈值的延迟，单位 ms。
4. 例如 `A=250、B=50、C=50、D=50`，意味着此次观测 A 比其他轮晚约 200 ms 达到阈值。看速度曲线确认 A 的上升确实较晚，结合当次 yaw 变化判断与起步歪斜是否相关。
5. 如果“数据不足 / 采集中断”，先看原因，不能把它解释成同步。没有静止基线、反馈过期、低目标速度、指令方向/路线改变都会拒绝给出正常判定。

默认阈值为 `0.03 m/s`，连续 2 个独立接收样本确认，起步观察窗口 `1.5 s`，四轮时间差达到 `100 ms` 告警。可调整：

```bash
ros2 run robot_monitor robot_monitor.py --host 0.0.0.0 \
  --start-threshold 0.02 --delay-warning-ms 100
```

这里测到的是“编码器轮速达到阈值的时间”，不是肉眼开始动的绝对时刻。串口反馈、ROS 发布与调度均带来延迟；若实际反馈为 20 Hz，时间分辨率约 50 ms，不能据此证明 5–10 ms 级的严格同时起步，也不能证明轮胎没有打滑。页面显示实际收到的频率和每个事件的平均反馈间隔。消息头不是 STM32 采样时间时，重复发布的旧采样也可能限制可信度；需要更精确同步性时，应在固件增加采样序号/采样时间，再做更高频采集。

四轮速度在航向修正/沿墙修正期间本来就可能不同，不以“轮速必须始终相等”判断故障。发现 A 延迟只能说明观测结果，不能单凭本页断定是电机、PWM、静摩擦还是供电造成。

## 指令来源和几何参数

- `--command-source auto`：没有出现路线指令时使用 `/base/cmd_vel`；一旦收到 `/base/route_command`，本次运行固定以路线指令为准，避免同一指令的镜像话题被重复计为起步。若后来切换到纯手动 `cmd_vel` 测试，请重启监测或显式传 `--command-source cmd_vel`。
- `--command-source route`：始终使用路线指令。
- `--command-source cmd_vel`：始终使用 Twist 指令。
- `--chassis-config`：读取 `base_serial_node.ros__parameters` 的轴距、轮距、限速与超时。若不指定，后台尝试向 `/base_serial_node/get_parameters` 查询实际值；查询不到会明确显示“默认尺寸，请核对”。监测不修改参数。

换算公式（轮线速度，m/s）：

```text
r = (wheel_base_m + track_width_m) / 2
A = vx + vy - r*wz
B = vx - vy - r*wz
C = vx + vy + r*wz
D = vx - vy + r*wz
```

按每轮限速裁剪。它只是现有 ROS 底盘运动学的预期值，不证明下位机收到或执行了该值。PWM、电流、积分项、驱动器故障等当前没有 ROS 反馈的项目不会凭空出现；这些项目需要固件新增遥测后再接入。

## 记录在哪里

默认在 Jetson `~/.ros/robot_monitor/日期-时间/`：

```text
telemetry.jsonl       原消息、接收单调时间、墙钟时间、类型和话题
telemetry.1.jsonl     上一卷，依次滚动
startup_events.csv    起步事件及每轮延迟、时间差、航向变化
```

JSONL 默认每卷 128 MiB、当前加历史共保留 4 卷（同一次运行约 512 MiB 上限，不包含很小的事件 CSV）。历史运行目录不会自动删除。后台队列满时丢弃记录并计数，不阻塞 ROS 接收；磁盘写入失败也会显示在页面。网页 CSV 导出仅包括内存中最近 100 个事件，服务器 CSV 保存本次运行全部事件。

```bash
# 不记录到磁盘
ros2 run robot_monitor robot_monitor.py --no-record
# 指定目录和滚动大小
ros2 run robot_monitor robot_monitor.py --log-dir ~/robot_logs --max-log-mb 64 --keep-log-files 4
# 增加一个普通消息话题
ros2 run robot_monitor robot_monitor.py --topic /diagnostics
```

本工具没有录像/画面识别显示，也不接管 RealSense；当前视觉部分监测的是 ROS 检测结果与停车状态。

## 验证和排查

不依赖 ROS 的测试：

```bash
python3 -m unittest discover -s src/robot_monitor/test -v
python3 src/robot_monitor/scripts/robot_monitor.py --demo --no-record
```

网页打不开：在 Jetson 本机检查 `curl http://127.0.0.1:8765/api/state?compact=1`，再检查服务是否使用 `--host 0.0.0.0`、IP、局域网连通性和防火墙。网页在线但没有 ROS 数据时检查：

```bash
ros2 topic list -t
ros2 topic info /base/wheel_state -v
ros2 topic echo --qos-profile sensor_data /base/wheel_state --once
```

监测使用 best-effort/volatile 订阅以兼容传感器数据；显示的是接收频率。只发布一次且没有持续重发的历史消息，不保证在监测晚启动后仍能收到。

本次本地离线测试与演示不等于 Jetson Humble 构建/ROS 实机链路已经验证；尤其不能把演示中的 A 轮延迟当作你的机器人实测结果。

本次已完成的验证（2026-11-12）：Windows Python 3.10 的 14 项测试与 Edge 演示页渲染；本地 WSL ROS Jazzy 编译、CTest，以及隔离本机 ROS 域中的真实消息订阅/参数查询检查。未进行 Jetson Humble 部署与底盘运动测试。

在本地 WSL 运行时，将上文 `/opt/ros/humble/setup.bash` 换为 `/opt/ros/jazzy/setup.bash`，工作空间路径使用 `/home/wrm/Dual-arm-mango-harvesting-robot/ros2_ws`。
