# 机械臂工作空间故障记录

本文记录 `robot_arm_description` 在 URDF/RViz 初始搭建阶段遇到的问题。
后续遇到新问题时，按照“现象、原因、排查、修复、验证”的格式继续追加。

## 1. `robot_description` 参数无法解析

### 现象

执行：

```bash
ros2 launch robot_arm_description display.launch.py
```

出现：

```text
Unable to parse the value of parameter robot_description as yaml.
If the parameter is meant to be a string, try wrapping it in
launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
```

### 原因

URDF/Xacro 的输出是一整段 XML 字符串，但 launch 文件直接把 `Command(...)` 的结果作为参数传给了 ROS 2。ROS 2 参数系统尝试把 XML 自动解析成 YAML，导致参数类型解析失败。

### 修复

在 `display.launch.py` 中显式指定参数类型为字符串：

```python
from launch_ros.parameter_descriptions import ParameterValue

{
    'robot_description': ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str,
    )
}
```

如果输入文件是 Xacro，则使用：

```python
ParameterValue(
    Command(['xacro ', LaunchConfiguration('model')]),
    value_type=str,
)
```

### 验证

```bash
python3 -m py_compile src/robot_arm_description/launch/display.launch.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm_description
```

启动日志中应能看到：

```text
robot_state_publisher: got segment base_link
robot_state_publisher: got segment link1
robot_state_publisher: got segment link2
robot_state_publisher: got segment link3
robot_state_publisher: got segment link4
robot_state_publisher: got segment link5
```

## 2. RViz 启动但不显示机械臂

### 现象

RViz 可以启动，左侧显示：

```text
Global Status: Ok
RobotModel: Status Ok
```

但右侧只有 Grid，看不到机械臂。

### 排查过程

先检查 ROS 话题：

```bash
ros2 topic list
```

应至少包含：

```text
/joint_states
/robot_description
/tf
/tf_static
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link link5
```

如果能持续输出 Translation 和 Rotation，说明 URDF 的关节链和 TF 基本正常。

检查 URDF 是否被读取：

```bash
ros2 topic echo /robot_description \
  --qos-durability transient_local \
  --qos-reliability reliable \
  --full-length \
  --once > /tmp/robot_description.txt
```

注意：不加 `--full-length` 时，`ros2 topic echo` 会把很长的 XML 截断成 `Th...`，不能据此判断 URDF 为空。

还可以直接检查安装后的文件：

```bash
grep -n "link\|joint\|package://" \
  install/robot_arm_description/share/robot_arm_description/urdf/imported/arm_urdf_v1.urdf \
  | head -30
```

### 发现的两个问题

#### 2.1 SolidWorks 导出的材质名为空

原始 URDF 中存在：

```xml
<material name="">
```

部分 RViz/URDF 解析路径对空材质名处理不一致，可能出现：

```text
URDF failed Model parse
```

修复为非空材质名，例如：

```xml
<material name="base_link_material">
```

其他 link 使用各自的合法名称。

#### 2.2 RViz 的 `robot_description` QoS 不匹配

`robot_state_publisher` 对 `/robot_description` 使用持久化发布，而 RViz 默认配置中的：

```text
Durability Policy: Volatile
```

当 RViz 启动晚于 `robot_state_publisher` 时，可能收不到已经发布的 URDF，导致 RobotModel 为空。切换为 File 能显示模型，说明模型文件和 STL 路径本身有效。

在 RViz 配置中，将 RobotModel 的 Description Topic 设置为：

```text
Description Source: Topic
Description Topic: /robot_description
Durability Policy: Transient Local
Reliability Policy: Reliable
```

### 验证

重新构建并启动：

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

RViz 中确认：

```text
RobotModel -> Visual Enabled: true
Global Options -> Fixed Frame: base_link
RobotModel -> Description Source: Topic
RobotModel -> Durability Policy: Transient Local
```

必要时点击 RViz 右侧 3D 视图并按 `F` 聚焦模型。

## 3. 常用验证清单

以后修改 URDF、Xacro 或 RViz 配置后，按以下顺序检查：

```bash
source /opt/ros/humble/setup.bash
xacro src/robot_arm_description/urdf/robot_arm.urdf.xacro > /tmp/robot_arm.urdf
check_urdf src/robot_arm_description/urdf/imported/arm_urdf_v1.urdf
colcon build --symlink-install --packages-select robot_arm_description
source install/setup.bash
ros2 topic list
ros2 run tf2_ros tf2_echo base_link link5
```

如果只有 RViz 不显示，优先检查：

1. `RobotModel` 是否启用 Visual；
2. Fixed Frame 是否为 `base_link`；
3. Description Source 是否为 Topic；
4. Durability 是否为 `Transient Local`；
5. URDF 中 mesh 的 `package://` 路径是否存在；
6. RViz 启动日志是否出现 `Could not load resource`。

## 4. 记录模板

```markdown
## 日期：YYYY-MM-DD 问题标题

### 现象

### 环境和命令

### 原因

### 排查证据

### 修复

### 验证结果

### 后续风险
```

## 5. Gazebo 仿真阶段问题总结

### 5.1 URDF/XML 结构

**现象**：`robot_state_publisher` 崩溃、`exit code -6` 或 XML 解析失败。

**原因**：文件中存在多个 `<?xml ?>`、多个 `<robot>` 根节点，或复制产生了重复的 link/joint。`<ros2_control>` 和 `<gazebo>` 放在 `<robot>` 外也会导致模型或控制器无法加载。

**修复**：整个文件只保留一个 XML 声明和一个 `<robot>...</robot>`，并用：

```bash
check_urdf path/to/robot.urdf
```

### 5.2 Gazebo 找不到 STL

**现象**：`Unable to find file with URI`，常见于 `package://robot_arm_description/meshes/...`。

**原因**：Gazebo 没有获得 ROS 包资源路径，或资源路径多了包名这一层。

**修复**：Fortress 使用 `IGN_GAZEBO_RESOURCE_PATH`，路径指向安装空间的 `share` 目录：

```python
from launch.actions import AppendEnvironmentVariable

env_var = AppendEnvironmentVariable(
    "IGN_GAZEBO_RESOURCE_PATH",
    os.path.dirname(description_share),
)
```

目标应为：

```text
.../install/robot_arm_description/share
```

新版本 Gazebo 使用 `GZ_SIM_RESOURCE_PATH`，以实际版本为准。

### 5.3 `$(find ...)` 没有展开

**现象**：参数文件路径中出现字面量 `$(find robot_arm_gazebo)`。

**原因**：launch 使用 `cat` 读取纯 URDF 时不会执行 xacro 替换。

**修复**：使用 xacro 处理，或读取文本后替换：

```python
import re


def replace_find(text):
    return re.sub(
        r"\$\(find ([^)]+)\)",
        lambda match: get_package_share_directory(match.group(1)),
        text,
    )

```
### 5.4 Launch 文件常见错误

- 使用 `re`、`TimerAction`、`AppendEnvironmentVariable` 前先导入；
- 定义和使用统一的变量名，例如 `robot_description`；
- `IncludeLaunchDescription` 使用 `PythonLaunchDescriptionSource(path)` 包装路径；
- controller manager 启动后再延迟启动两个 spawner。

```python
TimerAction(
    period=8.0,
    actions=[
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller"], output="screen"),
    ],
)
```

### 5.5 ros2_control 服务不存在

**现象**：`Could not contact service /controller_manager/list_controllers`。

**原因**：ros2_control 名称、spawner 使用的 controller manager 名称不一致，或 controllers.yaml 没有加载成功。

**修复**：优先统一：

```xml
<ros2_control name="controller_manager" type="system">
```

如果实际节点名称不同，给 spawner 指定：

```bash
--controller-manager /实际的controller_manager名称
```

如果有 `/gz_ros2_control` 但没有 `list_controllers`，检查 `$(find ...)` 展开、YAML 安装路径和插件名称。

### 5.6 controllers.yaml 没有安装

在 `robot_arm_gazebo/CMakeLists.txt` 中加入：

```cmake
install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)
```

修改源码后必须执行：

```bash
colcon build --symlink-install --packages-select robot_arm_gazebo
source install/setup.bash
```

### 5.7 机器人悬空或倒下

`world` link 只是 URDF 模型中的普通 link，不等于 Gazebo 的物理世界锚点。`world -> base_link` 可以建立 TF 关系，但本身不一定固定动态模型。

仿真专用描述中应使用固定基座方案，并检查 spawn 的 `-z` 高度。不要把整个机器人设置为 static 后再验证关节运动，否则会影响可动关节控制。

### 5.8 tcp_link 惯性警告

固定的 `tcp_link` 没有惯性时，Gazebo 可能提示缺少 inertia。固定末端通常不需要独立动力学；如果插件强制要求，可在仿真文件中加入临时小惯性。正式模型应填写实际工具质量和惯量，不能把 `1e-6` 当成真实参数。

### 5.9 `/joint_states` 读取

`joint_state_broadcaster` 不保证消息数组顺序，必须按 `msg.name` 建立索引：

```python
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
name_to_index = {name: i for i, name in enumerate(msg.name)}
q = [msg.position[name_to_index[name]] for name in joint_names]
```

`effort: NaN` 通常表示没有声明 effort 状态接口。位置和速度控制不受影响；需要力矩反馈时再增加：

```xml
<state_interface name="effort"/>
```

### 5.10 Gazebo 版本和插件

确认版本使用：

```bash
ign gazebo --version
# 或
 gz sim --version
```

| 环境 | 启动命令 | 资源变量 | 控制插件示例 |
| --- | --- | --- | --- |
| Fortress | `ign gazebo` | `IGN_GAZEBO_RESOURCE_PATH` | `ign_ros2_control` / `libign_ros2_control-system.so` |
| Harmonic/Ionic | `gz sim` | `GZ_SIM_RESOURCE_PATH` | `gz_ros2_control` / `libgz_ros2_control-system.so` |

以系统实际安装的 ROS 包和插件库为准，不能混用两套配置。

### 5.11 WSL 图形渲染

Gazebo GUI 出现 OGRE/OpenGL 崩溃时，可临时使用：

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch robot_arm_bringup sim.launch.py
```

## 6. 仿真验证结果

当前验证链路：

```text
pick_sequence_node
  -> /arm_controller/joint_trajectory
  -> arm_controller
  -> Gazebo
  -> joint_state_broadcaster
  -> /joint_states
  -> robot_state_publisher
  -> /tf
```

检查命令：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo base_link tcp_link
ros2 run robot_arm_node pick_sequence_node
```

固定关节轨迹已经能够驱动仿真机械臂完成往返运动。下一步是把固定关节角替换为 `base_link -> tcp_link` 的 IK 结果，再实现预抓取、抓取、抬升和返回起始位流程。
