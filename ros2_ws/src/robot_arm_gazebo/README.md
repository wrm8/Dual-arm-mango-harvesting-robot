# robot_arm_gazebo 仿真说明

## 一、正常仿真需要加载的内容

一次完整的 Gazebo 仿真需要依次加载：

```text
仿真启动文件
  -> Gazebo 世界
  -> 机器人描述（URDF）
  -> robot_state_publisher
  -> /clock 桥接
  -> 生成机械臂实体
  -> ros2_control 控制器
  -> /joint_states 和 /tf
```

当前入口：

```bash
ros2 launch robot_arm_gazebo simulation.launch.py
```

对应 `launch/simulation.launch.py` 的作用如下。

### 1. Gazebo 资源路径

```python
AppendEnvironmentVariable(
    "IGN_GAZEBO_RESOURCE_PATH",
    os.path.dirname(description_share),
)
```

Fortress 使用 `IGN_GAZEBO_RESOURCE_PATH` 查找 `package://robot_arm_description/meshes/...` 的 STL。路径应指向安装空间的 `share` 目录。

### 2. 启动世界

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gz_launch_path),
    launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
)
```

`worlds/arm_test.sdf` 定义地面、光照、重力和物理参数，不包含机械臂本体结构。

### 3. 发布机器人描述和 TF

```python
Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{
        "robot_description": robot_description,
        "use_sim_time": True,
    }],
)
```

当前仿真使用：

```text
urdf/sim/arm_urdf_v1_sim.urdf
```

模型链为：

```text
world -> base_link -> link1 -> ... -> link5 -> tcp_link
```

`robot_state_publisher` 根据 `/joint_states` 发布可动关节 TF。

### 4. 桥接 Gazebo 时钟

```python
Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
)
```

仿真节点使用 `/clock`，所以 `robot_state_publisher` 设置了 `use_sim_time: True`。

### 5. 生成机械臂

```python
Node(
    package="ros_gz_sim",
    executable="create",
    arguments=["-topic", "robot_description", "-name", "robot_arm", ...],
)
```

该节点从 `robot_description` 读取 URDF，并将机械臂生成到 Gazebo 世界。

### 6. 加载 ros2_control 控制器

启动 8 秒后执行两个 spawner：

```python
Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
)
Node(
    package="controller_manager",
    executable="spawner",
    arguments=["arm_controller"],
)
```

作用：

```text
joint_state_broadcaster -> 发布 /joint_states
arm_controller           -> 接收关节轨迹并驱动 Gazebo 关节
```

完整数据链路：

```text
JointTrajectory
  -> arm_controller
  -> Gazebo 关节
  -> joint_state_broadcaster
  -> /joint_states
  -> robot_state_publisher
  -> /tf
```

## 二、标准启动和检查顺序

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_gazebo simulation.launch.py
```

等待控制器启动后检查：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list | grep arm_controller
ros2 topic echo /joint_states
```

正常结果应包含：

```text
arm_controller active
joint_state_broadcaster active
```

查看末端 TF：

```bash
ros2 run tf2_ros tf2_echo base_link tcp_link
```

## 检验gazebo是否可以控制
1.先确认控制器加载成功
  ros2 control list_controllers
2.确认硬件接口被识别
  ros2 control list_hardware_interfaces
3.确认话题存在
  ros2 topic list | grep arm_controller
# 4. 发一条测试轨迹
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [joint1, joint2, joint3, joint4, joint5],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }]
  }'

LIBGL_ALWAYS_SOFTWARE=1 \
ros2 launch robot_arm_gazebo simulation.launch.py

## 三、问题记录：未启动控制器导致机械臂倒下

### 现象

Gazebo 能生成机械臂，但机械臂加载后倒下或关节无法保持姿态。

### 原因

机械臂 link 是带质量和重力的动态刚体。实体生成后，如果 `arm_controller` 还没有启动，关节没有位置控制命令，模型会在重力作用下自由运动。仅有 `robot_state_publisher` 或 `world -> base_link` 的 TF 关系，不能代替 Gazebo 的物理控制。

### 修复

确保 `simulation.launch.py` 延迟加载：

```python
TimerAction(
    period=8.0,
    actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
            output="screen",
        ),
    ],
)
```

控制器配置应包含五个关节：

```yaml
arm_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5]
```

### 验证

```bash
ros2 control list_controllers
```

确认两个控制器均为 `active`，再发送测试轨迹：

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [joint1, joint2, joint3, joint4, joint5],
    points: [{
      positions: [0.0, 0.3, -0.5, 0.2, 0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }'
```

如果 `/joint_states` 随之变化且机械臂完成运动，说明控制链路正常。

### 注意

- `world` link 只建立模型坐标关系，不单独提供物理固定；
- 不要把整个机器人设为 static 后验证关节控制；
- 控制器启动前短暂下落属于启动时序问题，控制器 active 后再观察最终姿态；
- 如果仍然倒下，检查 ros2_control 插件、控制器 YAML、关节名称和 `arm_controller` 状态。

## 四、手动控制测试

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list | grep arm_controller
```

WSL 中出现图形渲染问题时：

```bash
LIBGL_ALWAYS_SOFTWARE=1 \
ros2 launch robot_arm_gazebo simulation.launch.py
```

