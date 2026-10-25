# robot_arm_description

这是机械臂和移动平台的 URDF/Xacro 描述包。

SolidWorks 导出的第一版模型已导入到 `urdf/imported/arm_urdf_v1.urdf`，对应网格位于 `meshes/`。原始导出目录仍保留在桌面，不由本包直接引用。

当前导入模型包含 `base_link`、`link1` 到 `link5` 以及 `joint1` 到 `joint5`。它还没有加入 D435i 的 `camera_link`，也没有加入移动底盘坐标系。

相机外参中的 `xyz` 和 `rpy` 仍需通过实测或标定获得，不能直接用于真实抓取。后续应在确认关节名称、轴方向、原点、单位、限位、惯性和 mesh 路径后，把导入 URDF 整理成 `urdf/robot_arm.urdf.xacro`。

查看当前骨架：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```
