# 执行机构 STM32 固件适配

在用户提供的底盘版 usart.c、main.c、bsp_tim.c 基础上改为独立执行机构控制板，适配已实现的 robot_end_effector 串口协议。**本目录不是底盘固件，不要替换底盘板工程。**

已确认 USART3（PD8=TX、PD9=RX）连接 Jetson，左执行机构接电机输出 A，右接 B。C、D 始终输出 0。接收参数为 115200/8N1、无流控。

| 动作 | 接收的 6 个二进制字节 | 事件 |
| --- | --- | --- |
| 左侧 | AA 55 01 01 0D 0A | STRIKE_LEFT（1） |
| 右侧 | AA 55 01 02 0D 0A | STRIKE_RIGHT（2） |
| 双侧 | AA 55 01 03 0D 0A | STRIKE_BOTH（3） |

帧尾已在帧内，不追加换行。不解析 ASCII 文本或 CMD_WHEELS；MASK=00 不是停止命令。上位机只发一次触发，不读取反馈，不持续发送速度指令。

## 参数

编辑 striker_config.h，以下参数控制执行机构板，不修改上位机：

| 参数 | 用途 |
| --- | --- |
| STRIKER_A_ARM=1、STRIKER_B_ARM=2 | 左 A、右 B，按用户接线填写 |
| STRIKER_A/B_COUNTS_PER_REV=1652 | 输出轴每转编码器计数，按用户提供的 7 PPR、59:1 和四倍频驱动填写 |
| STRIKER_A_PWM_SIGN=-1、STRIKER_B_PWM_SIGN=1 | 用户确认左 A 负 PWM、右 B 正 PWM |
| STRIKER_A_ENCODER_SIGN=-1、STRIKER_B_ENCODER_SIGN=1 | 沿用原 A/B 编码器方向与 PWM 正负的对应关系；实机仍需核对 |
| STRIKER_TARGET_REVOLUTIONS=10 | 每次事件的输出轴圈数 |
| STRIKER_RUN_PWM=700 | 常规运行 PWM，沿用用户测试的幅值，上限 1000 |
| STRIKER_MIN_PWM=200 | 最后半圈减速时的 PWM 下限，需实测能否带动负载 |
| STRIKER_STALL_MS=2000 | 无新的正向计数进展的最大持续时间 |
| STRIKER_MAX_RUN_MS=15000 | 单次动作总体超时，毫秒 |
| STRIKER_SETTLE_MS=200 | 全部选中侧置零后，连续无编码器增量多久才允许下次触发 |
| STRIKER_MAX_SETTLE_MS=2000 | 置零后持续无法静止的最大等待时间 |

用户提供减速比 1:59、编码器 7 PPR，编码器代码使用 TIM_EncoderMode_TI12 四倍频，Read_Encoder() 直接读取增量，没有除频。**按电机轴 A 相每圈 7 个周期计算，输出轴每转为 7×59×4=1652 次，10 圈为 16520 次。** 已写入参数，替代底盘原来的 1560。

左 A 的 PWM 已设负数，右 B 已设正数；编码器符号暂按原 A/B 正 PWM 产生正计数的关系设为 A=-1、B=1。编码器线序无法仅从初始化代码判断，首次测试如果报 fault=3，先确保机构转动方向正确，再核对并修改对应 ENCODER_SIGN。不要通过改计数符号掩盖机械方向接反。A/B 编码器输入沿用原代码 TIM2（PA15/PB3）和 TIM3（PB4/PB5）；接在其他口时需改 bsp_tim.c 的读取映射。

## Keil 工程替换位置

先备份执行机构工程，再按下表替换/添加；GPIO、PWM 和编码器驱动使用用户现有 STM32F4 标准外设库。

| 文件 | 操作 |
| --- | --- |
| main.c | 替换原 main：去掉 IMU、光电、底盘反馈，以及循环中的 motor_control(-700,700,700,700) |
| usart.c | 替换原串口文件：保留原 USART1 调试和 printf，删除底盘 USART3 实现 |
| bsp_tim.c、bsp_tim.h | 替换 TIM7：去掉四轮 PI、摩擦补偿、光电/后退限位和底盘通信看门狗 |
| usart3_striker.c、usart3_striker.h | 新增串口模块，把 .c 加入 Keil 工程 |
| striker_control.c、striker_control.h | 新增计圈/停机状态机，把 .c 加入 Keil 工程 |
| striker_config.h | 新增参数头文件，填写方向和每转计数 |

工程里只保留一个 main()、一份 TIM7_IRQHandler()、一份 uart3_init() 和一份 USART3_IRQHandler()。不要同时加入旧底盘 usart3_chassis.c。

保留原 usart.h，新 main 已额外包含 usart3_striker.h。其他文件若仍调用 USART3_ProcessCommand、USART3_CommandWatchdogTick、USART3_GetWheelTargets 或底盘段号/反馈函数，应去除这些底盘调用，不要用空函数掩盖残留的底盘逻辑。

Read_Encoder() 必须保持原代码“读取并清除本次增量”的行为。启动时初始化原 TIM2～TIM5 和四路 PWM；之后仅 TIM7 每 5 ms 写一次 motor_control，主循环不写电机输出。TIM7 保留 84 MHz 时钟、预分频 83、重装 4999；改时基须同步修改 STRIKER_TICK_MS。

## 执行流程

1. 收齐合法帧后，串口中断立即锁存 busy，产生一次事件，不直接驱动电机。
2. 主循环取走事件，清除启动前的残留计数，以各自当前轴位置为相对起点启动指定侧。
3. TIM7 按编码器净计数控制，最后半圈降低 PWM；每路达到 10 圈对应计数后独立置零，未选中侧一直为零。
4. 双侧必须都置零，且选中侧连续 200 ms 无计数，才解锁。忙时所有新帧丢弃、不排队；完成后相同帧可再次触发。
5. 配置错误、反向计数、无进展、总体超时或无法静止会全部置零并保持故障/busy 锁存；查明原因后复位。

USART1 打印 ready、start、finished 或 fault。故障码：1=配置不完整，2=无计数进展，3=计数方向反，4=总体超时，5=置零后持续有计数。

置零使用现有 motor_control(0,0,0,0)，其刹车/滑行行为由驱动实现决定。计数阈值和 5 ms 采样不能保证物理轴恰好停在 10.000 圈，需实测调整减速和停机；没有新增固定反向 PWM。

## 验证

在 WSL 仓库根目录运行：

```bash
python3 deployment/stm32_striker/test_firmware.py
```

用 GCC 编译真实串口、计圈和 TIM7 源码，虚拟寄存器输入上位机实际 STRIKE_COMMAND_HEX，覆盖拆帧、错帧恢复、忙时丢弃、相对计圈、独立停车、连续静止和故障停机。测试不打开串口，不控制电机；通过不代表 Keil 全工程或硬件验证通过。

确认并填写参数后，用 Keil 编译并下载到执行机构板。先断开电机动力电源核对 USART1 接收/start 信息，再接电单次测试左、右、双侧。串口助手选 HEX 发送；上位机独立测试命令见 ros2_ws/src/robot_end_effector/SERIAL_STRIKER.md。

motor_return_delay_s / wait_s 只控制机械臂回位前的本地等待，不改变 STM32 圈数，也不确认完成。必须大于实测动作时间，原默认 1 秒可能不够。取消 ROS Action 不能停止已经启动的机构；本板依靠计圈、故障限时和机构电源开关停止。
