# STM32 串口三：段号下发与光电停车参考实现

这是根据本次提供的 USART3 代码整理的完整替换模块，使用 STM32F4 标准外设库。当前仓库没有对应的下位机工程，因此文件尚未加入 Keil 工程，也未烧录到控制板。GPIO 具体端口和有效电平需要按接线填写。

## 1. 替换串口三代码

- 将 `usart3_chassis.c`、`usart3_chassis.h` 复制到下位机工程，给编译器添加头文件搜索目录，并将 `.c` 加入编译。
- 从原文件删除本次贴出的整块 USART3 代码，避免 `uart3_init`、`USART3_IRQHandler` 等重复定义。若与串口一/二共用一个 `usart.c`，只删除串口三部分，保留其他串口。
- 在 `main.c`、TIM7 所在源文件中加入 `#include "usart3_chassis.h"`。
- 继续调用 `uart3_init(115200)` 和主循环中的 `USART3_ProcessCommand()`。PD8/PD9、优先级、四轮 mm/s 单位、原有 `FB` 格式保持一致。

新接收帧为 `CMD_WHEELS A B C D stage_id\r\n`。段号范围 `0..15`，只有不同段号才清除到位锁存。启动时仍接受原四速度帧；进入非零段后，忽略不带段号的旧帧，避免旧命令解除停车。需要回到手动模式时必须明确发送段号 `0`。

## 2. 在 TIM7 控制周期内输入光电状态

在原来读取轮速目标、执行 PI 的位置前加入以下调用。保留原来的定时器清中断逻辑、PI 和编码器处理：

```c
/* 以下为放入原 TIM7 控制周期中的调用顺序，不是完整中断函数。 */
USART3_UpdatePhotoelectric(front_active, left_active, right_active);
USART3_CommandWatchdogTick(elapsed_ms);
USART3_GetWheelTargets(target_mmps);
/* 此后沿用原 PI 控制器，使用这四个 target_mmps，不可改用旧缓存。 */
```

`front_active / left_active / right_active` 是在本周期读取 GPIO 得到的 `0/1`，**1 表示检测到障碍**。如果模块输出低电平有效，应写成：

```c
front_active = (GPIO_ReadInputDataBit(实际端口, 实际引脚) == Bit_RESET);
```

上面的端口/引脚是说明用占位符，不能原样编译。高电平有效则比较 `Bit_SET`。先配置好实际 GPIO 输入，再开始周期采样；不要把未接入的输入固定写为 `0` 后就认为光电兜底已启用。`elapsed_ms` 使用 TIM7 实际周期，原来已有看门狗调用的地方不要重复调用。

段号与光电对应：1 前方；2/6/10/14 右侧；4/8/12 左侧；0 与其余后退段不启用这三个光电。对应表在 `photo_mask_for_stage()`。后退段保留上位机已有策略。

触发后 `usart3_stage_reached=1`，四个目标速度同时置零；光电松开、同段重复速度或通信恢复均不清除锁存。换段时会立即检查已采集的新段光电状态。如果上一段前方还触发、第二段右侧未触发，则第二段可以正常右移。

零速在下位机本地生效，不依赖 Jetson 收到反馈；轮速 PI/电机实际刹车响应仍需实物确认。

## 3. 反馈继续从主循环发送

保留原来的周期调用：

```c
USART_SendFeedback(sequence, period_ms, encoder_delta, yaw_mdeg);
```

它会先发原有 `FB ...\r\n`，再自动发送 `STAGE_FB stage_id reached\r\n`。不用再手动重复调用 `USART3_SendStageFeedback()`；段号为 0 时不追加此帧。发送仍采用原来的阻塞方式，因此这些发送函数放在同一个主循环/任务内；TIM7/EXTI 只更新状态，不格式化或发送串口数据，避免阻塞控制周期及反馈帧交错。

没有完整的 `main.c`/TIM7 文件，无法替你确认当前反馈函数的调用上下文；若原来在 TIM7 内发送，需把反馈快照复制到主循环再发送。

## 4. 与 Jetson 对接

```text
接收 CMD_WHEELS 200 200 200 200 1  -> 第一段，未触发时执行速度
前方触发                        -> 立即四轮目标零速，锁存
反馈 STAGE_FB 1 1
接收 CMD_WHEELS 200 200 200 200 1  -> 同一段，仍是零速
接收 CMD_WHEELS 0 0 0 0 2          -> 切换第二段，改判断右侧
接收 CMD_WHEELS -100 100 -100 100 2 -> 右侧未触发时向右横移
```

确认下位机编译烧录、GPIO 接入并完成上述联调后，再启用 `ros2_ws/src/robot_chassis/config/chassis.yaml` 中的 `stage_protocol_enabled: true`，重启上位机串口节点。完整协议见 [光电停车协议](../../ros2_ws/src/robot_chassis/docs/photoelectric_protocol.md)。

主机侧逻辑测试（模拟 STM32 外设函数，不连接控制板）：

```bash
python3 deployment/stm32_chassis/test_usart3.py
```

该测试只验证 C 模块的命令解析、段号切换、锁存、断线停车和反馈格式，不等于 Keil 编译或实车验证。
