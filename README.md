# 肘击装置控制器

STM32F407VETx 固件，包含视觉串口解析、四区域肘击电机控制、编码器测速、底盘 CAN 协同及 128×64 OLED 状态显示。

## 工程入口

- Keil 工程位于 `RED_Demo/RED_Demo/MDK-ARM/RED_Demo.uvprojx`，使用 ARMCC5。
- CubeMX 配置位于 `RED_Demo/RED_Demo/RED_Demo.ioc`。
- 应用代码位于 `RED_Demo/RED_Demo/BSP`。
- 硬件参考见 `原理图.pdf`，视觉接线与控制说明见 `RED_Demo/RED_Demo/视觉联动说明.md`。

## OLED 显示

每 100 ms 刷新一次，使用现有英文字库。八行依次显示视觉在线状态、区域和树偏移，电机编号与动作阶段及剩余时间、PWM，车辆状态与 CAN 在线状态，肘击作业阶段与结果码及 CAN 诊断码，最后四行显示 A/B/C/D 电机转速。

`CAR` 来自底盘针对当前任务的 CAN 应答。

| 屏幕文字 | 含义 |
| --- | --- |
| OFFLINE | 底盘通信离线，超过 500 ms 无有效应答后隐藏旧状态 |
| WAIT | 已在线，等待当前任务的有效状态 |
| SYNC | 正在握手同步 |
| READY | 底盘就绪 |
| MOVING | 底盘应答携带运动标志 |
| SLOW | 底盘已确认减速 |
| STOPPING | 正在停车 |
| STOPPED | 底盘已确认停稳 |
| RELEASED | 底盘已确认放行 |
| FAULT | 底盘报告故障 |

`W` 表示肘击作业阶段。`E` 表示作业结果码，底盘报告故障时显示其故障原因。0 正常，1 达到区域重试上限，2 单树超时，3 视觉异常，4 位置异常，5 通信异常，6 停车超时，7 轮速反馈异常，8 控制器复位，9 协议异常，10 无目标跳过。

`C` 表示 CAN 诊断码。0 等待，1 收到有效应答，2 初始化错误，3 发送错误，4 接收错误，5 应答超时。车辆状态按有效应答显示，诊断码保留最近通信诊断结果。

## 验证

在原工作区使用 Python 和 MinGW GCC 执行 `tests/verify_work_can.py`、`tests/verify_vision_motor.py` 和 `tests/verify_oled.py`，脚本位于工程目录内。测试依赖原工作区同级的 `Mango-Swerve-Firmware/firmware` 中的底盘源码与模拟头文件。

2026-11-09 三项主机测试通过，Keil 编译为 0 错误、0 警告，生成 `RED_Demo/RED_Demo/MDK-ARM/RED_Demo/RED_Demo.hex`。新增显示功能尚待烧录和双板实机验证。

本次恢复了既有作业控制代码所需的缺失声明和参数，使用此前实现值。作业中心为 0，减速与停车半宽分别为 100 和 30 像素，离树方向为 -1，动作后稳定等待 300 ms。这些值需要根据实际安装与车辆运行标定。
