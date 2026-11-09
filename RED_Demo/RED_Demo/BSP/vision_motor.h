#ifndef VISION_MOTOR_H
#define VISION_MOTOR_H

#include "main.h"

/* ==================== 常用参数：主要修改下面五行 ====================
 * 修改方法：只改右侧数字，保留数字后面的 U。U 是 C 语言的类型标记，不是单位。
 * 时间单位：毫秒，1000 毫秒 = 1 秒；500 = 0.5 秒；5000 = 5 秒。
 * 四路电机共用下面的占空比和时间，修改后需要重新编译并下载到 STM32。
 *
 * 检测到目标后的动作顺序：
 * 打击 → 换向停顿 → 返回 → 打击后等待 → 接收下一条指令。
 * 实际机械方向取决于接线，方向调整在各区域的独立控制函数中完成。
 * 这里是按时间控制，逆转相同时间不保证机构精确回到原位。
 */
#define VISION_MOTOR_PWM 100U           // 电机占空比，当前 100 表示 100%，可填 0～100；0 为不输出。
                                     // 当前 TIM1 的 ARR=99，因此这里的数值对应百分比。
                                     // 占空比调大通常会提高驱动力/转速，不代表固定转速。
#define VISION_CW_MS 600U            // 打击阶段运行时间，当前 800 毫秒，即 0.8 秒；改 1000U 就是 1 秒。
#define VISION_CCW_MS 600U           // 返回阶段运行时间，当前 800 毫秒，即 0.8 秒；改 2000U 就是 2 秒。
#define VISION_REVERSE_PAUSE_MS 50U  // 换向前先停止输出，当前停 50 毫秒，即 0.05 秒，再开始返回。
#define VISION_RETRIGGER_DELAY_MS 50U // 一轮结束后等待，当前 50 毫秒，即 0.05 秒，再接收新区域指令。

/* ==================== 修改电机方向的位置 ====================
 * 打开同目录 vision_motor.c，找到对应区域的独立控制函数。
 * 第1区 B电机，Region1_Motor_Control，方向 PC2/PC3，PWM PE11。
 * 第2区 D电机，Region2_Motor_Control，方向 PD3/PD5，PWM PE14。
 * 第3区 C电机，Region3_Motor_Control，方向 PC4/PC5，PWM PE13。
 * 第4区 A电机，Region4_Motor_Control，方向 PC0/PC1，PWM PE9。
 * 每个函数都直接写出打击和返回时的高低电平，需要反向时交换两组电平。
 */

/* ==================== 视觉通信：通常不需要修改 ==================== */
#define VISION_TIMEOUT_MS 500U       // 超过 0.5 秒无合法帧，显示视觉离线；这不是打击时间。
                                    // 已经开始的正反转动作仍会执行完，不因视觉离线中断。
#define VISION_TREE_DX_LIMIT 2048     // 接受的树水平偏移范围为 -2048～2048 像素；另外允许 9999 表示无树。

/* 以下是程序运行时的数据和函数声明，不是在这里填写参数。 */
extern volatile uint8_t Vision_Region;          // 当前视觉区域：0 无目标，1～4 为对应区域。
extern volatile int32_t Vision_TreeDX;          // 树的水平偏移：左负右正；9999 表示无树或视觉离线。
extern volatile uint8_t Vision_Online;          // 视觉通信状态：1 正常，0 离线。
extern volatile uint32_t Vision_FrameCount;     // 累计解析成功的帧数，即 OLED 上的 RX。
extern volatile uint32_t Vision_ByteCount;      // 累计收到并交给解析器的字节数，排查串口时查看。
extern volatile uint32_t Vision_BadFrameCount;  // 累计丢弃的格式错误帧/超长数据次数。

extern volatile uint8_t Vision_MotorRegion;  // 本次动作锁定的电机：0 无，1=A、2=B、3=C、4=D。
extern volatile uint8_t Vision_MotorPhase;   // 动作阶段：0 等待指令，1 打击，2 换向停顿，3 返回，4 打击后等待。

void Vision_Motor_Init(void);                // 上电初始化，清除旧状态并停止全部电机输出。
void Vision_Motor_Task(void);                // 主循环反复调用，负责触发动作、计时和换向。
uint32_t Vision_Motor_RemainingMs(void);     // 读取当前阶段还剩多少毫秒，用于 OLED 倒计时。
uint8_t Vision_Motor_IsArmed(void);          // 是否允许开始新动作；完成后按设定间隔等待，再接收新指令。
void Vision_Motor_Receive(const uint8_t *data, uint16_t size); // 串口回调送入数据，支持一帧拆成多次接收。
void Vision_Motor_RxError(void);             // 串口出错时丢弃未接收完整的数据。

#endif
