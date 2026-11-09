#ifndef __FUN_H
#define __FUN_H

#include "headfile.h"

/* 下标 0~3 对应 A~D；计数单位为脉冲/10ms，速度单位为 RPM。 */
extern volatile int32_t Encoder_Count[4];
extern volatile int32_t Speed[4];

void Motor_Statu(uint8_t mode);
void LED_Blink(uint32_t time);
void Init(void);
void OLED_PowerOn(void);
void OLED_Display(void);
/* 独立串口上报任务，在主循环中持续调用，内部按 500ms 限频。 */
void UART_SendStatus(void);
void Vision_UART_Task(void);
extern volatile uint32_t Vision_UART_ErrorCount, Vision_UART_LastError;
extern volatile uint32_t Vision_UART_ArmErrorCount;


#endif



