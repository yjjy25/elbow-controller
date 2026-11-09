#ifndef CAN_TEST_H
#define CAN_TEST_H
#include "main.h"
/* 0 waiting, 1 peer online, 2 init error, 3 TX error, 4 RX error, 5 peer timeout. */
extern volatile uint32_t CAN_Test_TxCount, CAN_Test_RxCount, CAN_Test_ErrorCount;
extern volatile uint32_t CAN_Test_LastHalError;
extern volatile uint8_t CAN_Test_State;
/* Names retained for the existing Keil project and diagnostic callers. */
void CAN_Test_Init(void);
void CAN_Test_Task(void);
/* Laser ports 1..4 = left, right, rear, front. Distances are raw millimetres. */
#define CAN_LASER_ID 0x601u
#define CAN_LASER_TIMEOUT_MS 300u
#define CAN_LASER_MAX_MM 15000u
/* Read status: 0 waiting, 1 valid, 2 out of range, 3 frame timeout. */
uint8_t CAN_Laser_Read(uint8_t channel, uint16_t *distance_mm);
#endif
