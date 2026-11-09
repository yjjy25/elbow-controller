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
#endif
