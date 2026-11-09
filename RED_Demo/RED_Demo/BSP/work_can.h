#ifndef ELBOW_WORK_CAN_H
#define ELBOW_WORK_CAN_H

#include "work_can_protocol.h"

void Work_CAN_SetRequest(uint8_t state, uint32_t task, uint8_t region, uint8_t result);
uint8_t Work_CAN_GetReply(uint32_t task);
uint8_t Work_CAN_Online(void);
uint8_t Work_CAN_Result(void);
uint8_t Work_CAN_Moving(void);

#endif
