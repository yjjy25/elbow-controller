#ifndef ELBOW_WORK_CONTROL_H
#define ELBOW_WORK_CONTROL_H
#include <stdint.h>
void Work_Control_Init(void);
void Work_Control_Task(uint32_t now, uint32_t frame_count);
#endif
