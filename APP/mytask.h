#ifndef __MYTASK_H__
#define __MYTASK_H__

#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <task.h>

void WheelTask(void *param);
void IMUTask(void *param);
void LineTrack(void* param);
void ZDTDriver(void* param);
void VOFA_Task(void* param);

#endif