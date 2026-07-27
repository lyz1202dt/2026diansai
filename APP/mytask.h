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

#endif