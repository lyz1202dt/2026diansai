#ifndef __MYTASK_H__
#define __MYTASK_H__

#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <task.h>

#pragma pack(1)

typedef struct{
    uint8_t head;    //0x5A
    float position;
    uint8_t check;  //和校验
}RecvPack;

#pragma pack()


//任务函数
void WheelTask(void *param);
void IMUTask(void *param);
void LineTrack(void* param);
void ZDTDriver(void* param);
void VOFA_Task(void* param);
void K230RecvTask(void* param);


//工具函数
bool is_line(uint16_t value);



//赛题任务
void Task1(void* parma);
void Task2(void* parma);
void Task3(void* parma);

void TestTask(void* param);

#endif