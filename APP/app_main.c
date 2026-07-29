// #include "CLI/App/port.h"

#include "projdefs.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <task.h>

/* 使用 driverlib 的 DMA 接口 */
#include "Bsp/OLED.h"
#include "Bsp/gw_model.h"
#include "Bsp/motor.h"
#include "Bsp/mpu6050.h"
#include "Driver/uart/uart.h"
#include "Lib/pid/PID.h"
#include <ti/driverlib/dl_dma.h>

#include "config.h"
#include "mytask.h"


TaskHandle_t wheel_task_handle;
TaskHandle_t imu_task_handle;
TaskHandle_t line_track_task_handle;
TaskHandle_t zdt_driver_task_handle;
TaskHandle_t k230_recv_task_handle;
TaskHandle_t test_task_handle;
TaskHandle_t vofa_comm_task_handle;

TaskHandle_t task_x_handle;

extern bool enable_line_track;
extern float cur_robot_pos_x, cur_robot_pos_y;
extern float cur_robot_yaw, odom_yaw_offset;
extern float mpu6050_yaw;
extern float sum_distance;

uint8_t k230_cmd=0;

int current_task_id=0;
int last_task_id=0;
int current_select_task_id=0;

bool task_running=false;
bool force_exit=false;

//拉起所有任务，UI功能
int app_main() {

  SetupConfig();
  OLED_Init();
  xTaskCreate(WheelTask, "wheel_task", 128, NULL, 5, &wheel_task_handle);
  xTaskCreate(IMUTask, "imu_task", 256, NULL, 5, &imu_task_handle);
  xTaskCreate(LineTrack, "line_track", 128, NULL, 4, &line_track_task_handle);
  xTaskCreate(ZDTDriver,"zdt_driver",128,NULL, 4,&zdt_driver_task_handle);
  xTaskCreate(K230RecvTask,"k230_recv",256,NULL, 4,&k230_recv_task_handle);
  xTaskCreate(TestTask, "test_task",128, NULL, 2, &test_task_handle);
  //xTaskCreate(VOFA_Task,"vofa",512,NULL, 1,&vofa_comm_task_handle);
  
  
  //SerialTransmit(zdt_serial, send_str, 6);
  //SerialReceive(zdt_serial, recv_str, 8, 100);
  BaseType_t last_wake_time=xTaskGetTickCount();
  while(1)
  {
    if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K1_PIN))   //任务清零
    {
      OLED_Printf(80, 0, 8, "clear");
      //enable_line_track=1;
      current_task_id=0;
      force_exit=true;
    }
    else if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K2_PIN))  //确认
    {
      OLED_Printf(80, 0, 8, "ok");
      //enable_line_track=0;
      current_task_id=current_select_task_id;
    }
    // else if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))   //里程计复位
    // {
    //   OLED_Printf(90, 0, 8, "k3-3");
    //   cur_robot_pos_x=0.0f;
    //   cur_robot_pos_y=0.0f;
    //   cur_robot_yaw=0.0f;
    //   odom_yaw_offset=mpu6050_yaw;
    // }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_UP_PIN)) //第一题
    {
      OLED_Printf(80, 0, 8, "task1");
      current_select_task_id=1;
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_DOWN_PIN))   //第二题
    {
      OLED_Printf(80, 0, 8, "task2");
      current_select_task_id=2;
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_LEFT_PIN))  //第三题
    {
      OLED_Printf(80, 0, 8, "task3");
      current_select_task_id=3;
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_RIGHT_PIN))    //第四题
    {
      OLED_Printf(80, 0, 8, "task4");
      current_select_task_id=4;
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_MAIN_PIN))    //第五题
    {
      OLED_Printf(80, 0, 8, "task5");
      current_select_task_id=5;
    }
    else {
      OLED_Printf(80, 0, 8, "     ");
    }
    //OLED
    //OLED_Printf(0, 10, 8, "pos=(%.3f,%.3f)   ", cur_robot_pos_x,cur_robot_pos_y);
    //OLED_Printf(0, 20, 8, "yaw=%.2f   ", cur_robot_yaw);
    OLED_Printf(0, 20, 8, "s=%.3f   ",sum_distance);
    OLED_Printf(0, 0, 16, "oled");




    if(current_task_id==1&&task_running==false)      //当前是第一题
    {
        force_exit=false;
        task_running=true;
        xTaskCreate(Task1, "task1", 512, NULL, 2, &task_x_handle);
    }
    else if(current_task_id==2&&task_running==false)      //当前是第二题
    {
        force_exit=false;
        task_running=true;
        xTaskCreate(Task2, "task2", 512, NULL, 2, &task_x_handle);
    }
    else if(current_task_id==3&&task_running==false)      //当前是第三题
    {
        force_exit=false;
        task_running=true;
        xTaskCreate(Task3, "task3", 512, NULL, 2, &task_x_handle);
    }
    SerialTransmit(g_serial, &k230_cmd, 1);         //更新K230状态
    vTaskDelayUntil(&last_wake_time,50);
  }
  
  return 0;
}
