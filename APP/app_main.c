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
#include "Lib/PID.h"
#include <ti/driverlib/dl_dma.h>

#include "config.h"
#include "mytask.h"


TaskHandle_t wheel_task_handle;
TaskHandle_t imu_task_handle;
TaskHandle_t line_track_task_handle;

extern bool enable_line_track;
extern float cur_robot_pos_x, cur_robot_pos_y;
extern float cur_robot_yaw, odom_yaw_offset;
extern float mpu6050_yaw;

int current_page=0;

//拉起所有任务，UI功能
int app_main() {

  SetupConfig();
  xTaskCreate(WheelTask, "wheel_task", 128, NULL, 5, &wheel_task_handle);
  xTaskCreate(IMUTask, "imu_task", 256, NULL, 5, &imu_task_handle);
  xTaskCreate(LineTrack, "line_track", 128, NULL, 4, &line_track_task_handle);

  OLED_Init();
  
  
  //SerialTransmit(g_serial, send_str, 6, send_done);
  //SerialReceive(g_serial, revb_str, 8, 100, NULL);
  BaseType_t last_wake_time=xTaskGetTickCount();
  while(1)
  {
    if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K1_PIN))
    {
      OLED_Printf(90, 0, 8, "k3-1");
      enable_line_track=1;
    }
    else if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K2_PIN))
    {
      OLED_Printf(90, 0, 8, "k3-2");
      enable_line_track=0;
    }
    else if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))   //里程计复位
    {
      OLED_Printf(90, 0, 8, "k3-3");
      cur_robot_pos_x=0.0f;
      cur_robot_pos_y=0.0f;
      cur_robot_yaw=0.0f;
      odom_yaw_offset=mpu6050_yaw;
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_UP_PIN)) //上
    {
      OLED_Printf(90, 0, 8, "k5-u");
      
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_DOWN_PIN))   //下
    {
      OLED_Printf(90, 0, 8, "k5-d");
      
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_LEFT_PIN))  //左
    {
      OLED_Printf(90, 0, 8, "k5-l");
      
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_RIGHT_PIN))    //右
    {
      OLED_Printf(90, 0, 8, "k5-r");
      
    }
    else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_MAIN_PIN))    //中键
    {
      OLED_Printf(90, 0, 8, "k5-m");
      
    }
    else {
      OLED_Printf(90, 0, 8, "    ");
    }
    //adc_get_success = GWGetState(adc_value);


    //OLED
    if(current_page==0)
    {
      OLED_Printf(0, 0, 8, "page0");
      OLED_Printf(0, 10, 8, "pos=(%.3f,%.3f)   ", cur_robot_pos_x,cur_robot_pos_y);
      OLED_Printf(0, 20, 8, "yaw=%.2f   ", cur_robot_yaw);
    }
    
    vTaskDelayUntil(&last_wake_time,100);
  }
  
  return 0;
}
