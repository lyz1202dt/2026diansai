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

int app_main() {

  SetupConfig();
  xTaskCreate(WheelTask, "wheel_task", 128, NULL, 5, &wheel_task_handle);
  xTaskCreate(IMUTask, "imu_task", 256, NULL, 5, &imu_task_handle);
  xTaskCreate(LineTrack, "line_track", 128, NULL, 4, &line_track_task_handle);
  
  //SerialTransmit(g_serial, send_str, 6, send_done);
  //SerialReceive(g_serial, revb_str, 8, 100, NULL);
  while(1)
  {
    if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K1_PIN))
    {
      enable_line_track=1;
    }
    if(!DL_GPIO_readPins(KEY3_PORT, KEY3_K2_PIN))
    {
      enable_line_track=0;
    }
    //adc_get_success = GWGetState(adc_value);
    vTaskDelay(50);
  }
  
  return 0;
}
