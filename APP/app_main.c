/* 示例：使用 SysConfig 生成的 DMA 通道做持续接收（缓冲区）
   并每 100ms 通过 DMA 发送一次 "Hello World"。
   说明：此示例假定 SYSCFG_DL_init() 在系统启动时或此处调用过，
   并且使用的 driverlib API 在工程 include path 中可用。 */

#include <stdint.h>
#include <string.h>
#include "projdefs.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* 使用 driverlib 的 DMA 接口 */
#include <ti/driverlib/dl_dma.h>
#include "Driver/uart/uart.h"
#include "Driver/zdt/uartport.h"
#include "Driver/zdt/Emm_V5.h"

char send_buf[24];
char recv_buf[29];
static SerialHandle_t *g_serial;

TaskHandle_t handle;

int32_t exp_velocity=0;
uint16_t dev_addr=0x01;
uint8_t zdt_motor_buffer[64];
uint8_t recv_count;

float current_vel,current_pos;


void MotorTask(void*param)
{
    
}



int app_main()
{
    g_serial = SerialInit(UART_0_INST,SERIAL_MODE_IT, NULL, NULL);
    SerialConfigDMA(g_serial, DMA_CH1_CHAN_ID, DMA_CH0_CHAN_ID);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    vTaskDelay(pdMS_TO_TICKS(1000));
    MakeZDTSerialEnv(g_serial);
    Emm_V5_Modify_Ctrl_Mode(0x01, 1, 2);
    vTaskDelay(pdMS_TO_TICKS(2));
	Emm_V5_En_Control(0x01, 1, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    TickType_t last_wake_time=xTaskGetTickCount();
    while(1)
    {
        //SerialTransmit(g_serial, (uint8_t *)send_buf, 19, send_cb);
        Emm_V5_Read_Sys_Params(dev_addr, S_CPOS);
        uart_Receive_Data(zdt_motor_buffer, &recv_count);
        Emm_V5_GetPos(dev_addr,zdt_motor_buffer,&current_pos);
        Emm_V5_Vel_Control(dev_addr, (exp_velocity>=0.0f?0:1), ABS(exp_velocity), 0, 0);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
    return 0;
}


void UART_0_INST_IRQHandler(void)
{
    SerialIRQ(g_serial);
}
