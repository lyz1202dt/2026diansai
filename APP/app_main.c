/* 示例：使用 SysConfig 生成的 DMA 通道做持续接收（缓冲区）
   并每 100ms 通过 DMA 发送一次 "Hello World"。
   说明：此示例假定 SYSCFG_DL_init() 在系统启动时或此处调用过，
   并且使用的 driverlib API 在工程 include path 中可用。 */

#include <stdint.h>
#include <string.h>
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* 使用 driverlib 的 DMA 接口 */
#include <ti/driverlib/dl_dma.h>
#include "Driver/uart/uart.h"

char send_buf[24];
char recv_buf[29];
static SerialHandle_t *g_serial;

TaskHandle_t handle;

int ret;


int debug_rcnt;
int debug_tcnt;
void recv_cb(uint8_t *data, uint16_t size, void* param)
{
    debug_rcnt++;
}

void send_cb(void* param)
{
    debug_tcnt++;
}

void SerialSendTask(void*param)
{
    while(1)
    {
        SerialTransmit(g_serial, (uint8_t *)send_buf, 19, send_cb);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}



int app_main()
{
    g_serial = SerialInit(UART_0_INST,SERIAL_MODE_IT, NULL, NULL);
    SerialConfigDMA(g_serial, DMA_CH1_CHAN_ID, DMA_CH0_CHAN_ID);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    xTaskCreate(SerialSendTask, "task_name", 128, NULL, 3, & handle);

    for(int i=0;i<24;i++)
    {
        send_buf[i]=i;
    }

    while(1)
    {
        ret=SerialReceiveIDLE(g_serial, (uint8_t *) recv_buf, 25, recv_cb);
    }
    return 0;
}


void UART_0_INST_IRQHandler(void)
{
    SerialIRQ(g_serial);
}
