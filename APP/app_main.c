#include "CLI/App/port.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "projdefs.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* 使用 driverlib 的 DMA 接口 */
#include <ti/driverlib/dl_dma.h>
#include "Driver/uart/uart.h"

uint8_t send_str[8]={1,2,3,4,5,6,7,8};
uint8_t revb_str[8]={};
static SerialHandle_t *g_serial;

void send_done(void* param)
{

}

void recv_done(void* param)
{

}


int app_main()
{
    // if (CLI_EnvInit() != 0) {
    //     return -1;
    // }
    g_serial = SerialInit(UART_0_INST, SERIAL_MODE_DMA, NULL, NULL);
    if ((g_serial == NULL) ||
        (SerialConfigDMA(g_serial, DMA_CH1_CHAN_ID, DMA_CH0_CHAN_ID) != SERIAL_OK)) {
        return -1;
    }
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);


    // if (CLI_Port_Init(g_serial) != 0) {
    //     return -1;
    // }
    SerialTransmit(g_serial, send_str, 6, send_done);
    SerialReceiveIDLE(g_serial, revb_str, sizeof(revb_str), NULL);

    return 0;
}
