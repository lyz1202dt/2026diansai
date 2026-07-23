#include "CLI/App/port.h"

#include "projdefs.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <task.h>

/* 使用 driverlib 的 DMA 接口 */
#include "Driver/uart/uart.h"
#include <ti/driverlib/dl_dma.h>

uint8_t send_str[8] = {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t revb_str[8] = {};
static SerialHandle_t *g_serial;

void send_done(void *param) {}

void recv_done(void *param) {}

int app_main() {
  g_serial = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);
  NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
  SerialTransmit(g_serial, send_str, 6, send_done);
  SerialReceive(g_serial, revb_str, 8, 100, NULL);

  while (1) {
    SerialTransmit(g_serial, send_str, 3, send_done);
    vTaskDelay(10);
  }
  return 0;
}

void UART_0_INST_IRQHandler(void) {
    SerialIRQ(g_serial);
}

// void UART_1_INST_IRQHandler(void) {
//     //SerialIRQ(g_serial);
// }

// void UART_2_INST_IRQHandler(void) {
//     //SerialIRQ(g_serial);
// }