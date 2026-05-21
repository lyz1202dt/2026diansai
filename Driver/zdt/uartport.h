#ifndef __UARTSEND_H__
#define __UARTSEND_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "Driver/uart/uart.h"
#include <stdint.h>
#include <string.h>

void MakeZDTSerialEnv( SerialHandle_t *handle);
void uart_SendCmd(uint8_t *cmd, uint32_t size);
void uart_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);

extern QueueHandle_t zdt_uart_send_semaphore;
extern QueueHandle_t zdt_uart_receive_semaphore;
extern uint8_t zdt_uart_recv_buf[128];
extern uint8_t zdt_last_recv_cnt;

#endif