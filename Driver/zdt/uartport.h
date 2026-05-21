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


#endif