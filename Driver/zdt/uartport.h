#ifndef __UARTSEND_H__
#define __UARTSEND_H__

#include "Driver/uart/uart.h"
#include <stdint.h>

void MakeZDTSerialEnv( SerialHandle_t *handle);
void uart_SendCmd(uint8_t *cmd, uint32_t size);
void uart_Receive_Data(uint8_t *rxCmd,uint8_t exp_cnt, uint8_t *rxCount);


#endif
