#include "uartport.h"

static SerialHandle_t *zdt_serial_handle;

void MakeZDTSerialEnv( SerialHandle_t *handle)
{
    zdt_serial_handle = handle;
}

void uart_SendCmd(uint8_t *cmd, uint32_t size)
{
    if ((cmd == NULL) || (zdt_serial_handle == NULL) || (size == 0U)) {
        return;
    }

    (void) SerialTransmit(zdt_serial_handle, cmd, (uint16_t) size, NULL);
}

void uart_Receive_Data(uint8_t *rxCmd, uint8_t exp_cnt,uint8_t *rxCount)
{
    int received;

    if (rxCount != NULL) {
        *rxCount = 0;
    }

    if ((rxCmd == NULL) || (rxCount == NULL) || (zdt_serial_handle == NULL)) {
        return;
    }

    received = SerialReceive(zdt_serial_handle, rxCmd, exp_cnt,10,NULL);
    if (received < 0) {
        *rxCount = 0;
        return;
    }

    *rxCount = (uint8_t) received;
}
