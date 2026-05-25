#include "uartport.h"

static QueueHandle_t zdt_uart_send_semaphore;

static  SerialHandle_t *zdt_serial_handle;

void MakeZDTSerialEnv( SerialHandle_t *handle)
{
    zdt_uart_send_semaphore=xSemaphoreCreateBinary();
    zdt_serial_handle=handle;
}

static void send_notify_cb(void* param)
{
    BaseType_t temp;
    xSemaphoreGiveFromISR(zdt_uart_send_semaphore,&temp);
    portYIELD_FROM_ISR(temp);
}

void uart_SendCmd(uint8_t *cmd, uint32_t size)
{
    xSemaphoreTake(zdt_uart_send_semaphore,0);
    SerialTransmit(zdt_serial_handle, cmd, size, send_notify_cb);
    xSemaphoreTake(zdt_uart_send_semaphore,portMAX_DELAY);
}

void uart_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
    int received;

    if (rxCount != NULL) {
        *rxCount = 0;
    }

    if ((rxCmd == NULL) || (rxCount == NULL) || (zdt_serial_handle == NULL)) {
        return;
    }

    received = SerialReceiveIDLE(zdt_serial_handle, rxCmd, 128, NULL);
    if (received < 0) {
        *rxCount = 0;
        return;
    }

    *rxCount = (uint8_t) received;
}
