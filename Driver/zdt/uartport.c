#include "uartport.h"

static QueueHandle_t zdt_uart_send_semaphore;
static QueueHandle_t zdt_uart_receive_semaphore;
static uint8_t zdt_uart_recv_buf[128];
static uint8_t zdt_last_recv_cnt=0;

static  SerialHandle_t *zdt_serial_handle;

void MakeZDTSerialEnv( SerialHandle_t *handle)
{
    zdt_uart_send_semaphore=xSemaphoreCreateBinary();
    zdt_uart_receive_semaphore=xSemaphoreCreateBinary();
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

void uart_recv_cb(uint8_t *data, uint16_t size, void* param)
{
    BaseType_t temp;
    xSemaphoreGiveFromISR(zdt_uart_receive_semaphore,&temp);
    portYIELD_FROM_ISR(temp);
}

void uart_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
    SerialReceiveIDLE(zdt_serial_handle, rxCmd, 128,uart_recv_cb);
  if(xSemaphoreTake(zdt_uart_receive_semaphore,pdMS_TO_TICKS(100))!=pdPASS)
  {
    *rxCount=0;
    return ;
  }
  
  *rxCount=zdt_last_recv_cnt;
  memcpy(rxCmd,zdt_uart_recv_buf,zdt_last_recv_cnt);
}
