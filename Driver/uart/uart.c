#include "Driver/uart/uart.h"

#include <stddef.h>
#include <ti/driverlib/dl_uart_main.h>

#define SERIAL_UART_RX_INTERRUPTS                                             \
    (DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR)

#define SERIAL_UART_TX_INTERRUPTS                                             \
    (DL_UART_MAIN_INTERRUPT_TX | DL_UART_MAIN_INTERRUPT_EOT_DONE)

#define SERIAL_UART_ERROR_INTERRUPTS                                          \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | DL_UART_MAIN_INTERRUPT_BREAK_ERROR |\
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR | DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |\
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

static void SerialTxFillFifo(SerialHandle_t *handle)
{
    while ((handle->tx_index < handle->tx_length) &&
           !DL_UART_Main_isTXFIFOFull(handle->hardware)) {
        DL_UART_Main_transmitData(handle->hardware,
                                  handle->send_buffer[handle->tx_index]);
        handle->tx_index++;
    }
}

static void SerialRxDrainFifo(SerialHandle_t *handle)
{
    while ((handle->recv_count < handle->recv_expected) &&
           !DL_UART_Main_isRXFIFOEmpty(handle->hardware)) {
        handle->recv_buffer[handle->recv_count] =
            DL_UART_Main_receiveData(handle->hardware);
        handle->recv_count++;
    }
}

static void SerialNotifyError(SerialHandle_t *handle, int err_code)
{
    if ((handle != NULL) && (handle->error_cb != NULL)) {
        handle->error_cb(err_code, handle->param);
    }
}

SerialHandle_t* SerialInit(UART_Regs *hw_uart, uint8_t mode, ErrorCb error_cb, void* param)
{
    SerialHandle_t *handle;

    if ((hw_uart == NULL) || (mode != SERIAL_MODE_IT)) {
        return NULL;
    }

    handle = (SerialHandle_t *) pvPortMalloc(sizeof(SerialHandle_t));
    if (handle == NULL) {
        return NULL;
    }

    handle->hardware = hw_uart;
    handle->send_buffer = NULL;
    handle->recv_buffer = NULL;
    handle->send_buffer_size = 0U;
    handle->recv_buffer_size = 0U;
    handle->tx_length = 0U;
    handle->tx_index = 0U;
    handle->recv_count = 0U;
    handle->recv_expected = 0U;
    handle->mode = (SerialMode_t) mode;
    handle->tx_state = SERIAL_STATE_READY;
    handle->rx_state = SERIAL_STATE_READY;
    handle->error_cb = error_cb;
    handle->param = param;

    handle->tx_sem = xSemaphoreCreateBinary();
    handle->rx_sem = xSemaphoreCreateBinary();
    if ((handle->tx_sem == NULL) || (handle->rx_sem == NULL)) {
        if (handle->tx_sem != NULL) {
            vSemaphoreDelete(handle->tx_sem);
        }
        if (handle->rx_sem != NULL) {
            vSemaphoreDelete(handle->rx_sem);
        }
        vPortFree(handle);
        return NULL;
    }

    DL_UART_Main_disableInterrupt(hw_uart, SERIAL_UART_RX_INTERRUPTS |
                                               SERIAL_UART_TX_INTERRUPTS);
    DL_UART_Main_clearInterruptStatus(hw_uart, SERIAL_UART_RX_INTERRUPTS |
                                               SERIAL_UART_TX_INTERRUPTS |
                                               SERIAL_UART_ERROR_INTERRUPTS);
    DL_UART_Main_enableInterrupt(hw_uart, SERIAL_UART_ERROR_INTERRUPTS);

    return handle;
}

int SerialTransmit(SerialHandle_t* handle, uint8_t *data, uint16_t size)
{
    if ((handle == NULL) || (data == NULL) || (size == 0U) ||
        (handle->mode != SERIAL_MODE_IT)) {
        return SERIAL_ERR_INVALID;
    }

    taskENTER_CRITICAL();
    if (handle->tx_state != SERIAL_STATE_READY) {
        taskEXIT_CRITICAL();
        return SERIAL_ERR_BUSY;
    }

    handle->send_buffer = data;
    handle->send_buffer_size = size;
    handle->tx_length = size;
    handle->tx_index = 0U;
    handle->tx_state = SERIAL_STATE_BUSY;
    (void) xSemaphoreTake(handle->tx_sem, 0U);

    DL_UART_Main_clearInterruptStatus(handle->hardware,
                                      SERIAL_UART_TX_INTERRUPTS);
    SerialTxFillFifo(handle);
    if (handle->tx_index < handle->tx_length) {
        DL_UART_Main_enableInterrupt(handle->hardware,
                                     DL_UART_MAIN_INTERRUPT_TX);
    } else {
        DL_UART_Main_enableInterrupt(handle->hardware,
                                     DL_UART_MAIN_INTERRUPT_EOT_DONE);
    }
    taskEXIT_CRITICAL();

    (void) xSemaphoreTake(handle->tx_sem, portMAX_DELAY);

    return size;
}

int SerialReceive(SerialHandle_t* handle, uint8_t *data, uint16_t size,int timeout)
{
    TickType_t ticks;
    BaseType_t sem_ret;
    uint16_t received;

    if ((handle == NULL) || (data == NULL) || (size == 0U) ||
        (handle->mode != SERIAL_MODE_IT)) {
        return SERIAL_ERR_INVALID;
    }

    taskENTER_CRITICAL();
    if (handle->rx_state != SERIAL_STATE_READY) {
        taskEXIT_CRITICAL();
        return SERIAL_ERR_BUSY;
    }

    handle->recv_buffer = data;
    handle->recv_buffer_size = size;
    handle->recv_expected = size;
    handle->recv_count = 0U;
    handle->rx_state = SERIAL_STATE_BUSY;
    (void) xSemaphoreTake(handle->rx_sem, 0U);

    DL_UART_Main_clearInterruptStatus(handle->hardware,
                                      SERIAL_UART_RX_INTERRUPTS);
    SerialRxDrainFifo(handle);
    if (handle->recv_count >= handle->recv_expected) {
        handle->rx_state = SERIAL_STATE_READY;
        (void) xSemaphoreGive(handle->rx_sem);
    } else {
        DL_UART_Main_enableInterrupt(handle->hardware,
                                     SERIAL_UART_RX_INTERRUPTS);
    }
    taskEXIT_CRITICAL();

    ticks = (timeout < 0) ? portMAX_DELAY : pdMS_TO_TICKS((uint32_t) timeout);
    sem_ret = xSemaphoreTake(handle->rx_sem, ticks);

    taskENTER_CRITICAL();
    received = handle->recv_count;
    if (sem_ret != pdTRUE) {
        DL_UART_Main_disableInterrupt(handle->hardware,
                                      SERIAL_UART_RX_INTERRUPTS);
        handle->rx_state = SERIAL_STATE_READY;
    }
    taskEXIT_CRITICAL();

    if (sem_ret == pdTRUE) {
        return received;
    }

    return (received > 0U) ? (int) received : SERIAL_ERR_TIMEOUT;
}

void SerialIRQ(SerialHandle_t* handle)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    bool done = false;

    if (handle == NULL) {
        return;
    }

    while (1) {
        switch (DL_UART_Main_getPendingInterrupt(handle->hardware)) {
        case DL_UART_MAIN_IIDX_RX:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            if (handle->rx_state == SERIAL_STATE_BUSY) {
                SerialRxDrainFifo(handle);
                if (handle->recv_count >= handle->recv_expected) {
                    handle->rx_state = SERIAL_STATE_READY;
                    DL_UART_Main_disableInterrupt(handle->hardware,
                                                  SERIAL_UART_RX_INTERRUPTS);
                    xSemaphoreGiveFromISR(handle->rx_sem,
                                          &higher_priority_task_woken);
                }
            } else {
                while (!DL_UART_Main_isRXFIFOEmpty(handle->hardware)) {
                    (void) DL_UART_Main_receiveData(handle->hardware);
                }
            }
            break;

        case DL_UART_MAIN_IIDX_TX:
            if (handle->tx_state == SERIAL_STATE_BUSY) {
                SerialTxFillFifo(handle);
                if (handle->tx_index >= handle->tx_length) {
                    DL_UART_Main_disableInterrupt(handle->hardware,
                                                  DL_UART_MAIN_INTERRUPT_TX);
                    DL_UART_Main_clearInterruptStatus(
                        handle->hardware, DL_UART_MAIN_INTERRUPT_EOT_DONE);
                    DL_UART_Main_enableInterrupt(handle->hardware,
                                                 DL_UART_MAIN_INTERRUPT_EOT_DONE);
                }
            } else {
                DL_UART_Main_disableInterrupt(handle->hardware,
                                              DL_UART_MAIN_INTERRUPT_TX);
            }
            break;

        case DL_UART_MAIN_IIDX_EOT_DONE:
            DL_UART_Main_disableInterrupt(handle->hardware,
                                          DL_UART_MAIN_INTERRUPT_EOT_DONE);
            if (handle->tx_state == SERIAL_STATE_BUSY) {
                handle->tx_state = SERIAL_STATE_READY;
                xSemaphoreGiveFromISR(handle->tx_sem,
                                      &higher_priority_task_woken);
            }
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            SerialNotifyError(handle, DL_UART_MAIN_IIDX_OVERRUN_ERROR);
            break;

        case DL_UART_MAIN_IIDX_BREAK_ERROR:
            SerialNotifyError(handle, DL_UART_MAIN_IIDX_BREAK_ERROR);
            break;

        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            SerialNotifyError(handle, DL_UART_MAIN_IIDX_PARITY_ERROR);
            break;

        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            SerialNotifyError(handle, DL_UART_MAIN_IIDX_FRAMING_ERROR);
            break;

        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            SerialNotifyError(handle, DL_UART_MAIN_IIDX_NOISE_ERROR);
            break;

        case DL_UART_MAIN_IIDX_NO_INTERRUPT:
        default:
            done = true;
            break;
        }

        if (done) {
            break;
        }
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

int SerialDeinit(SerialHandle_t* handle)
{
    if (handle == NULL) {
        return SERIAL_ERR_INVALID;
    }

    DL_UART_Main_disableInterrupt(handle->hardware, SERIAL_UART_RX_INTERRUPTS |
                                                SERIAL_UART_TX_INTERRUPTS |
                                                SERIAL_UART_ERROR_INTERRUPTS);
    DL_UART_Main_clearInterruptStatus(handle->hardware, SERIAL_UART_RX_INTERRUPTS |
                                                SERIAL_UART_TX_INTERRUPTS |
                                                SERIAL_UART_ERROR_INTERRUPTS);

    if (handle->tx_sem != NULL) {
        vSemaphoreDelete(handle->tx_sem);
    }
    if (handle->rx_sem != NULL) {
        vSemaphoreDelete(handle->rx_sem);
    }

    handle->tx_state = SERIAL_STATE_RESET;
    handle->rx_state = SERIAL_STATE_RESET;
    vPortFree(handle);

    return SERIAL_OK;
}
