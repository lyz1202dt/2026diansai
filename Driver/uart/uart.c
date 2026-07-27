/**
 * @file uart.c
 * @brief 可复用串口驱动库实现
 * 支持中断/DMA/轮询三种工作模式
 * 兼容 FreeRTOS 和裸机环境
 */

#include "Driver/uart/uart.h"
#include <stdlib.h>
#include <string.h>
#include "ti_msp_dl_config.h"
#include <ti/driverlib/dl_uart.h>
#include <ti/driverlib/driverlib.h>

/* ==================== 内部辅助函数 ==================== */

#define SERIAL_DMA_CHANNEL_INVALID (0xFFU)
#define SERIAL_ERROR_NONE          (0x00000000UL)
#define SERIAL_ERROR_OVERRUN       (0x00000001UL)
#define SERIAL_ERROR_BREAK         (0x00000002UL)
#define SERIAL_ERROR_PARITY        (0x00000004UL)
#define SERIAL_ERROR_FRAMING       (0x00000008UL)
#define SERIAL_ERROR_NOISE         (0x00000010UL)
#define SERIAL_IRQ_FIFO_BUDGET     (32U)
#define SERIAL_IRQ_EVENT_BUDGET    (16U)

/* ==================== 调试观测变量 ==================== */

/*
 * 如需继续使用 J-Link 观察接收过程，可把下面的 `#if 0`
 * 改成 `#if 1` 恢复调试变量与记录逻辑。
 */
#if 0
#define UART_DBG_RX_FINISH_NONE         (0U)
#define UART_DBG_RX_FINISH_RX_FULL      (1U)
#define UART_DBG_RX_FINISH_RX_TIMEOUT   (2U)
#define UART_DBG_RX_FINISH_DMA_DONE_RX  (3U)
#define UART_DBG_RX_FINISH_CANCEL       (4U)

volatile uint32_t uart_dbg_irq_rx_count = 0U;
volatile uint32_t uart_dbg_irq_rx_timeout_count = 0U;
volatile uint32_t uart_dbg_irq_dma_done_rx_count = 0U;
volatile uint32_t uart_dbg_irq_last_iidx = 0U;
volatile uint32_t uart_dbg_rx_finish_reason = UART_DBG_RX_FINISH_NONE;
volatile uint32_t uart_dbg_rx_finish_count = 0U;
volatile uint32_t uart_dbg_rx_finish_total_count = 0U;
volatile uint32_t uart_dbg_last_timeout_recv_count = 0U;
volatile uint32_t uart_dbg_last_finish_input_count = 0U;
volatile uint32_t uart_dbg_last_finish_recv_count = 0U;
volatile uint32_t uart_dbg_idle_return_count = 0U;

static uint32_t g_uart_dbg_pending_finish_reason = UART_DBG_RX_FINISH_NONE;

static void serial_dbg_set_finish_reason(uint32_t reason)
{
    g_uart_dbg_pending_finish_reason = reason;
}

static void serial_dbg_commit_finish(uint16_t count)
{
    uart_dbg_last_finish_input_count = count;
    uart_dbg_rx_finish_reason = g_uart_dbg_pending_finish_reason;
    uart_dbg_rx_finish_count = count;
    uart_dbg_rx_finish_total_count++;
    g_uart_dbg_pending_finish_reason = UART_DBG_RX_FINISH_NONE;
}
#else
#define UART_DBG_RX_FINISH_RX_FULL      (1U)
#define UART_DBG_RX_FINISH_RX_TIMEOUT   (2U)
#define UART_DBG_RX_FINISH_DMA_DONE_RX  (3U)
#define UART_DBG_RX_FINISH_CANCEL       (4U)
#define serial_dbg_set_finish_reason(reason) ((void) (reason))
#define serial_dbg_commit_finish(count)      ((void) (count))
#endif

/**
 * @brief 分配内存 (兼容有无 FreeRTOS)
 */
static void* serial_malloc(size_t size)
{
    #if defined(configTOTAL_HEAP_SIZE)
        return pvPortMalloc(size);
    #else
        return malloc(size);
    #endif
}

/**
 * @brief 释放内存 (兼容有无 FreeRTOS)
 */
static void serial_free(void *ptr)
{
    #if defined(configTOTAL_HEAP_SIZE)
        vPortFree(ptr);
    #else
        free(ptr);
    #endif
}

/**
 * @brief 获取信号量 (带超时)
 */
static int serial_sem_take(SemaphoreHandle_t sem, int timeout_ms)
{
    #if defined(configTOTAL_HEAP_SIZE)
        TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : 
                           pdMS_TO_TICKS(timeout_ms);
        return xSemaphoreTake(sem, ticks) == pdTRUE ? 0 : SERIAL_ERR_TIMEOUT;
    #else
        return 0;
    #endif
}

/**
 * @brief 释放信号量
 */
static void serial_sem_give(SemaphoreHandle_t sem)
{
    #if defined(configTOTAL_HEAP_SIZE)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    #endif
}

/**
 * @brief 清空信号量的历史状态
 */
static void serial_sem_reset(SemaphoreHandle_t sem)
{
    #if defined(configTOTAL_HEAP_SIZE)
        if (sem != NULL) {
            while (xSemaphoreTake(sem, 0) == pdTRUE) {
            }
        }
    #else
        (void) sem;
    #endif
}

/**
 * @brief 启用 UART 接收中断
 */
static void serial_enable_rx_it(UART_Regs *uart)
{
    DL_UART_enableInterrupt(uart, DL_UART_INTERRUPT_RX);
}

/**
 * @brief 禁用 UART 接收中断
 */
static void serial_disable_rx_it(UART_Regs *uart)
{
    DL_UART_disableInterrupt(uart, DL_UART_INTERRUPT_RX);
}

/**
 * @brief 启用 UART 发送中断
 */
static void serial_enable_tx_it(UART_Regs *uart)
{
    DL_UART_enableInterrupt(uart, DL_UART_INTERRUPT_TX);
}

/**
 * @brief 禁用 UART 发送中断
 */
static void serial_disable_tx_it(UART_Regs *uart)
{
    DL_UART_disableInterrupt(uart, DL_UART_INTERRUPT_TX);
}

/**
 * @brief 当前是否存在活跃接收事务
 */
static bool serial_rx_transaction_active(const SerialHandle_t *handle)
{
    if (handle == NULL) {
        return false;
    }

    return (handle->rxState == SERIAL_STATE_BUSY) &&
           (handle->recv_expected > 0U);
}

/**
 * @brief 当前是否存在活跃发送事务
 */
static bool serial_tx_transaction_active(const SerialHandle_t *handle)
{
    if (handle == NULL) {
        return false;
    }

    return (handle->gState == SERIAL_STATE_BUSY) &&
           (handle->tx_length > 0U);
}

/**
 * @brief 读空 UART FIFO，丢弃当前残留输入
 */
static void serial_flush_rx_fifo(UART_Regs *uart)
{
    if (uart == NULL) {
        return;
    }

    while (DL_UART_isRXFIFOEmpty(uart) == false) {
        (void) DL_UART_receiveData(uart);
    }
}

/**
 * @brief 有上界地丢弃 RX FIFO 中的数据，供中断上下文使用
 */
static void serial_flush_rx_fifo_bounded(UART_Regs *uart, uint16_t budget)
{
    uint16_t flushed = 0U;

    if (uart == NULL) {
        return;
    }

    while ((flushed < budget) && (DL_UART_isRXFIFOEmpty(uart) == false)) {
        (void) DL_UART_receiveData(uart);
        flushed++;
    }
}

/**
 * @brief 把当前 FIFO 中已经到达的数据吸收到接收缓冲
 * @return 新吸收的字节数
 */
static uint16_t serial_drain_rx_fifo(SerialHandle_t *handle, bool *overflow)
{
    uint16_t drained = 0U;

    if ((handle == NULL) || (handle->hardware == NULL)) {
        return 0U;
    }

    while (DL_UART_isRXFIFOEmpty(handle->hardware) == false) {
        uint8_t byte = DL_UART_receiveData(handle->hardware);

        if (handle->recv_count < handle->recv_expected) {
            handle->recv_buffer[handle->recv_count++] = byte;
            drained++;
        } else {
            if (overflow != NULL) {
                *overflow = true;
            }
        }
    }

    return drained;
}

/**
 * @brief 有上界地吸收 RX FIFO，供中断上下文使用
 */
static uint16_t serial_drain_rx_fifo_bounded(
    SerialHandle_t *handle, bool *overflow, uint16_t budget)
{
    uint16_t drained = 0U;

    if ((handle == NULL) || (handle->hardware == NULL)) {
        return 0U;
    }

    while ((drained < budget) &&
           (DL_UART_isRXFIFOEmpty(handle->hardware) == false)) {
        uint8_t byte = DL_UART_receiveData(handle->hardware);

        if (handle->recv_count < handle->recv_expected) {
            handle->recv_buffer[handle->recv_count++] = byte;
            drained++;
        } else if (overflow != NULL) {
            *overflow = true;
        }
    }

    return drained;
}

/**
 * @brief 有上界地向 TX FIFO 填充数据，供中断上下文使用
 */
static uint16_t serial_fill_tx_fifo_bounded(
    SerialHandle_t *handle, uint16_t budget)
{
    uint16_t pushed = 0U;

    if ((handle == NULL) || (handle->hardware == NULL)) {
        return 0U;
    }

    while ((pushed < budget) &&
           (handle->tx_index < handle->tx_length) &&
           (DL_UART_isTXFIFOFull(handle->hardware) == false)) {
        DL_UART_transmitData(handle->hardware,
            handle->send_buffer[handle->tx_index++]);
        pushed++;
    }

    return pushed;
}

/**
 * @brief 根据当前事务模式配置 RX FIFO 中断阈值
 */
static void serial_config_rx_fifo_threshold(SerialHandle_t *handle, bool wait_idle)
{
    if ((handle == NULL) || (handle->hardware == NULL)) {
        return;
    }

    if (wait_idle) {
        DL_UART_setRXFIFOThreshold(handle->hardware, DL_UART_RX_FIFO_LEVEL_1_2_FULL);
    } else {
        DL_UART_setRXFIFOThreshold(handle->hardware, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    }
}

/**
 * @brief 结束一次发送事务
 */
static void serial_finish_tx(SerialHandle_t *handle)
{
    SendDoneCb tx_done_cb;

    if (handle == NULL) {
        return;
    }

    handle->tx_done = true;
    handle->tx_length = 0U;
    handle->tx_index = 0U;
    handle->gState = SERIAL_STATE_READY;
    tx_done_cb = handle->tx_done_cb;
    handle->tx_done_cb = NULL;
    serial_disable_tx_it(handle->hardware);
    DL_UART_disableInterrupt(handle->hardware, DL_UART_INTERRUPT_EOT_DONE);

    if (handle->tx_sem) {
        serial_sem_give(handle->tx_sem);
    }

    if (tx_done_cb) {
        tx_done_cb(handle->param);
    }
}

/**
 * @brief 调用并清除本次接收完成回调
 */
static void serial_invoke_rx_done_cb(SerialHandle_t *handle,
    uint8_t *data, uint16_t size)
{
    RecvCb rx_done_cb;

    if (handle == NULL) {
        return;
    }

    rx_done_cb = handle->recv_cb;
    handle->recv_cb = NULL;

    if (rx_done_cb) {
        rx_done_cb(data, size, handle->param);
    }
}

/**
 * @brief 完成一次接收事务
 */
static void serial_complete_rx(
    SerialHandle_t *handle, uint16_t count, bool signal_waiter)
{
    if (handle == NULL) {
        return;
    }

    if (count > handle->recv_buffer_size) {
        count = handle->recv_buffer_size;
    }

    handle->recv_count   = count;
    handle->recv_expected = 0U;
    handle->rx_done      = true;
    handle->rx_using_dma = false;
    handle->rx_wait_idle = false;
    handle->rxState      = SERIAL_STATE_READY;
    serial_disable_rx_it(handle->hardware);
    DL_UART_clearInterruptStatus(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    /*
     * SerialReceiveIDLE 依赖 UART 硬件空闲事件判帧。
     * 在 MSPM0 driverlib 中，这个事件通过 RX_TIMEOUT_ERROR 上报，
     * 这里在事务结束后关闭它，避免空闲线继续打断后续流程。
     */
    DL_UART_disableInterrupt(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);

    if (handle->mode == SERIAL_MODE_DMA) {
        /*
         * DMA 模式下在空闲期重新打开 RX 中断，用于及时丢弃
         * “应用层没有接收但线上又来了”的无主数据。
         */
        serial_enable_rx_it(handle->hardware);
    }

    if ((handle->mode == SERIAL_MODE_DMA) &&
        (handle->dma_rx_ch != SERIAL_DMA_CHANNEL_INVALID) &&
        DL_DMA_isChannelEnabled(DMA, handle->dma_rx_ch)) {
        DL_DMA_disableChannel(DMA, handle->dma_rx_ch);
    }

    serial_dbg_commit_finish(count);

    if (signal_waiter && (handle->rx_sem != NULL)) {
        serial_sem_give(handle->rx_sem);
    }
}

/**
 * @brief 在中断上下文中结束一次接收事务
 */
static void serial_finish_rx(SerialHandle_t *handle, uint16_t count)
{
    serial_complete_rx(handle, count, true);
}

/**
 * @brief 获取当前这次接收已经收到的字节数
 */
static uint16_t serial_get_received_count(SerialHandle_t *handle)
{
    uint16_t received = 0U;

    if (handle == NULL) {
        return 0U;
    }

    if (handle->rx_using_dma &&
        (handle->dma_rx_ch != SERIAL_DMA_CHANNEL_INVALID)) {
        uint16_t remaining = DL_DMA_getTransferSize(DMA, handle->dma_rx_ch);
        uint16_t dma_expected = handle->recv_expected;

        if (handle->recv_count < dma_expected) {
            dma_expected -= handle->recv_count;
        } else {
            dma_expected = 0U;
        }

        received = handle->recv_count;
        if (remaining < dma_expected) {
            received += (uint16_t) (dma_expected - remaining);
        }
    } else {
        received = handle->recv_count;
    }

    if (received > handle->recv_buffer_size) {
        received = handle->recv_buffer_size;
    }

    return received;
}

/**
 * @brief 在任务上下文里结束当前接收并返回已经收到的长度
 */
static uint16_t serial_cancel_rx(SerialHandle_t *handle)
{
    uint16_t received = serial_get_received_count(handle);

    if (handle == NULL) {
        return 0U;
    }

    handle->recv_count   = received;
    serial_dbg_set_finish_reason(UART_DBG_RX_FINISH_CANCEL);
    serial_complete_rx(handle, received, false);
    handle->recv_cb = NULL;

    return received;
}

/**
 * @brief 开始一次中断接收
 */
static int serial_start_it_receive(
    SerialHandle_t *handle, uint16_t size, bool wait_idle)
{
    bool overflow = false;

    if ((handle == NULL) || (size == 0U)) {
        return SERIAL_ERR_INVALID;
    }

    if (serial_rx_transaction_active(handle)) {
        return SERIAL_ERR_BUSY;
    }

    if (size > handle->recv_buffer_size) {
        size = handle->recv_buffer_size;
    }

    handle->recv_count   = 0;
    handle->recv_expected = size;
    handle->rx_done      = false;
    handle->rx_wait_idle = wait_idle;
    handle->rx_using_dma = false;
    handle->rxState      = SERIAL_STATE_BUSY;

    serial_sem_reset(handle->rx_sem);
    DL_UART_clearInterruptStatus(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    serial_config_rx_fifo_threshold(handle, wait_idle);

    serial_enable_rx_it(handle->hardware);

    if (wait_idle) {
        /*
         * 不定长接收使用 UART 硬件空闲事件收帧。
         * MSPM0 上该事件由 RX_TIMEOUT_ERROR 中断给出，不使用任务层超时。
         */
        DL_UART_enableInterrupt(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    } else {
        DL_UART_disableInterrupt(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    }

    (void) serial_drain_rx_fifo(handle, &overflow);
    if (overflow && (handle->error_cb != NULL)) {
        handle->error_code |= SERIAL_ERROR_OVERRUN;
        handle->error_cb(SERIAL_ERR_BUSY, handle->param);
    }

    if (handle->recv_count >= handle->recv_expected) {
        serial_dbg_set_finish_reason(UART_DBG_RX_FINISH_RX_FULL);
        serial_complete_rx(handle, handle->recv_count, false);
    }

    return (int) size;
}

/**
 * @brief 开始一次 DMA 接收
 */
static int serial_start_dma_receive(
    SerialHandle_t *handle, uint16_t size, bool wait_idle)
{
    bool overflow = false;
    uint16_t preloaded;
    uint16_t dma_size;

    if ((handle == NULL) || (size == 0U) ||
        (handle->dma_rx_ch == SERIAL_DMA_CHANNEL_INVALID)) {
        return SERIAL_ERR_INVALID;
    }

    if (serial_rx_transaction_active(handle)) {
        return SERIAL_ERR_BUSY;
    }

    if (size > handle->recv_buffer_size) {
        size = handle->recv_buffer_size;
    }

    if (DL_DMA_isChannelEnabled(DMA, handle->dma_rx_ch)) {
        DL_DMA_disableChannel(DMA, handle->dma_rx_ch);
    }

    handle->recv_count    = 0;
    handle->recv_expected = size;
    handle->rx_done       = false;
    handle->rx_wait_idle  = wait_idle;
    handle->rx_using_dma  = true;
    handle->rxState       = SERIAL_STATE_BUSY;

    serial_sem_reset(handle->rx_sem);
    DL_UART_clearInterruptStatus(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    serial_disable_rx_it(handle->hardware);
    serial_config_rx_fifo_threshold(handle, wait_idle);

    if (wait_idle) {
        /*
         * DMA 模式下字节搬运由 DMA 完成，帧结束仍由 UART 硬件空闲事件判定。
         */
        DL_UART_enableInterrupt(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    } else {
        DL_UART_disableInterrupt(handle->hardware, DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    }

    preloaded = serial_drain_rx_fifo(handle, &overflow);
    if (overflow && (handle->error_cb != NULL)) {
        handle->error_code |= SERIAL_ERROR_OVERRUN;
        handle->error_cb(SERIAL_ERR_BUSY, handle->param);
    }

    if (handle->recv_count >= handle->recv_expected) {
        serial_dbg_set_finish_reason(UART_DBG_RX_FINISH_RX_FULL);
        serial_complete_rx(handle, handle->recv_count, false);
        return (int) size;
    }

    dma_size = (uint16_t) (size - preloaded);
    DL_DMA_setSrcAddr(DMA, handle->dma_rx_ch, (uint32_t) (&handle->hardware->RXDATA));
    DL_DMA_setDestAddr(DMA, handle->dma_rx_ch,
        (uint32_t) (handle->recv_buffer + preloaded));
    DL_DMA_setTransferSize(DMA, handle->dma_rx_ch, dma_size);
    DL_DMA_enableChannel(DMA, handle->dma_rx_ch);

    return (int) size;
}

/* ==================== 核心 API 实现 ==================== */

/**
 * @brief 初始化串口
 */
SerialHandle_t* SerialInit(UART_Regs *hw_uart, uint8_t mode, ErrorCb error_cb, void* param)
{
    if (!hw_uart) {
        return NULL;
    }

    /* 分配句柄内存 */
    SerialHandle_t *handle = (SerialHandle_t*)serial_malloc(sizeof(SerialHandle_t));
    if (!handle) {
        return NULL;
    }

    memset(handle, 0, sizeof(SerialHandle_t));

    /* 初始化基本字段 */
    handle->hardware = hw_uart;
    handle->mode = mode;
    handle->error_cb = error_cb;
    handle->param = param;
    handle->dma_rx_ch = SERIAL_DMA_CHANNEL_INVALID;
    handle->dma_tx_ch = SERIAL_DMA_CHANNEL_INVALID;
    handle->rx_done = true;
    handle->tx_done = true;
    handle->gState = SERIAL_STATE_READY;
    handle->rxState = SERIAL_STATE_READY;
    handle->error_code = SERIAL_ERROR_NONE;

    /* 分配发送缓冲区 (默认 256 字节) */
    handle->send_buffer_size = 256;
    handle->send_buffer = (uint8_t*)serial_malloc(handle->send_buffer_size);
    if (!handle->send_buffer) {
        serial_free(handle);
        return NULL;
    }

    /* 分配接收缓冲区 (默认 512 字节) */
    handle->recv_buffer_size = 512;
    handle->recv_buffer = (uint8_t*)serial_malloc(handle->recv_buffer_size);
    if (!handle->recv_buffer) {
        serial_free(handle->send_buffer);
        serial_free(handle);
        return NULL;
    }

    /* 创建同步信号量 */
    #if defined(configTOTAL_HEAP_SIZE)
        handle->rx_sem = xSemaphoreCreateBinary();
        handle->tx_sem = xSemaphoreCreateBinary();
        if (!handle->rx_sem || !handle->tx_sem) {
            serial_free(handle->send_buffer);
            serial_free(handle->recv_buffer);
            serial_free(handle);
            return NULL;
        }
    #endif

    /* 根据模式配置 */
    switch (mode) {
        case SERIAL_MODE_IT:
            /* 中断模式：仅在真正开始接收事务时打开 RX 中断 */
            serial_disable_rx_it(hw_uart);
            break;

        case SERIAL_MODE_DMA:
            DL_UART_enableInterrupt(hw_uart,
                DL_UART_INTERRUPT_RX |
                DL_UART_INTERRUPT_DMA_DONE_RX |
                DL_UART_INTERRUPT_DMA_DONE_TX |
                DL_UART_INTERRUPT_EOT_DONE |
                DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
            break;

        case SERIAL_MODE_POLL:
            /* 轮询模式：无需配置 */
            break;

        default:
            serial_free(handle->send_buffer);
            serial_free(handle->recv_buffer);
            serial_free(handle);
            return NULL;
    }


    return handle;
}

/**
 * @brief 反初始化串口
 */
int SerialDeinit(SerialHandle_t* handle)
{
    if (!handle) {
        return SERIAL_ERR_INVALID;
    }

    /* 禁用中断 */
    if (handle->hardware) {
        serial_disable_rx_it(handle->hardware);
        serial_disable_tx_it(handle->hardware);
        DL_UART_disableInterrupt(handle->hardware,
            DL_UART_INTERRUPT_EOT_DONE |
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
            DL_UART_INTERRUPT_DMA_DONE_RX |
            DL_UART_INTERRUPT_DMA_DONE_TX);
    }

    /* 释放缓冲区 */
    if (handle->send_buffer) {
        serial_free(handle->send_buffer);
    }
    if (handle->recv_buffer) {
        serial_free(handle->recv_buffer);
    }

    /* 释放信号量 */
    #if defined(configTOTAL_HEAP_SIZE)
        if (handle->rx_sem) {
            vSemaphoreDelete(handle->rx_sem);
        }
        if (handle->tx_sem) {
            vSemaphoreDelete(handle->tx_sem);
        }
    #endif

    /* 释放句柄 */
    serial_free(handle);

    return SERIAL_OK;
}

/**
 * @brief 串口发送数据
 */
int SerialTransmit(SerialHandle_t* handle, uint8_t *data, uint16_t size,
                   SendDoneCb tx_done_cb)
{
    uint16_t i;

    if (!handle || !data || size == 0) {
        return SERIAL_ERR_INVALID;
    }

    if (!handle->hardware) {
        return SERIAL_ERR_INVALID;
    }

    switch (handle->mode) {
        /* 轮询模式：直接发送 */
        case SERIAL_MODE_POLL:
            for (i = 0; i < size; i++) {
                while (DL_UART_isTXFIFOFull(handle->hardware)) {
                    /* 等待 FIFO 有空间 */
                }
                DL_UART_transmitData(handle->hardware, data[i]);
            }
            return size;

        /* 中断模式：复制到缓冲区并启用发送中断 */
        case SERIAL_MODE_IT:
            if (size > handle->send_buffer_size) {
                size = handle->send_buffer_size;
            }

            if (serial_tx_transaction_active(handle)) {
                return SERIAL_ERR_BUSY;
            }

            serial_sem_reset(handle->tx_sem);
            memcpy(handle->send_buffer, data, size);
            handle->tx_length = size;
            handle->tx_index = 0;
            handle->tx_done = false;
            handle->gState = SERIAL_STATE_BUSY;
            handle->tx_done_cb = tx_done_cb;
            DL_UART_clearInterruptStatus(handle->hardware, DL_UART_INTERRUPT_EOT_DONE);
            DL_UART_enableInterrupt(handle->hardware, DL_UART_INTERRUPT_EOT_DONE);

            /* 先把 FIFO 填到满，剩余部分交给 TX 中断继续搬运 */
            while ((handle->tx_index < handle->tx_length) &&
                   (DL_UART_isTXFIFOFull(handle->hardware) == false)) {
                DL_UART_transmitData(handle->hardware,
                    handle->send_buffer[handle->tx_index++]);
            }

            if (handle->tx_index < handle->tx_length) {
                serial_enable_tx_it(handle->hardware);
            } else {
                serial_disable_tx_it(handle->hardware);
            }
            return size;

        /* DMA 模式：配置 DMA 传输 */
        case SERIAL_MODE_DMA:
        {
            if (size > handle->send_buffer_size) {
                size = handle->send_buffer_size;
            }

            if (handle->dma_tx_ch == SERIAL_DMA_CHANNEL_INVALID) {
                return SERIAL_ERR_INVALID;
            }

            if (serial_tx_transaction_active(handle) ||
                DL_DMA_isChannelEnabled(DMA, handle->dma_tx_ch)) {
                return SERIAL_ERR_BUSY;
            }

            serial_sem_reset(handle->tx_sem);
            memcpy(handle->send_buffer, data, size);
            handle->tx_length = size;
            handle->tx_index = size;
            handle->tx_done = false;
            handle->gState = SERIAL_STATE_BUSY;
            handle->tx_done_cb = tx_done_cb;
            DL_UART_clearInterruptStatus(handle->hardware, DL_UART_INTERRUPT_EOT_DONE);
            DL_UART_enableInterrupt(handle->hardware, DL_UART_INTERRUPT_EOT_DONE);

            DL_DMA_setSrcAddr(DMA, handle->dma_tx_ch, (uint32_t)handle->send_buffer);
            DL_DMA_setDestAddr(DMA, handle->dma_tx_ch, (uint32_t)(&handle->hardware->TXDATA));
            DL_DMA_setTransferSize(DMA, handle->dma_tx_ch, size);
            DL_DMA_enableChannel(DMA, handle->dma_tx_ch);

            return size;
        }

        default:
            return SERIAL_ERR_INVALID;
    }
}

/**
 * @brief 流式接收 (定长数据)
 */
int SerialReceive(SerialHandle_t* handle, uint8_t *data, uint16_t size,
                  int timeout, RecvCb rx_done_cb)
{
    int rc;
    uint16_t i = 0;
    TickType_t start_tick = 0;

    if (!handle || !data || size == 0) {
        return SERIAL_ERR_INVALID;
    }

    if (!handle->hardware) {
        return SERIAL_ERR_INVALID;
    }

    switch (handle->mode) {
        /* 轮询模式 */
        case SERIAL_MODE_POLL:
            if (timeout >= 0) {
                start_tick = xTaskGetTickCount();
            }

            while (i < size) {
                if (DL_UART_isRXFIFOEmpty(handle->hardware) == false) {
                    data[i++] = DL_UART_receiveData(handle->hardware);
                } else {
                    /* 超时检查 */
                    if ((timeout >= 0) &&
                        ((xTaskGetTickCount() - start_tick) >=
                         pdMS_TO_TICKS((uint32_t) timeout))) {
                        if (handle->error_cb) {
                            handle->error_cb(SERIAL_ERR_TIMEOUT, handle->param);
                        }
                        handle->recv_cb = NULL;
                        return (i > 0) ? i : SERIAL_ERR_TIMEOUT;
                    }
                }
            }
            handle->recv_cb = rx_done_cb;
            serial_invoke_rx_done_cb(handle, data, i);
            return i;

        /* 中断或 DMA 模式：使用信号量同步 */
        case SERIAL_MODE_IT:
            handle->recv_cb = rx_done_cb;
            rc = serial_start_it_receive(handle, size, false);
            if (rc < 0) {
                handle->recv_cb = NULL;
                return rc;
            }
            break;

        case SERIAL_MODE_DMA:
            handle->recv_cb = rx_done_cb;
            rc = serial_start_dma_receive(handle, size, false);
            if (rc < 0) {
                handle->recv_cb = NULL;
                return rc;
            }
            break;

        default:
            return SERIAL_ERR_INVALID;
    }

    if (handle->rx_done) {
        if (handle->recv_count > size) {
            handle->recv_count = size;
        }
        memcpy(data, handle->recv_buffer, handle->recv_count);
        serial_invoke_rx_done_cb(handle, data, handle->recv_count);
        return handle->recv_count;
    }

    /* 等待接收完成 (使用信号量) */
    if (serial_sem_take(handle->rx_sem, timeout) != 0) {
        uint16_t received = serial_cancel_rx(handle);

        if (received > size) {
            received = size;
        }

        if (received > 0U) {
            memcpy(data, handle->recv_buffer, received);
            return received;
        }

        if (handle->error_cb) {
            handle->error_cb(SERIAL_ERR_TIMEOUT, handle->param);
        }
        return SERIAL_ERR_TIMEOUT;
    }

    /* 复制数据 */
    if (handle->recv_count > size) {
        handle->recv_count = size;
    }
    memcpy(data, handle->recv_buffer, handle->recv_count);
    serial_invoke_rx_done_cb(handle, data, handle->recv_count);
    return handle->recv_count;
}

/**
 * @brief 不定长数据接收 (由硬件空闲事件判定一帧结束)
 */
int SerialReceiveIDLE(SerialHandle_t* handle, uint8_t *data, uint16_t max_size,
                      RecvCb rx_done_cb)
{
    int rc;

    if (!handle || !data || max_size == 0) {
        return SERIAL_ERR_INVALID;
    }

    if (!handle->hardware) {
        return SERIAL_ERR_INVALID;
    }

    switch (handle->mode) {
        /* 轮询模式：返回实时可用数据 */
        case SERIAL_MODE_POLL:
        {
            uint16_t count = 0;
            while (!DL_UART_isRXFIFOEmpty(handle->hardware) && count < max_size) {
                data[count++] = DL_UART_receiveData(handle->hardware);
            }
            handle->recv_cb = rx_done_cb;
            serial_invoke_rx_done_cb(handle, data, count);
            return count;
        }

        /* 中断或 DMA 模式：启动接收，直到硬件空闲事件到来 */
        case SERIAL_MODE_IT:
            handle->recv_cb = rx_done_cb;
            rc = serial_start_it_receive(handle, max_size, true);
            if (rc < 0) {
                handle->recv_cb = NULL;
                return rc;
            }
            break;

        case SERIAL_MODE_DMA:
            handle->recv_cb = rx_done_cb;
            rc = serial_start_dma_receive(handle, max_size, true);
            if (rc < 0) {
                handle->recv_cb = NULL;
                return rc;
            }
            break;

        default:
            return SERIAL_ERR_INVALID;
    }

    if (handle->rx_done) {
        if (handle->recv_count > max_size) {
            handle->recv_count = max_size;
        }
        memcpy(data, handle->recv_buffer, handle->recv_count);
        serial_invoke_rx_done_cb(handle, data, handle->recv_count);
        return handle->recv_count;
    }

    /*
     * SerialReceiveIDLE 专门用于硬件空闲事件判帧：
     * 一旦 UART 报告 RX 空闲，就立即认为一帧结束并返回。
     * 这里不使用任务层软件超时作为分包条件，避免高频通信时
     * 因任务调度抖动导致判帧不稳定。
     */
    if (serial_sem_take(handle->rx_sem, -1) != 0) {
        (void) serial_cancel_rx(handle);
        return SERIAL_ERR_TIMEOUT;
    }

    /* 复制接收到的数据 */
    if (handle->recv_count > max_size) {
        handle->recv_count = max_size;
    }
    memcpy(data, handle->recv_buffer, handle->recv_count);
    serial_invoke_rx_done_cb(handle, data, handle->recv_count);

    return handle->recv_count;
}
int debug_irq_cnt=0;
/**
 * @brief UART 中断处理函数
 * 应在 UART ISR 中调用此函数
 *
 * 使用示例:
 * void UART0_IRQHandler(void)
 * {
 *     SerialIRQ(g_serial_handle);
 * }
 */
void SerialIRQ(SerialHandle_t* handle)
{
    uint16_t event_budget = SERIAL_IRQ_EVENT_BUDGET;
    DL_UART_IIDX status;

    if (!handle || !handle->hardware) {
        return;
    }

    while ((event_budget-- > 0U) &&
           ((status = DL_UART_Main_getPendingInterrupt(handle->hardware)) !=
            DL_UART_MAIN_IIDX_NO_INTERRUPT)) {
        switch (status) {
            case DL_UART_MAIN_IIDX_RX:
            {
                bool overflow = false;

                if (!serial_rx_transaction_active(handle)) {
                    serial_flush_rx_fifo_bounded(
                        handle->hardware, SERIAL_IRQ_FIFO_BUDGET);
                    break;
                }

                (void) serial_drain_rx_fifo_bounded(
                    handle, &overflow, SERIAL_IRQ_FIFO_BUDGET);
                if (overflow) {
                    handle->error_code |= SERIAL_ERROR_OVERRUN;
                    if (handle->error_cb != NULL) {
                        handle->error_cb(SERIAL_ERR_BUSY, handle->param);
                    }
                }

                /*
                 * 定长模式下收到目标长度立即完成；
                 * IDLE 模式下如果缓冲区已满，也应立即完成，避免溢出。
                 */
                if ((handle->recv_count >= handle->recv_expected) &&
                    (handle->recv_expected > 0U)) {
                    serial_dbg_set_finish_reason(UART_DBG_RX_FINISH_RX_FULL);
                    serial_finish_rx(handle, handle->recv_count);
                    return;
                }
                break;
            }

            case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
                if (!serial_rx_transaction_active(handle)) {
                    DL_UART_clearInterruptStatus(handle->hardware,
                        DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
                    serial_flush_rx_fifo_bounded(
                        handle->hardware, SERIAL_IRQ_FIFO_BUDGET);
                    break;
                }

                /*
                 * MSPM0 driverlib 将 RX 空闲事件作为 RX_TIMEOUT_ERROR 上报。
                 * 对 SerialReceiveIDLE 来说，这里表示“本帧结束”，
                 * 应立刻按当前已收到的长度完成接收，而不是继续等软件超时。
                 */
                if (handle->rx_using_dma && handle->rx_wait_idle &&
                    (handle->dma_rx_ch != SERIAL_DMA_CHANNEL_INVALID)) {
                    uint16_t remaining = DL_DMA_getTransferSize(DMA, handle->dma_rx_ch);
                    uint16_t dma_expected = handle->recv_expected;
                    uint16_t received = handle->recv_count;

                    if (handle->recv_count < dma_expected) {
                        dma_expected -= handle->recv_count;
                    } else {
                        dma_expected = 0U;
                    }

                    if (remaining < dma_expected) {
                        received += (uint16_t) (dma_expected - remaining);
                    }

                    serial_dbg_set_finish_reason(UART_DBG_RX_FINISH_RX_TIMEOUT);
                    serial_finish_rx(handle, received);
                    return;
                } else if (handle->rx_wait_idle) {
                    bool overflow = false;

                    /*
                     * IT + IDLE 模式下，尾部未达到 RX FIFO 阈值的字节会
                     * 留在 FIFO 中，等 RX_TIMEOUT_ERROR 到来后在这里一起取走。
                     */
                    (void) serial_drain_rx_fifo_bounded(
                        handle, &overflow, SERIAL_IRQ_FIFO_BUDGET);
                    if (overflow) {
                        handle->error_code |= SERIAL_ERROR_OVERRUN;
                        if (handle->error_cb != NULL) {
                            handle->error_cb(SERIAL_ERR_BUSY, handle->param);
                        }
                    }

                    if (handle->recv_count == 0U) {
                        break;
                    }

                    serial_dbg_set_finish_reason(UART_DBG_RX_FINISH_RX_TIMEOUT);
                    serial_finish_rx(handle, handle->recv_count);
                    return;
                }
                break;

            case DL_UART_MAIN_IIDX_TX:
                (void) serial_fill_tx_fifo_bounded(
                    handle, SERIAL_IRQ_FIFO_BUDGET);

                if (handle->tx_index >= handle->tx_length) {
                    serial_disable_tx_it(handle->hardware);
                }
                break;

            case DL_UART_MAIN_IIDX_DMA_DONE_RX:
                if (handle->rx_using_dma) {
                    serial_dbg_set_finish_reason(UART_DBG_RX_FINISH_DMA_DONE_RX);
                    serial_finish_rx(handle, handle->recv_expected);
                    return;
                }
                break;

            case DL_UART_MAIN_IIDX_DMA_DONE_TX:
                /*
                 * DMA_DONE_TX 表示数据已搬入 UART FIFO，真正的线缆发送完成
                 * 还需要等待 EOT_DONE。
                 */
                break;

            case DL_UART_MAIN_IIDX_EOT_DONE:
                if (((handle->mode == SERIAL_MODE_DMA) ||
                     (handle->mode == SERIAL_MODE_IT)) &&
                    serial_tx_transaction_active(handle)) {
                    serial_finish_tx(handle);
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
                handle->error_code |= SERIAL_ERROR_OVERRUN;
                DL_UART_clearInterruptStatus(
                    handle->hardware, DL_UART_INTERRUPT_OVERRUN_ERROR);
                if (serial_rx_transaction_active(handle)) {
                    serial_finish_rx(handle, serial_get_received_count(handle));
                    return;
                }
                if (handle->error_cb) {
                    handle->error_cb((int) status, handle->param);
                }
                break;
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
                handle->error_code |= SERIAL_ERROR_BREAK;
                DL_UART_clearInterruptStatus(
                    handle->hardware, DL_UART_INTERRUPT_BREAK_ERROR);
                if (handle->error_cb) {
                    handle->error_cb((int) status, handle->param);
                }
                break;
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
                handle->error_code |= SERIAL_ERROR_PARITY;
                DL_UART_clearInterruptStatus(
                    handle->hardware, DL_UART_INTERRUPT_PARITY_ERROR);
                if (serial_rx_transaction_active(handle)) {
                    serial_finish_rx(handle, serial_get_received_count(handle));
                    return;
                }
                if (handle->error_cb) {
                    handle->error_cb((int) status, handle->param);
                }
                break;
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
                handle->error_code |= SERIAL_ERROR_FRAMING;
                DL_UART_clearInterruptStatus(
                    handle->hardware, DL_UART_INTERRUPT_FRAMING_ERROR);
                if (serial_rx_transaction_active(handle)) {
                    serial_finish_rx(handle, serial_get_received_count(handle));
                    return;
                }
                if (handle->error_cb) {
                    handle->error_cb((int) status, handle->param);
                }
                break;
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                handle->error_code |= SERIAL_ERROR_NOISE;
                DL_UART_clearInterruptStatus(
                    handle->hardware, DL_UART_INTERRUPT_NOISE_ERROR);
                if (handle->error_cb) {
                    handle->error_cb((int) status, handle->param);
                }
                break;

            default:
                break;
        }
    }
}

/**
 * @brief 配置 DMA 发送通道 (必须在 SerialInit 后调用)
 * @param handle 串口句柄
 * @param dma_tx_ch DMA TX 通道号 (例如 DMA_CH0_CHAN_ID)
 * @param dma_rx_ch DMA RX 通道号 (例如 DMA_CH1_CHAN_ID)
 * @return SERIAL_OK 或错误码
 */
int SerialConfigDMA(SerialHandle_t* handle, uint8_t dma_tx_ch, uint8_t dma_rx_ch)
{
    if (!handle || handle->mode != SERIAL_MODE_DMA) {
        return SERIAL_ERR_INVALID;
    }

    /* 存储 DMA 通道号 */
    handle->dma_tx_ch = dma_tx_ch;
    handle->dma_rx_ch = dma_rx_ch;

    /* 启用 UART DMA 事件和相关中断 */
    if (handle->hardware) {
        DL_UART_enableDMATransmitEvent(handle->hardware);
        /*
         * RX DMA 使用正常接收事件持续搬运字节，空闲判帧仍由
         * RX_TIMEOUT_ERROR 中断负责，这样不会只在超时时搬运尾部数据。
         */
        DL_UART_enableDMAReceiveEvent(handle->hardware, DL_UART_DMA_INTERRUPT_RX);
        DL_UART_setRXFIFOThreshold(handle->hardware, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
        DL_UART_enableInterrupt(handle->hardware,
            DL_UART_INTERRUPT_RX |
            DL_UART_INTERRUPT_DMA_DONE_RX |
            DL_UART_INTERRUPT_DMA_DONE_TX |
            DL_UART_INTERRUPT_EOT_DONE |
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    }

    return SERIAL_OK;
}
