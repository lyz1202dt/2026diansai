#include "CLI/App/port.h"
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include "Driver/uart/uart.h"

#define CLI_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE * 4)
#define CLI_TASK_PRIORITY    (tskIDLE_PRIORITY + 2)

static CLI_t* global_cli_handle = NULL;
static SerialHandle_t *g_cli_serial = NULL;
static TaskHandle_t g_cli_task_handle = NULL;

static void CLI_UARTPutBuffer(const uint8_t *data, uint16_t len)
{
    if ((g_cli_serial == NULL) || (data == NULL) || (len == 0U)) {
        return;
    }

    if (SerialTransmit(g_cli_serial, (uint8_t *)data, len, NULL) < 0) {
        return;
    }

    while (g_cli_serial->gState == SERIAL_STATE_BUSY) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void CLI_UARTPutChar(char ch)
{
    uint8_t byte = (uint8_t)ch;
    CLI_UARTPutBuffer(&byte, 1U);
}

static char CLI_UARTGetChar(void)
{
    uint8_t byte = 0;
    int rc;

    if (g_cli_serial == NULL) {
        return '\0';
    }

    rc = SerialReceive(g_cli_serial, &byte, 1U, -1, NULL);
    if (rc <= 0) {
        return '\0';
    }

    return (char)byte;
}

static void CLITask(void *param)
{
    CLI_t *cli = (CLI_t *)param;

    if (cli == NULL) {
        vTaskDelete(NULL);
        return;
    }

    printf_cli(cli, "\r\nCLI ready on UART0 @ %d baud\r\n", UART_0_BAUD_RATE);

    while (1) {
        printf_cli(cli, "cli> ");
        CLIRun(cli);
    }
}

int CLI_EnvInit(void)
{
    if (global_cli_handle != NULL) {
        return 0;
    }

    g_cli_serial = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);
    if (g_cli_serial == NULL) {
        return -1;
    }

    global_cli_handle = CLICreate();
    if (global_cli_handle == NULL) {
        SerialDeinit(g_cli_serial);
        g_cli_serial = NULL;
        return -1;
    }

    global_cli_handle->putchar = CLI_UARTPutChar;
    global_cli_handle->getchar = CLI_UARTGetChar;

    if (CLIAddCommand(global_cli_handle, (Command_t){.name = "help", .cmd = __help}) != 0) {
        CLIDelete();
        SerialDeinit(g_cli_serial);
        g_cli_serial = NULL;
        return -1;
    }

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    if (xTaskCreate(CLITask,
                    "cli_uart0",
                    256,
                    global_cli_handle,
                    CLI_TASK_PRIORITY,
                    &g_cli_task_handle) != pdPASS) {
        CLIDelete();
        SerialDeinit(g_cli_serial);
        g_cli_serial = NULL;
        return -1;
    }

    return 0;
}

