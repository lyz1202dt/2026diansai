#include "projdefs.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <task.h>

/* 使用 driverlib 的 DMA 接口 */
#include "Driver/uart/uart.h"
#include "Bsp/motor.h"
#include "Bsp/mpu6050.h"
#include "Bsp/gw_model.h"
#include "Bsp/OLED.h"
#include <ti/driverlib/dl_dma.h>
#include "Lib/pid/PID.h"

#include "config.h"

void UART_0_INST_IRQHandler(void) {
    SerialIRQ(g_serial);
}


void UART_1_INST_IRQHandler(void) {
    SerialIRQ(zdt_serial);
}


void GROUP1_IRQHandler(void) {
  switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
  case ENCODER_PIN_GPIOB_INT_IIDX: {
    uint32_t encoder_pins = ENCODER_PIN_E1A_PIN | ENCODER_PIN_E1B_PIN;
    uint32_t interrupt_status =
        DL_GPIO_getEnabledInterruptStatus(GPIOB, encoder_pins);

    if (interrupt_status != 0U) {
      EncoderUpdate(&g_encoder1, GPIOB, interrupt_status);
      DL_GPIO_clearInterruptStatus(GPIOB, interrupt_status);
    }
    break;
  }

  case ENCODER_PIN_GPIOA_INT_IIDX: {
    uint32_t encoder_pins = ENCODER_PIN_E2A_PIN | ENCODER_PIN_E2B_PIN;
    uint32_t interrupt_status =
        DL_GPIO_getEnabledInterruptStatus(GPIOA, encoder_pins);

    if (interrupt_status != 0U) {
      EncoderUpdate(&g_encoder2, GPIOA, interrupt_status);
      DL_GPIO_clearInterruptStatus(GPIOA, interrupt_status);
    }
    break;
  }

  default:
    break;
  }
}
