#include "gw_model.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

uint8_t GWGetState() {
  uint8_t result = 0;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_0_PORT, LINE_TRACE_PIN_0_PIN))
    result |= 0x01;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_1_PORT, LINE_TRACE_PIN_1_PIN))
    result |= 0x02;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_2_PORT, LINE_TRACE_PIN_2_PIN))
    result |= 0x04;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_3_PORT, LINE_TRACE_PIN_3_PIN))
    result |= 0x08;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_4_PORT, LINE_TRACE_PIN_4_PIN))
    result |= 0x10;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_5_PORT, LINE_TRACE_PIN_5_PIN))
    result |= 0x20;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_6_PORT, LINE_TRACE_PIN_6_PIN))
    result |= 0x40;
  if (DL_GPIO_readPins(LINE_TRACE_PIN_7_PORT, LINE_TRACE_PIN_7_PIN))
    result |= 0x80;
  return result;
}
