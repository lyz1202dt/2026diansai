#include "gw_model.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"

SemaphoreHandle_t k_gw_tracker_semphr;

static void switch_sensor_channel(uint8_t channnel)
{
    if(channnel&0x01)
        DL_GPIO_setPins(GW_ADDR_PIN_PORT, GW_ADDR_PIN_ADDR0_PIN);
    else 
        DL_GPIO_clearPins(GW_ADDR_PIN_PORT, GW_ADDR_PIN_ADDR0_PIN);

    if(channnel&0x02)
        DL_GPIO_setPins(GW_ADDR_PIN_PORT, GW_ADDR_PIN_ADDR1_PIN);
    else 
        DL_GPIO_clearPins(GW_ADDR_PIN_PORT, GW_ADDR_PIN_ADDR1_PIN);

    if(channnel&0x04)
        DL_GPIO_setPins(GW_ADDR_PIN_PORT, GW_ADDR_PIN_ADDR2_PIN);
    else 
        DL_GPIO_clearPins(GW_ADDR_PIN_PORT, GW_ADDR_PIN_ADDR2_PIN);
}

void GWModelInit()
{
    k_gw_tracker_semphr=xSemaphoreCreateBinary();
    if(k_gw_tracker_semphr != NULL) {
        xSemaphoreTake(k_gw_tracker_semphr, 0);
    }
}

uint32_t GWGetState(uint16_t *value)
{
    if((k_gw_tracker_semphr == NULL) || (value == NULL)) {
        return 0;
    }

    xSemaphoreTake(k_gw_tracker_semphr, 0);

    for(int i=0;i<8;i++)
    {
        switch_sensor_channel(i);
        vTaskDelay(pdMS_TO_TICKS(1));   //等待电平稳定
        xSemaphoreTake(k_gw_tracker_semphr, 0);
        DL_ADC12_enableConversions(ADC12_0_INST);
        DL_ADC12_startConversion(ADC12_0_INST);
        if(xSemaphoreTake(k_gw_tracker_semphr, pdMS_TO_TICKS(5))==pdFALSE) {
            return 0;
        }
        value[i]=DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    }
    return 1;
}

void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED: {
            BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(k_gw_tracker_semphr, &pxHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

            break;
        }
        default:
            break;
    }
}
