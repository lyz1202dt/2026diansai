// #include "CLI/App/port.h"


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
#include <ti/driverlib/dl_dma.h>

uint8_t send_str[8] = {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t revb_str[8] = {};
static SerialHandle_t *g_serial;

L298N_t g_l298n = {
    .pwm_timer = WHEEL_PWM_INST,
    .motor1 = {
        .in1 = {
            .port = MOTOR_PIN_PORT,
            .pin = MOTOR_PIN_M1A_PIN,
        },
        .in2 = {
            .port = MOTOR_PIN_PORT,
            .pin = MOTOR_PIN_M1B_PIN,
        },
        .pwm_cc_index = GPIO_WHEEL_PWM_C0_IDX,
        .invert_direction = false,
        .duty_permille = 0,
    },
    .motor2 = {
        .in1 = {
            .port = MOTOR_PIN_PORT,
            .pin = MOTOR_PIN_M2A_PIN,
        },
        .in2 = {
            .port = MOTOR_PIN_PORT,
            .pin = MOTOR_PIN_M2B_PIN,
        },
        .pwm_cc_index = GPIO_WHEEL_PWM_C1_IDX,
        .invert_direction = false,
        .duty_permille = 0,
    },
};

Encoder_t g_encoder1 = {
    .cha = {
        .port = ENCODER_PIN_E1A_PORT,
        .pin = ENCODER_PIN_E1A_PIN,
    },
    .chb = {
        .port = ENCODER_PIN_E1B_PORT,
        .pin = ENCODER_PIN_E1B_PIN,
    },
    .encoder_value = 0,
};

static void L298N_TestUpdate(void) {
  static uint8_t phase = 0;
  static uint16_t tick = 0;

  tick++;
  if (tick < 10U) {
    return;
  }
  tick = 0;

  switch (phase) {
  case 0:
    L298N_SetPWMValue(&g_l298n, 1, 400);
    L298N_SetPWMValue(&g_l298n, 2, 400);
    break;
  case 1:
    L298N_SetPWMValue(&g_l298n, 1, -400);
    L298N_SetPWMValue(&g_l298n, 2, -400);
    break;
  default:
    L298N_SetPWMValue(&g_l298n, 1, 0);
    L298N_SetPWMValue(&g_l298n, 2, 0);
    break;
  }

  phase = (uint8_t) ((phase + 1U) % 3U);
}

void send_done(void *param) {}

void recv_done(void *param) {}

uint16_t adc_value_group[8];
bool adc_success=false;
float mpu6050_quat[4];
float mpu6050_roll;
float mpu6050_pitch;
float mpu6050_yaw;
bool mpu6050_success=false;

int app_main() {
  L298N_Init(&g_l298n);
  EncoderInit(&g_encoder1);
  MPU6050_Init();

  GWModelInit();
  g_serial = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);
  NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
  NVIC_EnableIRQ(ENCODER_PIN_GPIOB_INT_IRQN);
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
  SerialTransmit(g_serial, send_str, 6, send_done);
  SerialReceive(g_serial, revb_str, 8, 100, NULL);

  while (1) {
    L298N_TestUpdate();
    SerialTransmit(g_serial, send_str, 3, send_done);
    DL_GPIO_setPins(LED_PORT, LED_LED0_PIN_PIN);
    vTaskDelay(50);
    DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN_PIN);
    vTaskDelay(50);
    adc_success=GWGetState(adc_value_group);
    mpu6050_success=(read_quad(mpu6050_quat)==0);
    if(mpu6050_success) {
      get_euler_angles(mpu6050_quat, &mpu6050_roll, &mpu6050_pitch, &mpu6050_yaw);
    }
  }
  return 0;
}

void UART_0_INST_IRQHandler(void) {
    SerialIRQ(g_serial);
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
      DL_GPIO_clearInterruptStatus(GPIOA, interrupt_status);
    }
    break;
  }

  default:
    break;
  }
}
