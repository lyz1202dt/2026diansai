#include "config.h"

SerialHandle_t *g_serial;
SerialHandle_t *zdt_serial;

L298N_t g_l298n = {
    .pwm_timer = WHEEL_PWM_INST,
    .motor1 =
        {
            .in1 =
                {
                    .port = MOTOR_PIN_PORT,
                    .pin = MOTOR_PIN_M1A_PIN,
                },
            .in2 =
                {
                    .port = MOTOR_PIN_PORT,
                    .pin = MOTOR_PIN_M1B_PIN,
                },
            .pwm_cc_index = GPIO_WHEEL_PWM_C0_IDX,
            .invert_direction = false,
            .duty_permille = 0,
        },
    .motor2 =
        {
            .in1 =
                {
                    .port = MOTOR_PIN_PORT,
                    .pin = MOTOR_PIN_M2A_PIN,
                },
            .in2 =
                {
                    .port = MOTOR_PIN_PORT,
                    .pin = MOTOR_PIN_M2B_PIN,
                },
            .pwm_cc_index = GPIO_WHEEL_PWM_C1_IDX,
            .invert_direction = false,
            .duty_permille = 0,
        },
};

Encoder_t g_encoder1 = {
    .cha =
        {
            .port = ENCODER_PIN_E1A_PORT,
            .pin = ENCODER_PIN_E1A_PIN,
        },
    .chb =
        {
            .port = ENCODER_PIN_E1B_PORT,
            .pin = ENCODER_PIN_E1B_PIN,
        },
    .encoder_value = 0,
};

Encoder_t g_encoder2 = {
    .cha =
        {
            .port = ENCODER_PIN_E2A_PORT,
            .pin = ENCODER_PIN_E2A_PIN,
        },
    .chb =
        {
            .port = ENCODER_PIN_E2B_PORT,
            .pin = ENCODER_PIN_E2B_PIN,
        },
    .encoder_value = 0,
};

PID wheel1_vel_pid = {
    .Kp = 1200.0f, .Ki = 10.0f, .limit = 400.0f, .output_limit = 1000.0f};
PID wheel2_vel_pid = {
    .Kp = 1200.0f, .Ki = 10.0f, .limit = 400.0f, .output_limit = 1000.0f};

void SetupConfig() {
  L298N_Init(&g_l298n);
  EncoderInit(&g_encoder1);
  EncoderInit(&g_encoder2);
  g_serial = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);
  zdt_serial=SerialInit(UART_1_INST,SERIAL_MODE_IT, NULL, NULL);
  NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
  NVIC_EnableIRQ(ENCODER_PIN_GPIOB_INT_IRQN);
  NVIC_EnableIRQ(ENCODER_PIN_GPIOA_INT_IRQN);
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

  GWModelInit();        //寻迹模块初始化
}
