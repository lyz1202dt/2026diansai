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
#include "Bsp/OLED.h"
#include <ti/driverlib/dl_dma.h>
#include "Lib/PID.h"

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

Encoder_t g_encoder2 = {
    .cha = {
        .port = ENCODER_PIN_E2A_PORT,
        .pin = ENCODER_PIN_E2A_PIN,
    },
    .chb = {
        .port = ENCODER_PIN_E2B_PORT,
        .pin = ENCODER_PIN_E2B_PIN,
    },
    .encoder_value = 0,
};

void send_done(void *param) {}

void recv_done(void *param) {}

uint16_t adc_value_group[8];
bool adc_success=false;
float mpu6050_quat[4];
float mpu6050_roll;
float mpu6050_pitch;
float mpu6050_yaw;
bool mpu6050_success=false;
bool enable_dir_control=false;

int16_t m1_value,m2_value;
float m1_exp_vel,m2_exp_vel;
PID wheel1_vel_pid={.Kp=1200.0f,.Ki=10.0f,.limit=400.0f,.output_limit=1000.0f};
PID wheel2_vel_pid={.Kp=1200.0f,.Ki=10.0f,.limit=400.0f,.output_limit=1000.0f};

TaskHandle_t wheel_task_handle;

void WheelTask(void* param);

int app_main() {

  L298N_Init(&g_l298n);
  EncoderInit(&g_encoder1);
  EncoderInit(&g_encoder2);
  MPU6050_Init();

  //GWModelInit();
  // OLED_Init();
  // OLED_ShowString(0, 0, 16, "Hello World");
  //g_serial = SerialInit(UART_0_INST, SERIAL_MODE_IT, NULL, NULL);
  //NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
  NVIC_EnableIRQ(ENCODER_PIN_GPIOB_INT_IRQN);
  NVIC_EnableIRQ(ENCODER_PIN_GPIOA_INT_IRQN);
  xTaskCreate(WheelTask, "wheel_task", 256,NULL, 5,&wheel_task_handle);
  //NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
  //SerialTransmit(g_serial, send_str, 6, send_done);
  //SerialReceive(g_serial, revb_str, 8, 100, NULL);

TickType_t pxPreviousWakeTime=xTaskGetTickCount();
  while (1) {
    //SerialTransmit(g_serial, send_str, 3, send_done);
    // DL_GPIO_setPins(LED_PORT, LED_LED0_PIN_PIN);
    // vTaskDelay(50);
    // DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN_PIN);
    // vTaskDelay(50);

        mpu6050_success=(read_quad(mpu6050_quat)==0);
        if(mpu6050_success) {
            get_euler_angles(mpu6050_quat, NULL,NULL, &mpu6050_yaw);
        }
    
    //L298N_SetPWMValue(&g_l298n, 1, m1_value);
    //L298N_SetPWMValue(&g_l298n, 2, m2_value);
    //adc_success=GWGetState(adc_value_group);
    //mpu6050_success=(read_quad(mpu6050_quat)==0);
    //if(mpu6050_success) {
    //  get_euler_angles(mpu6050_quat, &mpu6050_roll, &mpu6050_pitch, &mpu6050_yaw);
    //}
    vTaskDelayUntil(&pxPreviousWakeTime,pdMS_TO_TICKS(5));
  }
  return 0;
}

float m1_cur_vel,m2_cur_vel;
float exp_yaw=0.0f;   //单位是度
float robot_exp_vel=0.0f;
float dir_kp=1.0f;
float filter_gate=0.8f;
float yaw_offset=0.0f;

static float NormalizeAngle180(float angle_deg)
{
    while(angle_deg >= 180.0f)
        angle_deg -= 360.0f;
    while(angle_deg < -180.0f)
        angle_deg += 360.0f;
    return angle_deg;
}

void WheelTask(void* param)
{
    int m1_last_enc=0,m2_last_enc=0;
    bool last_enable_dir_control_state=false;
    TickType_t pxPreviousWakeTime=xTaskGetTickCount();
    while(1)
    {
        float m1_omega=(g_encoder1.encoder_value-m1_last_enc)/52.0f*3.14159265f*2.0f/28.0f/0.005f;  //差分求瞬时速度
        float m2_omega=(g_encoder2.encoder_value-m2_last_enc)/52.0f*3.14159265f*2.0f/28.0f/0.005f;
        m1_last_enc=g_encoder1.encoder_value;
        m2_last_enc=g_encoder2.encoder_value;
        
        m1_cur_vel=filter_gate*m1_cur_vel+(1.0f-filter_gate)*m1_omega;    //速度滤波
        m2_cur_vel=filter_gate*m2_cur_vel+(1.0f-filter_gate)*m2_omega;

        //如果发生状态切变，记录当前yaw
        if(enable_dir_control==true&&last_enable_dir_control_state==false&&mpu6050_success)
            yaw_offset=mpu6050_yaw;
        last_enable_dir_control_state=enable_dir_control;
      

      m1_exp_vel=robot_exp_vel;
      m2_exp_vel=-robot_exp_vel;
      //角度闭环
      if(enable_dir_control&&mpu6050_success)
      {
        float yaw_error=NormalizeAngle180(exp_yaw+yaw_offset-mpu6050_yaw);
        m1_exp_vel-=dir_kp*yaw_error;
        m2_exp_vel-=dir_kp*yaw_error;
      }
      
      //计算轮子期望速度
      
      

      PID_Control(m1_cur_vel, m1_exp_vel, &wheel1_vel_pid);
      PID_Control(m2_cur_vel, m2_exp_vel, &wheel2_vel_pid);

      //摩擦和死区补偿
        if(m1_exp_vel>0.0f)
          wheel1_vel_pid.pid_out+=400.0f;
        else if(m1_exp_vel<0.0f)
          wheel1_vel_pid.pid_out-=400.0f;

        if(m2_exp_vel>0.0f)
          wheel2_vel_pid.pid_out+=400.0f;
        else if(m2_exp_vel<0.0f)
          wheel2_vel_pid.pid_out-=400.0f;

        
        L298N_SetPWMValue(&g_l298n, 1, wheel1_vel_pid.pid_out);
        L298N_SetPWMValue(&g_l298n, 2, wheel2_vel_pid.pid_out);
        
        
        vTaskDelayUntil(&pxPreviousWakeTime,pdMS_TO_TICKS(5));
    }
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
      EncoderUpdate(&g_encoder2, GPIOA, interrupt_status);
      DL_GPIO_clearInterruptStatus(GPIOA, interrupt_status);
    }
    break;
  }

  default:
    break;
  }
}
