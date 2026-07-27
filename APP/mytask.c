#include "mytask.h"

#include "portmacro.h"
#include "projdefs.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include <FreeRTOS.h>
#include <math.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <task.h>
#include <math.h>

/* 使用 driverlib 的 DMA 接口 */
#include "Bsp/OLED.h"
#include "Bsp/gw_model.h"
#include "Bsp/motor.h"
#include "Bsp/mpu6050.h"
#include "Driver/uart/uart.h"
#include "Driver/zdt/Emm_V5.h"
#include "Driver/zdt/uartport.h"
#include "Lib/pid/PID.h"
#include <ti/driverlib/dl_dma.h>

#include "config.h"

// 寻迹模块

// IMU
float mpu6050_yaw;
bool mpu6050_success = false;

// Chassis
bool enable_dir_control = false;
bool enabl_odometer = true;
float exp_yaw = 0.0f; // 单位是度
float dir_kp = 1.0f;

float m1_cur_omega, m2_cur_omega;
float filter_gate = 0.8f;
float yaw_offset = 0.0f;
float robot_exp_vel = 0.0f;
float robot_exp_omega = 0.0f;

float cur_robot_pos_x, cur_robot_pos_y;
float cur_robot_yaw, odom_yaw_offset;

#define WHEEL_RADIUS_M 0.033f
#define WHEEL_TASK_PERIOD_S 0.005f
#define DEG_TO_RAD 0.01745329252f

void IMUTask(void *param) {
  MPU6050_Init();
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  float mpu6050_quat[4];
  while (1) {
    mpu6050_success = (read_quad(mpu6050_quat) == 0);
    if (mpu6050_success) {
      get_euler_angles(mpu6050_quat, NULL, NULL, &mpu6050_yaw);
    }
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(5));
  }
}

static float NormalizeAngle180(float angle_deg) {
  while (angle_deg >= 180.0f)
    angle_deg -= 360.0f;
  while (angle_deg < -180.0f)
    angle_deg += 360.0f;
  return angle_deg;
}

void WheelTask(void *param) {
  int m1_last_enc = 0, m2_last_enc = 0;
  bool last_enable_dir_control_state = false;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while (1) {
    float m1_omega = (g_encoder1.encoder_value - m1_last_enc) / 52.0f *
                     3.14159265f * 2.0f / 28.0f / 0.005f; // 差分求瞬时速度
    float m2_omega = (g_encoder2.encoder_value - m2_last_enc) / 52.0f *
                     3.14159265f * 2.0f / 28.0f / 0.005f;
    m1_last_enc = g_encoder1.encoder_value;
    m2_last_enc = g_encoder2.encoder_value;

    m1_cur_omega = filter_gate * m1_cur_omega +
                   (1.0f - filter_gate) * m1_omega; // 速度滤波
    m2_cur_omega = filter_gate * m2_cur_omega + (1.0f - filter_gate) * m2_omega;

    // 如果发生状态切变，记录当前yaw
    if (enable_dir_control == true && last_enable_dir_control_state == false &&
        mpu6050_success)
      yaw_offset = mpu6050_yaw;
    last_enable_dir_control_state = enable_dir_control;

    float m1_exp_omega = robot_exp_vel / WHEEL_RADIUS_M +
                         robot_exp_omega * 0.104 /
                             0.033; // 机器人角速度*机身半径/轮子半径=轮子角速度
    float m2_exp_omega =
        -robot_exp_vel / WHEEL_RADIUS_M + robot_exp_omega * 0.104 / 0.033;
    // 角度闭环
    if (enable_dir_control && mpu6050_success) {
      float yaw_error = NormalizeAngle180(exp_yaw + yaw_offset - mpu6050_yaw);
      m1_exp_omega -= dir_kp * yaw_error;
      m2_exp_omega -= dir_kp * yaw_error;
    }

    // 计算轮子期望速度

    PID_Control(m1_cur_omega, m1_exp_omega, &wheel1_vel_pid);
    PID_Control(m2_cur_omega, m2_exp_omega, &wheel2_vel_pid);

    // 摩擦和死区补偿
    if (m1_exp_omega > 0.0f)
      wheel1_vel_pid.pid_out += 400.0f;
    else if (m1_exp_omega < 0.0f)
      wheel1_vel_pid.pid_out -= 400.0f;

    if (m2_exp_omega > 0.0f)
      wheel2_vel_pid.pid_out += 400.0f;
    else if (m2_exp_omega < 0.0f)
      wheel2_vel_pid.pid_out -= 400.0f;

    L298N_SetPWMValue(&g_l298n, 1, wheel1_vel_pid.pid_out);
    L298N_SetPWMValue(&g_l298n, 2, wheel2_vel_pid.pid_out);

    if (enabl_odometer) // 里程计计算
    {
      cur_robot_yaw = NormalizeAngle180(mpu6050_yaw - odom_yaw_offset);
      float yaw_rad = cur_robot_yaw * DEG_TO_RAD;
      float linear_vel = (m1_cur_omega - m2_cur_omega) * WHEEL_RADIUS_M * 0.5f;
      float vx = linear_vel * cosf(yaw_rad);
      float vy = linear_vel * sinf(yaw_rad);
      cur_robot_pos_x += vx * WHEEL_TASK_PERIOD_S;
      cur_robot_pos_y += vy * WHEEL_TASK_PERIOD_S;
    }

    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(5));
  }
}

float line_track_omega = 0.0f;
float line_track_vel = 0.0f;
uint16_t adc_value_group[8];
uint16_t is_line_gate = 800;
bool adc_success = false;

#define LINE_TRACK_VEL 0.10f
#define IS_LINE_GATE_UPDATE_ALPHA 0.05f

static bool is_line(uint16_t value) { return value < is_line_gate; }

void update_is_line_gate(void) {
  uint16_t min_value = adc_value_group[0];
  uint16_t max_value = adc_value_group[0];

  for (int i = 1; i < 8; i++) {
    if (adc_value_group[i] < min_value) {
      min_value = adc_value_group[i];
    }
    if (adc_value_group[i] > max_value) {
      max_value = adc_value_group[i];
    }
  }

  float current_gate = ((float)min_value + (float)max_value) * 0.5f;

  if(max_value-current_gate>200&&current_gate-min_value>200)    //如果差别足够大，才认为可以用来区分
  {
    float filtered_gate =
      (1.0f - IS_LINE_GATE_UPDATE_ALPHA) * (float)is_line_gate +
      IS_LINE_GATE_UPDATE_ALPHA * current_gate;
  is_line_gate = (uint16_t)(filtered_gate + 0.5f);
  }
}

QueueHandle_t line_msg_queue;
bool enable_line_track = false;
void LineTrack(void *param) {
  line_msg_queue=xQueueCreate(4, sizeof(uint32_t));
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  const float omega_weight[8] = {-1.2f, -0.7f, -0.3f, -0.1f,
                                 0.1f, 0.3f,  0.7f,  1.2f};
  vTaskDelay(pdMS_TO_TICKS(3000));

  for(int i=0;i<40;i++)   //现场采集2s环境光信息
  {
    adc_success = GWGetState(adc_value_group);
    if(adc_success)
      update_is_line_gate();
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(50));
  }
  
  while (1) {
    adc_success = GWGetState(adc_value_group);
    if (adc_success) {
      float detected_omega = 0.0f;
      bool line_detected = false;

      for (int i = 0; i < 8; i++) {
        if (is_line(adc_value_group[i])) {
          detected_omega += omega_weight[i];
          line_detected = true;
        }
      }

      if (line_detected) {
        line_track_omega = 0.3*detected_omega+0.7*line_track_omega;
      }
      else {
        if(line_track_omega>0.0f)
          line_track_omega=1.0f;
        if(line_track_omega<0.0f)
          line_track_omega=-1.0f;
      }
      line_track_vel = LINE_TRACK_VEL;
    } else {
      line_track_omega = 0.0f;
      line_track_vel = 0.0f;
    }

    // 操控底盘运动

    if (enable_line_track) {
      enable_dir_control = false;
      robot_exp_vel = line_track_vel;
      robot_exp_omega = line_track_omega;
    }
    else {
      robot_exp_vel = 0.0f;
      robot_exp_omega = 0.0f;
    }

    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(50));
  }
}


#define SetMotorVel(id,omega) Emm_V5_Vel_Control(id, omega>=0.0f?0:1, (uint16_t)(ABS(omega*(60.0f/(2.0f*3.14159265f)))), 0, 0)

uint8_t zdt_recv_buf[32];
uint8_t zdt_recv_cnt;
float joint_cur_pos;

float exp_omega=0.0f;

void ZDTDriver(void* param)
{
    //初始化张大头串口环境
    MakeZDTSerialEnv(zdt_serial);
    BaseType_t last_wake_time=xTaskGetTickCount();
    while(1)
    {
        Emm_V5_Read_Sys_Params(0x03, S_CPOS);
        uart_Receive_Data(zdt_recv_buf, 8,&zdt_recv_cnt);
        Emm_V5_GetPos(0x03,zdt_recv_buf,&joint_cur_pos);

        SetMotorVel(0x03,exp_omega/*joint_target[0].omega+joint1_pid.pid_out*/);
        uart_Receive_Data(zdt_recv_buf,4, &zdt_recv_cnt);
        vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(10));
    }
}


char vofa_data_buffer[128];
uint16_t current_vofa_data_size=1;
float vofa_value[3];
void VOFA_Task(void* param)
{
  int x=0;
  TickType_t last_wake_time=xTaskGetTickCount();
  while(1)
  {
    vofa_value[0]=2.0*sin(x*0.01)+0.5*sin(3.1*x*0.01);
    vofa_value[1]=3.0*sin(2.0*x*0.01)+0.2*sin(0.5*x*0.01);
    vofa_value[2]=0.2*sin(5.0*x*0.01)+0.2*sin(1.6*x*0.01);
    sprintf(vofa_data_buffer,"%.3f,%.3f,%.3f\n",vofa_value[0],vofa_value[1],vofa_value[2]);
    SerialTransmit(g_serial,  vofa_data_buffer, strlen(vofa_data_buffer));
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    x=(x+1)%1000;
  }
}