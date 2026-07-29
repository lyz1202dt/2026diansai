#include "mytask.h"

#include "Lib/kalman/kalman.h"
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

//
extern float vofa_value[4];
extern uint8_t k230_cmd;
extern bool task_running;

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

float sum_distance=0.0f;


extern RecvPack k230_comm_recv_pack;

#define WHEEL_RADIUS_M 0.033f
#define WHEEL_TASK_PERIOD_S 0.005f
#define DEG_TO_RAD 0.01745329252f
#define WHEEL_ENCODER_CPR 52.0f
#define WHEEL_REDUCTION_RATIO 28.0f
#define TWO_PI 6.28318530718f

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
  int32_t m1_last_enc = 0, m2_last_enc = 0;
  bool last_enable_dir_control_state = false;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while (1) {
    int32_t m1_cur_enc = g_encoder1.encoder_value;
    int32_t m2_cur_enc = g_encoder2.encoder_value;
    float m1_omega = (m1_cur_enc - m1_last_enc) / WHEEL_ENCODER_CPR *
                     TWO_PI / WHEEL_REDUCTION_RATIO /
                     WHEEL_TASK_PERIOD_S; // 差分求瞬时速度
    float m2_omega = (m2_cur_enc - m2_last_enc) / WHEEL_ENCODER_CPR *
                     TWO_PI / WHEEL_REDUCTION_RATIO / WHEEL_TASK_PERIOD_S;
    float average_wheel_enc = ((float)m1_cur_enc - (float)m2_cur_enc) * 0.5f;
    sum_distance = average_wheel_enc / WHEEL_ENCODER_CPR * TWO_PI /
                   WHEEL_REDUCTION_RATIO * WHEEL_RADIUS_M;
    m1_last_enc = m1_cur_enc;
    m2_last_enc = m2_cur_enc;

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

    vofa_value[0]=m1_cur_omega;
    vofa_value[1]=m1_exp_omega;
    vofa_value[2]=m2_cur_omega;
    vofa_value[3]=m2_exp_omega;

    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(5));
  }
}

float line_track_omega = 0.0f;
float line_track_vel = 0.0f;
uint16_t adc_value_group[8];
uint16_t is_line_gate = 800;
bool adc_success = false;

float kExpTrackVel=0.10f;
#define IS_LINE_GATE_UPDATE_ALPHA 0.05f

bool is_line(uint16_t value) { return value < is_line_gate; }

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

bool enable_line_track = false;
void LineTrack(void *param) {
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
      line_track_vel = kExpTrackVel;
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

    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(40));
  }
}


#define SetMotorVel(id,omega) Emm_V5_Vel_Control(id, omega>=0.0f?0:1, (uint16_t)(ABS(omega*(60.0f/(2.0f*3.14159265f)))), 0, 0)

uint8_t zdt_recv_buf[32];
uint8_t zdt_recv_cnt;
float joint_cur_pos;


#define RAD2ANGLE(x) ((x)*180.0f/3.14159265f)
#define ANGLE2RAD(x) ((x)*3.14159265f/180.0f)
#define SUPPORT_STICK_RADIUS 0.05f 
#define BASE_HEIGHT     0.04f
#define STICK_LENGTH    0.25f
#define PIXEL2POSITION(x) (x*0.01f+0.15f)

float motor_base_angle_offset=0.1f;

static float stick_angle_to_motor_angle(float angle)
{
    float motor_sin =
        (STICK_LENGTH * sinf(ANGLE2RAD(angle)) + BASE_HEIGHT) /
        (2.0f * SUPPORT_STICK_RADIUS);

    if (motor_sin > 1.0f) {
        motor_sin = 1.0f;
    } else if (motor_sin < -1.0f) {
        motor_sin = -1.0f;
    }

    return RAD2ANGLE(asinf(motor_sin))+motor_base_angle_offset;
}

static float motor_angle_to_stick_angle(float angle)
{
    float dh=2.0f*sinf(ANGLE2RAD(angle-motor_base_angle_offset))*SUPPORT_STICK_RADIUS-BASE_HEIGHT;
    float stick_rad=asinf(dh/STICK_LENGTH);
    return RAD2ANGLE(stick_rad);
}

Kalman1D filter;

float stick_exp_angle=0.0f;
float motor_exp_omega=0.0f;
float motor_exp_pos=0.0f;

float exp_ball_pos=0.0f;
float stick_cur_angle=0.0f;

float k_joint_idel_angle=45.0f;

//电机位置环
PID motor_pos_pid={.Kp=0.0f,.Kd=0.0f,.Ki=0.0f,.limit=100.0f,.output_limit=20.0f};

PID ball_pos_pid={.Kp=0.0f,.Kd=0.0f,.Ki=0.0f,.limit=100.0f,.output_limit=15.0f};

bool enable_ball_pos_control=false;
//钢球位置控制
void ZDTDriver(void* param)
{
    Kalman1D_Init(&filter, 0.0f, 0.0f, 1.0f, 1.0f, 0.1f);
    //初始化张大头串口环境
    MakeZDTSerialEnv(zdt_serial);
    BaseType_t last_wake_time=xTaskGetTickCount();
    while(1)
    {
        Emm_V5_Read_Sys_Params(0x03, S_CPOS);
        uart_Receive_Data(zdt_recv_buf, 8,&zdt_recv_cnt);
        Emm_V5_GetPos(0x03,zdt_recv_buf,&joint_cur_pos);

        
        //用滤波后小球位置跑PID
        if(enable_ball_pos_control)
        {
            PID_Control(filter.position, exp_ball_pos, &ball_pos_pid);
            motor_exp_pos=stick_angle_to_motor_angle(ball_pos_pid.pid_out);
        }
        else{
            motor_exp_pos=k_joint_idel_anglef;  //如果不执行平衡控制，那么保持电机位置在中性点位置
        }
        PID_Control(joint_cur_pos, motor_exp_pos, &motor_pos_pid);
        SetMotorVel(0x03,motor_exp_omega+motor_pos_pid.pid_out);
        uart_Receive_Data(zdt_recv_buf,4, &zdt_recv_cnt);
        vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(4));
    }
}


char vofa_data_buffer[128];
uint16_t current_vofa_data_size=1;
float vofa_value[4];
void VOFA_Task(void* param)
{
  TickType_t last_wake_time=xTaskGetTickCount();
  while(1)
  {
    sprintf(vofa_data_buffer,"%.3f,%.3f,%.3f,%.3f\n",vofa_value[0],vofa_value[1],vofa_value[2],vofa_value[3]);
    SerialTransmit(g_serial,  vofa_data_buffer, strlen(vofa_data_buffer));
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
  }
}


//K230通讯任务
#define K230_COMM_HEAD 0x5AU

uint8_t k230_comm_recv_buffer[sizeof(RecvPack)];
volatile uint32_t k230_comm_valid_cnt;
volatile uint32_t k230_comm_header_err_cnt;
volatile uint32_t k230_comm_check_err_cnt;
volatile uint32_t k230_comm_timeout_cnt;

float raw_position,filtered_position;
float raw_acc;

static uint8_t K230CommCalcCheck(const uint8_t *buffer)
{
    uint8_t check = 0U;

    for (uint32_t i = 0U; i < (sizeof(RecvPack) - 1U); i++) {
        check += buffer[i];
    }

    return check;
}

static bool K230CommFrameValid(const uint8_t *buffer)
{
    return (buffer[0] == K230_COMM_HEAD) &&
           (K230CommCalcCheck(buffer) == buffer[sizeof(RecvPack) - 1U]);
}

static uint16_t K230CommResync(uint8_t *buffer)
{
    for (uint16_t i = 1U; i < sizeof(RecvPack); i++) {
        if (buffer[i] == K230_COMM_HEAD) {
            uint16_t remain = (uint16_t)sizeof(RecvPack) - i;
            memmove(buffer, &buffer[i], remain);
            return remain;
        }
    }

    return 0U;
}

TickType_t last_ball_pos_update_time=0;
void k230_pack_parse(uint8_t *src)
{
    RecvPack recv_pack;
    memcpy(&recv_pack, src, sizeof(k230_comm_recv_pack));
    raw_position=PIXEL2POSITION(recv_pack.position);

    stick_cur_angle=motor_angle_to_stick_angle(joint_cur_pos);  //求解棍子角度，计算加速度作为滤波器输入
    raw_acc=sinf(ANGLE2RAD(stick_cur_angle))*9.8f;
    
    float dt=(xTaskGetTickCount()-last_ball_pos_update_time);
    if(dt>0.08f)    //最多容忍两次丢帧，防止时间过大导致滤波器崩溃
        dt=0.07f;
    last_ball_pos_update_time=xTaskGetTickCount();

    Kalman1D_Update(&filter,raw_position , raw_acc, dt);
}

void K230RecvTask(void* param)
{
    uint16_t recv_index = 0U;

    while(1)
    {
        uint8_t byte = 0U;
        int recv_size = SerialReceive(g_serial, &byte, 1U, 10);

        if (recv_size != 1) {
            k230_comm_timeout_cnt++;
            continue;
        }

        if ((recv_index == 0U) && (byte != K230_COMM_HEAD)) {
            k230_comm_header_err_cnt++;
            continue;
        }

        k230_comm_recv_buffer[recv_index] = byte;
        recv_index++;

        if (recv_index < sizeof(RecvPack)) {
            continue;
        }

        if (K230CommFrameValid(k230_comm_recv_buffer)) {
            k230_pack_parse(k230_comm_recv_buffer);
            k230_comm_valid_cnt++;
            recv_index = 0U;
        }
        else {
            k230_comm_check_err_cnt++;
            recv_index = K230CommResync(k230_comm_recv_buffer);
        }
    }
}


float task1_finished_gate_distance=3.0f;
float task1_distance_offset;
TickType_t task1_start_time;
void Task1(void* parma)
{
    task_running=true;
    vTaskDelay(pdMS_TO_TICKS(500));
    kExpTrackVel=0.2f;
    task1_start_time=xTaskGetTickCount();
    TickType_t pxPreviousWakeTime = task1_start_time;
    enable_line_track=true;
    task1_distance_offset=sum_distance;

    while(sum_distance-task1_distance_offset>task1_finished_gate_distance)      //高速行驶到停止线前
    {
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(100));
    }



    kExpTrackVel=0.1f;      //缓慢行驶直到遇到停止线
    while(!(is_line(adc_value_group[2])&&is_line(adc_value_group[3])&&is_line(adc_value_group[4])&&is_line(adc_value_group[5])))      //行驶到终点前附近
    {
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(30));
    }

    enable_line_track=false;
    OLED_Printf(20, 0, 16, "time=%dms", xTaskGetTickCount()-task1_start_time);

    //清理现场
    task_running=false;
    vTaskDelete(NULL);
    while(1){vTaskDelay(1000);}
}


void Task2(void* parma)
{
    task_running=true;
    vTaskDelay(pdMS_TO_TICKS(500));

    //尚未编写
    task_running=false;
    vTaskDelete(NULL);
    while(1){vTaskDelay(1000);}
}
