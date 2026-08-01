#include "mytask.h"

#include "Lib/kalman/kalman.h"
#include "portmacro.h"
#include "projdefs.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_gpio.h"
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
#include "Lib/quintic/quintic.h"
#include "Lib/kalman/kalman.h"
#include <ti/driverlib/dl_dma.h>

#include "config.h"

//
extern float vofa_value[4];
extern uint8_t k230_cmd;
extern bool task_running;
extern bool force_exit;
extern int current_task_id;
extern char vofa_data_buffer[128];


// IMU
float mpu6050_yaw;
float mpu6050_yaw_rate_dps;
float mpu6050_accel_x;
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
#define IMU_DMP_PERIOD_S 0.02f
Kalman1D acc_x_filter;


void IMUTask(void *param) {
  MPU6050_Init();
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  float acc_x_offset=0.0f;
  float acc_x_raw=0.0f;
  float mpu6050_quat[4];
  float mpu6050_gyro_dps[3];
  float mpu6050_accel_sample_g[3];
  int imu_success_cnt=0;
  
  while(imu_success_cnt<100)
  {
    mpu6050_success =
        (read_imu(mpu6050_quat, mpu6050_gyro_dps, mpu6050_accel_sample_g) == 0);
    if (mpu6050_success) {
      get_euler_angles(mpu6050_quat, NULL, NULL, &mpu6050_yaw);
      mpu6050_yaw_rate_dps = mpu6050_gyro_dps[2];
      acc_x_offset+=(9.8f*mpu6050_accel_sample_g[0])*0.01f;
      imu_success_cnt++;
    }
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(5));
  }
  Kalman1D_Init(&acc_x_filter, 0.0f, 0.0f,1.0e-5f,
                4.03e-4f, 4.03e-4f);
  while (1) {
    mpu6050_success =
        (read_imu(mpu6050_quat, mpu6050_gyro_dps, mpu6050_accel_sample_g) == 0);
    if (mpu6050_success) {
      get_euler_angles(mpu6050_quat, NULL, NULL, &mpu6050_yaw);
      mpu6050_yaw_rate_dps = mpu6050_gyro_dps[2];
      acc_x_raw=9.8f*mpu6050_accel_sample_g[0]-acc_x_offset;

      Kalman1D_Update(&acc_x_filter, acc_x_raw, IMU_DMP_PERIOD_S);

      mpu6050_accel_x = acc_x_filter.position-mpu6050_yaw_rate_dps*mpu6050_yaw_rate_dps*DEG_TO_RAD*DEG_TO_RAD*0.14f;    //补偿自旋引起的加速度

      //陀螺仪滤波处理
    }
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(20));
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

    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(5));
  }
}


static bool is_line(uint8_t index,uint8_t value)
{
  return ((~value)>>index)&0x01;
}

static uint8_t count_line_sensors(uint8_t value)
{
  uint8_t count = 0;

  for (uint8_t i = 0; i < 8; i++) {
    if (is_line(i, value)) {
      count++;
    }
  }

  return count;
}



float line_trace_exp_vel=0.10f;
float line_trace_exp_omega=0.0f;

bool line_detected=false;
bool enable_line_track = false;
bool ignore_line_sensor=false;
uint8_t line_trace_result;
void LineTrack(void *param) {
TickType_t pxPreviousWakeTime = xTaskGetTickCount();
const float omega_weight[8] = {-1.5f, -0.9f, -0.3f, -0.1f,
                                 0.1f, 0.3f,  0.9f,  1.5f};
float line_track_omega = 0.0f;
float line_track_vel = 0.0f;
float last_detected_omega = 0.0f;
  while (1) {
    line_trace_result=GWGetState();
    if(ignore_line_sensor)
      line_trace_result=0b11100111;     //Debug

    float detected_omega=0.0f;
    bool detected_any_line = false;
      for (int i = 0; i < 8; i++) {
        if (is_line(i,line_trace_result)) {
          detected_omega += omega_weight[i];
          detected_any_line = true;
        }
      }
      if (detected_any_line) {
        last_detected_omega = detected_omega;
      } else {
        detected_omega = last_detected_omega;
      }

      line_track_omega = 0.3*detected_omega+0.7*line_track_omega;
      line_track_vel = line_trace_exp_vel;

    // 操控底盘运动

    if (enable_line_track) {
      enable_dir_control = false;
      robot_exp_vel = line_track_vel;
      robot_exp_omega = line_track_omega+line_trace_exp_omega;
    }
    else
    {
      robot_exp_vel = 0.0f;
      robot_exp_omega = 0.0f;
    }

    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(40));
  }
}


static void SetMotorVel(uint8_t id, float omega)
{
    Emm_V5_Vel_Control(id, (omega)>=0.0f?0:1, (uint16_t)(ABS((omega)*(60.0f/(2.0f*3.14159265f)))), 0, 0);
}


uint8_t zdt_recv_buf[32];
uint8_t zdt_recv_cnt;
float joint_cur_pos;

float kBallDistanceOffset=-0.006f;

#define RAD2ANGLE(x) ((x)*180.0f/3.14159265f)
#define ANGLE2RAD(x) ((x)*3.14159265f/180.0f)
#define SUPPORT_STICK_RADIUS 0.053f 
#define BASE_HEIGHT     0.053f
#define STICK_LENGTH    0.25f
#define PIXEL2POSITION(x) ((x)*0.01f+kBallDistanceOffset)   //换算成国际单位
#define BALL_POS_TO_CENTER_DIS(x) ((-x)+0.11f)


float motor_base_angle_offset=-2.3f;    //注意：单位是度

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


//参数常量
const float k_joint_idel_angle=28.0f;

//对外暴露的接口
bool enable_ball_pos_control=false;
bool car_is_stop=true;
float exp_ball_pos=0.0f;
float acc_feedforward=0.0f;

float stick_current_angle=0.0f;

float stick_angle_feedforward=0.0f;

//电机位置环
PID motor_pos_pid={.Kp=0.17f,.Kd=0.3f,.Ki=0.0f,.limit=100.0f,.output_limit=10.0f};


Kalman1D ball_filter;
float stick_exp_angle=0.0f;
float motor_exp_omega=0.0f;
float motor_exp_pos=0.0f;

static float limit_motor_exp_pos_step(float target_pos,float cur_target_pos,float step)
{
    float delta = target_pos - cur_target_pos;

    if (delta > step) {
        return cur_target_pos + step;
    } else if (delta < -step) {
        return cur_target_pos - step;
    }
    return target_pos;
}

#define MOTOR_ID 0x02

float filtered_motor_exp_pos=0.0f;
float ball_pid_output_filter_gate=0.6f;
float max_motor_exp_step=1.0f;
//钢球位置控制
void ZDTDriver(void* param)
{
    Kalman1D_Init(&ball_filter, 0.0f, 0.0f, 20.0f, 0.001f, 0.1f);
    //初始化张大头串口环境
    MakeZDTSerialEnv(zdt_serial);
    filtered_motor_exp_pos=k_joint_idel_angle+motor_base_angle_offset;
    BaseType_t last_wake_time=xTaskGetTickCount();
    while(1)
    {
        float angle_temp=0.0f;
        Emm_V5_Read_Sys_Params(MOTOR_ID, S_CPOS);
        uart_Receive_Data(zdt_recv_buf, 8,&zdt_recv_cnt);
        Emm_V5_GetPos(MOTOR_ID,zdt_recv_buf,&angle_temp);
        joint_cur_pos=-angle_temp;

        if(!car_is_stop)
        {
            float yaw_rate_rad_s=mpu6050_yaw_rate_dps*DEG_TO_RAD;
            float forward_acc=BALL_POS_TO_CENTER_DIS(ball_filter.position)*yaw_rate_rad_s*yaw_rate_rad_s-acc_feedforward;
            float gravity_ratio=forward_acc/9.8f;

            if(gravity_ratio>1.0f)
              gravity_ratio=1.0f;
            else if(gravity_ratio<-1.0f)
              gravity_ratio=-1.0f;

            stick_angle_feedforward=RAD2ANGLE(asinf(gravity_ratio));    //补偿自旋和加速前进所需的角度
        }

        
        if(!enable_ball_pos_control)    //如果启用了视觉控制，那么将PID闭环放在接收中
          motor_exp_pos=k_joint_idel_angle+motor_base_angle_offset;

        //期望值滤波
        float limited_exp_pos=limit_motor_exp_pos_step(motor_exp_pos,filtered_motor_exp_pos,max_motor_exp_step);
        filtered_motor_exp_pos=ball_pid_output_filter_gate*filtered_motor_exp_pos+(1.0f-ball_pid_output_filter_gate)*limited_exp_pos;
        
        if(filtered_motor_exp_pos>50.0f)   //防止数据异常损坏电机
          filtered_motor_exp_pos=50.0f;
        else if(filtered_motor_exp_pos<0.0f)
          filtered_motor_exp_pos=0.0f;

        Emm_V5_Pos_ControlEx(MOTOR_ID,-filtered_motor_exp_pos,-joint_cur_pos,0.004f);
        uart_Receive_Data(zdt_recv_buf,4, &zdt_recv_cnt);
        vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(4));
    }
}


char vofa_data_buffer[128];
float vofa_value[4];
void VOFA_Task(void* param)
{
  TickType_t last_wake_time=xTaskGetTickCount();
  while(1)
  {
    sprintf(vofa_data_buffer,"%.3f,%.3f\n",mpu6050_yaw_rate_dps*mpu6050_yaw_rate_dps*DEG_TO_RAD*DEG_TO_RAD*0.14f,acc_x_filter.position);
    SerialTransmit(vofa_serial,  vofa_data_buffer, strlen(vofa_data_buffer));
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


PID ball_pos_pid={.Kp=2.5f,.Kd=0.0f,.Ki=0.0f,.limit=5.0f,.output_limit=0.09f};
PID ball_vel_pid={.Kp=30.0f,.Kd=0.0f,.Ki=1.0f,.limit=10.0f,.output_limit=6.0f};



void k230_pack_parse(uint8_t *src)
{
    RecvPack recv_pack;
    memcpy(&recv_pack, src, sizeof(k230_comm_recv_pack));
    raw_position=PIXEL2POSITION(recv_pack.position);
    
    float dt=(xTaskGetTickCount()-last_ball_pos_update_time)*0.001f;
    if(dt>0.08f)    //最多容忍两次丢帧，防止时间过大导致滤波器崩溃
        dt=0.08f;
    last_ball_pos_update_time=xTaskGetTickCount();
    
    if(raw_position<0.3f&&raw_position>-0.3f)   //数值合理才送到卡尔曼
      Kalman1D_Update(&ball_filter,raw_position, dt);

    
    if(enable_ball_pos_control)
    {
        PID_Control(ball_filter.position, exp_ball_pos, &ball_pos_pid);
        PID_Control(ball_filter.velocity,ball_pos_pid.pid_out, &ball_vel_pid);
        motor_exp_pos=stick_angle_to_motor_angle(stick_angle_feedforward+ball_vel_pid.pid_out);
    }

    //Debug
    // vofa_value[0]=raw_position;
    // vofa_value[1]=ball_filter.position;
    // vofa_value[2]=ball_filter.velocity;

    // sprintf(vofa_data_buffer,"%.3f,%.3f,%.3f\n",vofa_value[0],vofa_value[1],vofa_value[2]);
    // SerialTransmit(vofa_serial,  vofa_data_buffer, strlen(vofa_data_buffer));
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


float ball_pos_offset=0.0f;


float task1_finished_gate_distance=5.7f;
float task1_distance_offset;
TickType_t task1_start_time;
void Task1(void* parma)
{
    k230_cmd=0;
    task_running=true;
    vTaskDelay(pdMS_TO_TICKS(500));
    task1_start_time=xTaskGetTickCount();
    TickType_t pxPreviousWakeTime = task1_start_time;
    enable_line_track=true;
    task1_distance_offset=sum_distance;

    int cnt=0;
    ignore_line_sensor=true;
    line_trace_exp_vel=0.5f;
    while(sum_distance-task1_distance_offset<task1_finished_gate_distance)      //高速行驶到停止线前
    {
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(100));
        cnt++;
        if(cnt>5)
          ignore_line_sensor=false;
    }



    line_trace_exp_vel=0.1f;      //缓慢行驶直到遇到停止线
    while(count_line_sensors(line_trace_result) < 3)      //while退出的条件为任意3个传感器检测到黑线（黑线为0，白线为1）
    {
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(30));
    }

    enable_line_track=false;
    OLED_Printf(0, 0, 16, "time=%dms", xTaskGetTickCount()-task1_start_time);

    //清理现场
    force_exit=false;
    task_running=false;
    current_task_id=0;
    vTaskDelete(NULL);
    while(1){vTaskDelay(1000);}
}


void Task2(void* parma)
{
    float task2_start_pos;

    k230_cmd=1;
    task_running=true;
    enable_line_track=false;
    force_exit=false;
    enable_ball_pos_control=true;
    while(DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))    //等待直到按键按下，表示将当前钢球的位置设为期望它在运行时处于的位置
    {
        if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_UP_PIN))
            ball_pos_offset+=0.00005f;
        else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_DOWN_PIN))
            ball_pos_offset-=0.00005f;
        exp_ball_pos=ball_pos_offset;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    exp_ball_pos=-0.05f+ball_pos_offset;
    while(ball_filter.position>-0.042f+ball_pos_offset)
    {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    exp_ball_pos= 0.045f+ball_pos_offset;

    while(!force_exit)  //等待强制退出信号
    {
        vTaskDelay(200);
    }
    enable_ball_pos_control=false;
    k230_cmd=0;
    //清理现场
    task_running=false;
    force_exit=false;
    vTaskDelete(NULL);
    while(1){vTaskDelay(1000);}
}

Quintic task3_quintic;
float task3_exp_pos;
float task3_exp_vel;
float task3_exp_acc;
float task3_pos_kp=10.0f;
void Task3(void* param)
{
    TickType_t task3_start_time;
    TickType_t last_wake_time;

    k230_cmd=1;
    task_running=true;
    exp_ball_pos=0.0f;
    enable_ball_pos_control=true;
    car_is_stop=false;

    DL_GPIO_setPins(LED_RGB_PORT, LED_RGB_LED_R_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_G_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_B_PIN);

    while(DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))    //等待直到按键按下，表示将当前钢球的位置设为期望它在运行时处于的位置
    {
        if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_UP_PIN))
            ball_pos_offset+=0.00001f;
        else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_DOWN_PIN))
            ball_pos_offset-=0.00001f;
        exp_ball_pos=ball_pos_offset;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while(!DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))
    {
      vTaskDelay(pdMS_TO_TICKS(50));    //等待按键松开
    }
    DL_GPIO_clearPins(LED_RGB_PORT, LED_RGB_LED_R_PIN);
    DL_GPIO_setPins(LED_RGB_PORT,LED_RGB_LED_G_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_B_PIN);

    QuinticGenerate(&task3_quintic, sum_distance, 0.0f, sum_distance+1.7f, 0.0f, 7.0f);
    float init_distance=sum_distance;
    task3_start_time=xTaskGetTickCount();
    last_wake_time=task3_start_time;
    enable_line_track=true;

    bool trajectory_finished=false;
    while(!trajectory_finished && !force_exit)
    {
        float time=(xTaskGetTickCount()-task3_start_time)*portTICK_PERIOD_MS*0.001f;
        trajectory_finished=QuinticSample(time, &task3_exp_pos, &task3_exp_vel, &task3_exp_acc, &task3_quintic);
        acc_feedforward=0.5f*mpu6050_accel_x+0.5f*task3_exp_acc;
        line_trace_exp_vel=task3_exp_vel+task3_pos_kp*(task3_exp_pos-sum_distance);   //求循迹速度

        if(sum_distance- init_distance<0.1f)
          ignore_line_sensor=true;
        else
          ignore_line_sensor=false;
          

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
    }
    
    enable_line_track=false;
    enable_ball_pos_control=false;
    current_task_id=0;

    k230_cmd=0;
    task_running=false;
    force_exit=false;
    vTaskDelete(NULL);
    while(1){vTaskDelay(1000);}
}


#define TASK4_CRUISE_VEL_MPS 0.27f
#define TASK4_ARC_RADIUS_M 0.5f
#define TASK4_ARC_DISTANCE_M (3.14159265f*TASK4_ARC_RADIUS_M)
#define TASK4_STOP_LINE_APPROACH_DISTANCE_M 0.2f
#define TASK4_ARC_DIRECTION (1.0f)    //方向待测
#define TASK4_ARC_PERIOD_MS 20U
#define TASK4_ARC_OMEGA_RAMP_TIME_S 0.5f
#define TASK4_STOP_LINE_VEL_MPS 0.15f
#define TASK4_FINAL_STOP_DISTANCE_M 0.3f

static float Task4RampArcOmega(float target_omega, TickType_t start_time, float start_distance, float arc_distance)
{
    float elapsed_time=(xTaskGetTickCount()-start_time)*portTICK_PERIOD_MS*0.001f;
    float ramp_in_ratio;
    float ramp_out_distance=TASK4_CRUISE_VEL_MPS*TASK4_ARC_OMEGA_RAMP_TIME_S;
    float traveled_distance=sum_distance-start_distance;
    float remaining_distance=arc_distance-traveled_distance;
    float ramp_out_ratio;
    float ramp_ratio;

    if(arc_distance<=0.0f)
      return 0.0f;

    if(elapsed_time<=0.0f)
      ramp_in_ratio=0.0f;
    else if(elapsed_time>=TASK4_ARC_OMEGA_RAMP_TIME_S)
      ramp_in_ratio=1.0f;
    else
      ramp_in_ratio=elapsed_time/TASK4_ARC_OMEGA_RAMP_TIME_S;

    if(remaining_distance<=0.0f)
      ramp_out_ratio=0.0f;
    else if(remaining_distance>=ramp_out_distance)
      ramp_out_ratio=1.0f;
    else
      ramp_out_ratio=remaining_distance/ramp_out_distance;

    ramp_ratio=(ramp_in_ratio<ramp_out_ratio)?ramp_in_ratio:ramp_out_ratio;

    return target_omega*ramp_ratio;
}

void Task4(void* param)
{
    k230_cmd=1;
    task_running=true;
    exp_ball_pos=0.0f;
    enable_ball_pos_control=true;
    line_trace_exp_omega=0.0f;
    car_is_stop=false;
    
    DL_GPIO_setPins(LED_RGB_PORT, LED_RGB_LED_R_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_G_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_B_PIN);
    while(DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))    //等待直到按键按下，表示开始执行
    {
        if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_UP_PIN))
            ball_pos_offset+=0.00001f;
        else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_DOWN_PIN))
            ball_pos_offset-=0.00001f;
        exp_ball_pos=ball_pos_offset;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    while(!DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))    //等待按键松开
    {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    DL_GPIO_clearPins(LED_RGB_PORT, LED_RGB_LED_R_PIN);
    DL_GPIO_setPins(LED_RGB_PORT,LED_RGB_LED_G_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_B_PIN);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    float init_distance=sum_distance;

    {     //直线段到达第一个拐弯点
        TickType_t start_time;
        TickType_t last_wake_time;
        bool trajectory_finished=false;

        QuinticGenerate(&task3_quintic, sum_distance, 0.0f, sum_distance+1.6f,
                        TASK4_CRUISE_VEL_MPS, 8.3f);
        start_time=xTaskGetTickCount();
        last_wake_time=start_time;
        line_trace_exp_vel=0.0f;
        line_trace_exp_omega=0.0f;
        acc_feedforward=0.0f;
        ignore_line_sensor=true;
        enable_line_track=true;

        while(!trajectory_finished && !force_exit)
        {
            float time=(xTaskGetTickCount()-start_time)*portTICK_PERIOD_MS*0.001f;

            trajectory_finished=QuinticSample(time, &task3_exp_pos, &task3_exp_vel, &task3_exp_acc, &task3_quintic);
            acc_feedforward=0.5f*mpu6050_accel_x+0.5f*task3_exp_acc;
            line_trace_exp_vel=task3_exp_vel+task3_pos_kp*(task3_exp_pos-sum_distance);   //求循迹速度

            if(sum_distance- init_distance<0.1f)
              ignore_line_sensor=true;
            else
              ignore_line_sensor=false;

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)   //走第一个圆弧
    {
        TickType_t arc_start_time=xTaskGetTickCount();
        TickType_t last_wake_time=arc_start_time;
        float start_distance=sum_distance;
        float target_omega=TASK4_ARC_DIRECTION*TASK4_CRUISE_VEL_MPS/TASK4_ARC_RADIUS_M;
        bool arc_finished=false;

        line_trace_exp_vel=TASK4_CRUISE_VEL_MPS;
        line_trace_exp_omega=0.0f;
        acc_feedforward=0.0f;
        enable_line_track=true;

        while(!arc_finished && !force_exit)
        {
            line_trace_exp_omega=Task4RampArcOmega(target_omega, arc_start_time, start_distance, TASK4_ARC_DISTANCE_M);
            arc_finished=(sum_distance-start_distance>=TASK4_ARC_DISTANCE_M);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK4_ARC_PERIOD_MS));
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)   //第二段直线
    {
        TickType_t start_time;
        TickType_t last_wake_time;
        bool trajectory_finished=false;

        QuinticGenerate(&task3_quintic, sum_distance, TASK4_CRUISE_VEL_MPS, sum_distance+1.5f,
                        TASK4_CRUISE_VEL_MPS,5.0f);
        start_time=xTaskGetTickCount();
        last_wake_time=start_time;
        line_trace_exp_omega=0.0f;
        enable_line_track=true;

        while(!trajectory_finished && !force_exit)
        {
            float time=(xTaskGetTickCount()-start_time)*portTICK_PERIOD_MS*0.001f;

            trajectory_finished=QuinticSample(time, &task3_exp_pos, &task3_exp_vel, &task3_exp_acc, &task3_quintic);
            acc_feedforward=0.5f*mpu6050_accel_x+0.5f*task3_exp_acc;
            line_trace_exp_vel=task3_exp_vel+task3_pos_kp*(task3_exp_pos-sum_distance);   //求循迹速度
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)       //第二段圆弧
    {
        TickType_t arc_start_time=xTaskGetTickCount();
        TickType_t last_wake_time=arc_start_time;
        float start_distance=sum_distance;
        float arc_distance=TASK4_ARC_DISTANCE_M-TASK4_STOP_LINE_APPROACH_DISTANCE_M;
        float target_omega=TASK4_ARC_DIRECTION*TASK4_CRUISE_VEL_MPS/TASK4_ARC_RADIUS_M;
        bool arc_finished=(arc_distance<=0.0f);

        line_trace_exp_vel=TASK4_CRUISE_VEL_MPS;
        line_trace_exp_omega=0.0f;
        acc_feedforward=0.0f;
        enable_line_track=true;

        while(!arc_finished && !force_exit)
        {
            line_trace_exp_omega=Task4RampArcOmega(target_omega, arc_start_time, start_distance, arc_distance);
            arc_finished=(sum_distance-start_distance>=arc_distance);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK4_ARC_PERIOD_MS));


            if(sum_distance-start_distance>=(arc_distance-0.1f))    //提前屏蔽寻迹模块数据防止运行到A时机器人晃动
              ignore_line_sensor=true;
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)         //最末端停止部分
    {
        TickType_t last_wake_time=xTaskGetTickCount();
        float start_distance=sum_distance;
        float stop_decel=TASK4_CRUISE_VEL_MPS*TASK4_CRUISE_VEL_MPS/(2.0f*TASK4_FINAL_STOP_DISTANCE_M);
        bool stop_finished=false;

        line_trace_exp_omega=0.0f;
        acc_feedforward=-stop_decel;
        enable_line_track=true;

        while(!stop_finished && !force_exit)
        {
            float traveled_distance=sum_distance-start_distance;
            float vel_square=TASK4_CRUISE_VEL_MPS*TASK4_CRUISE_VEL_MPS-2.0f*stop_decel*traveled_distance;

            stop_finished=(traveled_distance>=TASK4_FINAL_STOP_DISTANCE_M);
            if(stop_finished || vel_square<=0.0f)
              line_trace_exp_vel=0.0f;
            else
              line_trace_exp_vel=sqrtf(vel_square);

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
        }

        line_trace_exp_omega=0.0f;
        line_trace_exp_vel=0.0f;
        acc_feedforward=0.0f;
    }


    ignore_line_sensor=false;
    enable_line_track=false;
    line_trace_exp_omega=0.0f;
    enable_ball_pos_control=false;
    car_is_stop=true;
    k230_cmd=0;
    robot_exp_vel=0.0f;
    robot_exp_omega=0.0f;
    current_task_id=0;
    task_running=false;
    force_exit=false;
    vTaskDelete(NULL);
    while(1){vTaskDelay(1000);}
}


void Task5(void* param)
{
    k230_cmd=1;
    task_running=true;
    
    line_trace_exp_omega=0.0f;
    car_is_stop=false;
    
    DL_GPIO_setPins(LED_RGB_PORT, LED_RGB_LED_R_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_G_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_B_PIN);

    while(DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))
    {
        if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_UP_PIN))
            ball_pos_offset+=0.00001f;
        else if(!DL_GPIO_readPins(KEY5_PORT, KEY5_K_DOWN_PIN))
            ball_pos_offset-=0.00001f;
        exp_ball_pos=ball_pos_offset;
      vTaskDelay(pdMS_TO_TICKS(50));    //等待直到按键按下，表示将当前钢球的位置设为期望它在运行时处于的位置
    }
    while(!DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))
    {
      vTaskDelay(pdMS_TO_TICKS(50));    //等待按键松开
    }

    DL_GPIO_clearPins(LED_RGB_PORT, LED_RGB_LED_R_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_G_PIN);
    DL_GPIO_setPins(LED_RGB_PORT,LED_RGB_LED_B_PIN);

    exp_ball_pos=ball_filter.position;
    enable_ball_pos_control=true;
    while(DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))    //等待直到按键按下，表示开始执行
    {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    while(!DL_GPIO_readPins(KEY3_PORT, KEY3_K3_PIN))    //等待按键松开
    {
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    DL_GPIO_clearPins(LED_RGB_PORT, LED_RGB_LED_R_PIN);
    DL_GPIO_setPins(LED_RGB_PORT,LED_RGB_LED_G_PIN);
    DL_GPIO_clearPins(LED_RGB_PORT,LED_RGB_LED_B_PIN);

    vTaskDelay(pdMS_TO_TICKS(300));
    
    float init_distance=sum_distance;

    {     //直线段到达第一个拐弯点
        TickType_t start_time;
        TickType_t last_wake_time;
        bool trajectory_finished=false;

        QuinticGenerate(&task3_quintic, sum_distance, 0.0f, sum_distance+1.6f,
                        TASK4_CRUISE_VEL_MPS, 8.3f);
        start_time=xTaskGetTickCount();
        last_wake_time=start_time;
        line_trace_exp_omega=0.0f;
        enable_line_track=true;

        while(!trajectory_finished && !force_exit)
        {
            float time=(xTaskGetTickCount()-start_time)*portTICK_PERIOD_MS*0.001f;

            trajectory_finished=QuinticSample(time, &task3_exp_pos, &task3_exp_vel, &task3_exp_acc, &task3_quintic);
            acc_feedforward=0.5f*mpu6050_accel_x+0.5f*task3_exp_acc;
            line_trace_exp_vel=task3_exp_vel+task3_pos_kp*(task3_exp_pos-sum_distance);   //求循迹速度

            if(sum_distance- init_distance<0.1f)
              ignore_line_sensor=true;
            else
              ignore_line_sensor=false;

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)   //走第一个圆弧
    {
        TickType_t arc_start_time=xTaskGetTickCount();
        TickType_t last_wake_time=arc_start_time;
        float start_distance=sum_distance;
        float target_omega=TASK4_ARC_DIRECTION*TASK4_CRUISE_VEL_MPS/TASK4_ARC_RADIUS_M;
        bool arc_finished=false;

        line_trace_exp_vel=TASK4_CRUISE_VEL_MPS;
        line_trace_exp_omega=0.0f;
        acc_feedforward=0.0f;
        enable_line_track=true;

        while(!arc_finished && !force_exit)
        {
            line_trace_exp_omega=Task4RampArcOmega(target_omega, arc_start_time, start_distance, TASK4_ARC_DISTANCE_M);
            arc_finished=(sum_distance-start_distance>=TASK4_ARC_DISTANCE_M);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK4_ARC_PERIOD_MS));
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)   //第二段直线
    {
        TickType_t start_time;
        TickType_t last_wake_time;
        bool trajectory_finished=false;

        QuinticGenerate(&task3_quintic, sum_distance, TASK4_CRUISE_VEL_MPS, sum_distance+1.5f,
                        TASK4_CRUISE_VEL_MPS,5.0f);
        start_time=xTaskGetTickCount();
        last_wake_time=start_time;
        line_trace_exp_omega=0.0f;
        enable_line_track=true;

        while(!trajectory_finished && !force_exit)
        {
            float time=(xTaskGetTickCount()-start_time)*portTICK_PERIOD_MS*0.001f;

            trajectory_finished=QuinticSample(time, &task3_exp_pos, &task3_exp_vel, &task3_exp_acc, &task3_quintic);
            acc_feedforward=0.5f*mpu6050_accel_x+0.5f*task3_exp_acc;
            line_trace_exp_vel=task3_exp_vel+task3_pos_kp*(task3_exp_pos-sum_distance);   //求循迹速度
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)       //第二段圆弧
    {
        TickType_t arc_start_time=xTaskGetTickCount();
        TickType_t last_wake_time=arc_start_time;
        float start_distance=sum_distance;
        float arc_distance=TASK4_ARC_DISTANCE_M-TASK4_STOP_LINE_APPROACH_DISTANCE_M;
        float target_omega=TASK4_ARC_DIRECTION*TASK4_CRUISE_VEL_MPS/TASK4_ARC_RADIUS_M;
        bool arc_finished=(arc_distance<=0.0f);

        line_trace_exp_vel=TASK4_CRUISE_VEL_MPS;
        line_trace_exp_omega=0.0f;
        acc_feedforward=0.0f;
        enable_line_track=true;

        while(!arc_finished && !force_exit)
        {
            line_trace_exp_omega=Task4RampArcOmega(target_omega, arc_start_time, start_distance, arc_distance);
            arc_finished=(sum_distance-start_distance>=arc_distance);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK4_ARC_PERIOD_MS));


            if(sum_distance-start_distance>=(arc_distance-0.1f))    //提前屏蔽寻迹模块数据防止运行到A时机器人晃动
              ignore_line_sensor=true;
        }

        line_trace_exp_omega=0.0f;
    }

    if(!force_exit)         //最末端停止部分
    {
        TickType_t last_wake_time=xTaskGetTickCount();
        float start_distance=sum_distance;
        float stop_decel=TASK4_CRUISE_VEL_MPS*TASK4_CRUISE_VEL_MPS/(2.0f*TASK4_FINAL_STOP_DISTANCE_M);
        bool stop_finished=false;

        line_trace_exp_omega=0.0f;
        acc_feedforward=-stop_decel;
        enable_line_track=true;

        while(!stop_finished && !force_exit)
        {
            float traveled_distance=sum_distance-start_distance;
            float vel_square=TASK4_CRUISE_VEL_MPS*TASK4_CRUISE_VEL_MPS-2.0f*stop_decel*traveled_distance;

            stop_finished=(traveled_distance>=TASK4_FINAL_STOP_DISTANCE_M);
            if(stop_finished || vel_square<=0.0f)
              line_trace_exp_vel=0.0f;
            else
              line_trace_exp_vel=sqrtf(vel_square);

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
        }

        line_trace_exp_omega=0.0f;
        line_trace_exp_vel=0.0f;
        acc_feedforward=0.0f;
    }


    ignore_line_sensor=false;
    enable_line_track=false;
    line_trace_exp_omega=0.0f;
    enable_ball_pos_control=false;
    car_is_stop=true;
    k230_cmd=0;
    robot_exp_vel=0.0f;
    robot_exp_omega=0.0f;
    current_task_id=0;
    task_running=false;
    force_exit=false;
    vTaskDelete(NULL);
    while(1){vTaskDelay(1000);}
}


float test_ball_exp_pos=0.0f;
void TestTask(void* param)
{
    vTaskDelay(2000);
    enable_ball_pos_control=false;
    enable_line_track=false;
    car_is_stop=true;
    k230_cmd=1;
    while(1)
    {
        
        
        //exp_ball_pos=test_ball_exp_pos;
        vTaskDelay(50);
    }
}
