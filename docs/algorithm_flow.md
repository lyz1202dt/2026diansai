# APP 算法逻辑与数据处理流程

本文档基于 `APP/app_main.c` 和 `APP/mytask.c` 整理，描述当前应用层的任务组织、主要算法逻辑和数据处理路径。

## 1. 总体结构

`app_main()` 是应用入口，完成系统配置、FreeRTOS 任务创建、OLED 初始化、按键选题和任务启动。`mytask.c` 承载具体控制算法，包括 IMU 读取、轮速闭环、里程计、灰度循迹、ZDT 电机控制、K230 通讯解析和赛题任务。

```mermaid
flowchart TD
    A[app_main] --> B[SetupConfig]
    A --> C[创建 WheelTask]
    A --> D[创建 IMUTask]
    A --> E[创建 LineTrack]
    A --> F[创建 ZDTDriver]
    A --> G[OLED_Init]
    A --> H[按键扫描与题号选择]
    H --> I{current_task_id == 1<br/>且 task_running == false}
    I -- 是 --> J[创建 Task1]
    I -- 否 --> K[继续主循环]
    A --> L[发送 k230_cmd 到 K230]
    L --> K
```

主循环每 50 ms 执行一次：

- K1：清零当前任务号 `current_task_id`。
- K2：确认所选任务，将 `current_select_task_id` 写入 `current_task_id`。
- 上、下、左、右、中键：分别选择任务 1 到任务 5。
- 当前仅实现 `current_task_id == 1` 时创建 `Task1`。
- 每轮将 `k230_cmd` 置 0 后通过 `g_serial` 发送给 K230。

## 2. FreeRTOS 任务分工

```mermaid
flowchart LR
    subgraph Sensor[传感与估计]
        IMU[IMUTask<br/>5 ms] --> Yaw[mpu6050_yaw<br/>mpu6050_yaw_rate_dps]
        Line[LineTrack<br/>40 ms] --> Track[line_track_vel<br/>line_track_omega]
        K230[K230RecvTask] --> Ball[filter.position]
    end

    subgraph Control[控制]
        Wheel[WheelTask<br/>5 ms]
        ZDT[ZDTDriver<br/>4 ms]
        T1[Task1]
    end

    Yaw --> Wheel
    Track --> Wheel
    Ball --> ZDT
    T1 -->|enable_line_track/kExpTrackVel| Line
    Wheel --> Motor[L298N 双轮电机]
    ZDT --> Joint[ZDT 关节电机]
```


| 任务           | 周期     | 主要输入                                    | 主要输出                                                 | 功能                                 |
| -------------- | -------- | ------------------------------------------- | -------------------------------------------------------- | ------------------------------------ |
| `IMUTask`      | 5 ms     | MPU6050 四元数、陀螺仪                      | `mpu6050_yaw`、`mpu6050_yaw_rate_dps`、`mpu6050_success` | 姿态角和角速度更新                   |
| `WheelTask`    | 5 ms     | 编码器、IMU yaw、底盘速度指令               | L298N PWM、里程计、VOFA 数据                             | 轮速滤波、方向闭环、速度 PID、里程计 |
| `LineTrack`    | 40 ms    | 8 路灰度 ADC                                | `robot_exp_vel`、`robot_exp_omega`                       | 灰度阈值自适应、循迹角速度计算       |
| `ZDTDriver`    | 4 ms     | ZDT 当前位置、Kalman 小球位置、IMU yaw rate | ZDT 速度指令                                             | 小球位置闭环和关节位置闭环           |
| `K230RecvTask` | 阻塞接收 | K230 串口帧                                 | Kalman 观测更新                                          | 帧同步、校验、位置解析               |
| `Task1`        | 流程任务 | `sum_distance`、灰度状态                    | `enable_line_track`、OLED 结果                           | 第一题执行流程                       |

> 注：`K230RecvTask` 在 `mytask.c` 中实现并在头文件声明，但当前 `app_main.c` 未创建该任务。

## 3. IMU 数据处理

`IMUTask()` 初始化 MPU6050 后循环读取 IMU 数据。

```mermaid
flowchart TD
    A[MPU6050_Init] --> B[read_imu]
    B --> C{读取成功}
    C -- 是 --> D[get_euler_angles<br/>提取 yaw]
    D --> E[mpu6050_yaw = yaw]
    D --> F[mpu6050_yaw_rate_dps = gyro_z]
    C -- 否 --> G[mpu6050_success = false]
    E --> H[延时到 5 ms 周期]
    F --> H
    G --> H
    H --> B
```

关键数据：

- `mpu6050_yaw`：当前航向角，单位为度。
- `mpu6050_yaw_rate_dps`：Z 轴角速度，单位为 deg/s。
- `mpu6050_success`：IMU 数据是否有效。

## 4. 底盘轮速闭环与里程计

`WheelTask()` 是底盘控制核心。它根据编码器差分得到轮速，经过一阶低通滤波后，与期望轮速做 PID 控制，最后输出到 L298N。

```mermaid
flowchart TD
    A[读取编码器 g_encoder1/g_encoder2] --> B[差分计算 m1_omega/m2_omega]
    B --> C[一阶滤波得到 m1_cur_omega/m2_cur_omega]
    A --> D[累计 sum_distance]
    C --> E[由 robot_exp_vel/robot_exp_omega<br/>换算左右轮期望角速度]
    E --> F{enable_dir_control<br/>且 IMU 有效}
    F -- 是 --> G[计算 yaw_error<br/>修正左右轮期望速度]
    F -- 否 --> H[直接使用期望轮速]
    G --> I[PID_Control]
    H --> I
    I --> J[摩擦和死区补偿]
    J --> K[L298N_SetPWMValue]
    C --> L{enabl_odometer}
    L -- 是 --> M[IMU yaw 修正航向]
    M --> N[积分更新 cur_robot_pos_x/y]
```

轮速计算：

```text
wheel_omega = 编码器增量 / 编码器线数 * 2π / 减速比 / 采样周期
```

底盘运动学：

```text
m1_exp_omega =  robot_exp_vel / R + robot_exp_omega * body_radius / R
m2_exp_omega = -robot_exp_vel / R + robot_exp_omega * body_radius / R
```

里程计更新：

```text
linear_vel = (m1_cur_omega - m2_cur_omega) * R / 2
vx = linear_vel * cos(yaw)
vy = linear_vel * sin(yaw)
pos += v * dt
```

其中 `R = 0.033 m`，控制周期 `dt = 0.005 s`。

## 5. 灰度循迹数据处理

`LineTrack()` 使用 8 路灰度传感器识别黑线，并根据黑线落在阵列中的位置生成转向角速度。

初始化阶段：

1. 启动后延时 3 s。
2. 以 50 ms 周期采集 40 次环境光。
3. 根据 8 路 ADC 的最小值和最大值计算阈值候选值。
4. 当亮暗差异足够大时，用一阶滤波更新 `is_line_gate`。

运行阶段：

```mermaid
flowchart TD
    A[GWGetState 读取 8 路 ADC] --> B{读取成功}
    B -- 否 --> C[line_track_vel = 0<br/>line_track_omega = 0]
    B -- 是 --> D[逐路判断 value < is_line_gate]
    D --> E{检测到黑线}
    E -- 是 --> F[按权重累加 detected_omega]
    F --> G[line_track_omega<br/>= 0.3 * detected + 0.7 * old]
    E -- 否 --> H[按上次转向方向<br/>保持 ±1.0 搜线]
    G --> I[line_track_vel = kExpTrackVel]
    H --> I
    I --> J{enable_line_track}
    C --> J
    J -- 是 --> K[关闭方向闭环<br/>robot_exp_vel = line_track_vel<br/>robot_exp_omega = line_track_omega]
    J -- 否 --> L[robot_exp_vel = 0<br/>robot_exp_omega = 0]
```

8 路灰度权重从左到右为：

```c
{-1.2f, -0.7f, -0.3f, -0.1f, 0.1f, 0.3f, 0.7f, 1.2f}
```

因此左侧传感器压线时输出负角速度，右侧传感器压线时输出正角速度。实际转向方向还取决于底盘电机安装方向和运动学符号定义。

## 6. K230 通讯与小球位置滤波

K230 接收帧格式由 `RecvPack` 定义：

```c
typedef struct {
    uint8_t head;     // 0x5A
    float position;
    uint8_t check;    // 和校验
} RecvPack;
```

接收任务逻辑：

```mermaid
flowchart TD
    A[SerialReceive 单字节读取] --> B{是否超时}
    B -- 是 --> C[timeout 计数 +1]
    B -- 否 --> D{帧头位置是否为 0x5A}
    D -- 否 --> E[header error 计数 +1]
    D -- 是 --> F[写入接收缓冲区]
    F --> G{是否收满 RecvPack}
    G -- 否 --> A
    G -- 是 --> H{帧头与校验有效}
    H -- 是 --> I[k230_pack_parse]
    I --> J[valid 计数 +1<br/>recv_index 清零]
    H -- 否 --> K[check error 计数 +1<br/>搜索下一个帧头重同步]
    C --> A
    E --> A
    J --> A
    K --> A
```

`k230_pack_parse()` 数据处理：

1. 将字节流复制为 `RecvPack`。
2. 使用 `PIXEL2POSITION(x)` 将视觉像素位置转换为物理位置。
3. 由当前关节角 `joint_cur_pos` 反解棍子角度 `stick_cur_angle`。
4. 用 `sin(stick_cur_angle) * 9.8` 估算小球沿杆方向加速度。
5. 根据两次视觉更新间隔计算 `dt`，并限制最大值。
6. 调用 `Kalman1D_Update(&filter, raw_position, raw_acc, dt)` 融合视觉位置和模型加速度。

## 7. ZDT 关节电机与钢球位置闭环

`ZDTDriver()` 负责读取 ZDT 关节电机当前位置，并根据小球位置控制需求输出电机速度。

```mermaid
flowchart TD
    A[Kalman1D_Init] --> B[MakeZDTSerialEnv]
    B --> C[读取 ZDT 当前角度 joint_cur_pos]
    C --> D{car_is_stop == false}
    D -- 是 --> E[根据小球位置和车体 yaw rate<br/>计算自旋前馈角 car_rotation_feedforward]
    D -- 否 --> F[前馈保持原值]
    E --> G{enable_ball_pos_control}
    F --> G
    G -- 是 --> H[ball_pos_pid:<br/>filter.position -> exp_ball_pos]
    H --> I[棍子期望角 + 前馈<br/>转换为 motor_exp_pos]
    G -- 否 --> J[motor_exp_pos = 中性角]
    I --> K[motor_pos_pid:<br/>joint_cur_pos -> motor_exp_pos]
    J --> K
    K --> L[SetMotorVel<br/>输出 ZDT 速度]
    L --> C
```

两级控制关系：

- 外环：小球位置环
  `filter.position` 与 `exp_ball_pos` 做 PID，输出期望棍子角度。
- 内环：关节位置环
  `joint_cur_pos` 与 `motor_exp_pos` 做 PID，输出期望电机速度补偿。
- 前馈：当车体旋转时，根据向心加速度估算棍子补偿角。

机构角度换算：

- `stick_angle_to_motor_angle()`：由棍子角度计算电机角度。
- `motor_angle_to_stick_angle()`：由电机角度反解棍子角度。

## 8. Task1 赛题流程

`Task1()` 是当前唯一由 `app_main()` 自动创建的赛题任务。

```mermaid
flowchart TD
    A[Task1 启动] --> B[延时 500 ms]
    B --> C[kExpTrackVel = 0.2]
    C --> D[记录开始时间和里程偏置]
    D --> E[enable_line_track = true]
    E --> F[高速循迹阶段<br/>等待距离条件]
    F --> G[kExpTrackVel = 0.1]
    G --> H[低速循迹<br/>等待中间 4 路灰度同时压线]
    H --> I[enable_line_track = false]
    I --> J[OLED 显示耗时]
    J --> K[task_running = false]
    K --> L[vTaskDelete 自删除]
```

任务意图可以概括为：

1. 启动循迹。
2. 先用较高速度接近终点区域。
3. 再降速，直到灰度阵列中间 4 路同时检测到黑线，认为到达停止线。
4. 停止循迹并显示耗时。

## 9. 关键全局数据流

```mermaid
flowchart LR
    Encoder[编码器] --> WheelTask[WheelTask]
    IMU[MPU6050] --> IMUTask[IMUTask]
    Gray[8 路灰度] --> LineTrack[LineTrack]
    K230[K230 视觉] --> K230Recv[K230RecvTask]
    ZDTMotor[ZDT 关节电机反馈] --> ZDTDriver[ZDTDriver]

    IMUTask -->|mpu6050_yaw/yaw_rate| WheelTask
    IMUTask -->|yaw_rate| ZDTDriver
    LineTrack -->|robot_exp_vel/omega| WheelTask
    WheelTask -->|PWM| L298N[L298N 电机驱动]
    WheelTask -->|cur_robot_pos/yaw/sum_distance| Task1[Task1]
    K230Recv -->|filter.position| ZDTDriver
    ZDTDriver -->|速度控制帧| ZDTMotor
    Task1 -->|enable_line_track/kExpTrackVel| LineTrack
```
