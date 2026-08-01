# 小球平衡控制算法流程

本文档基于 `APP/app_main.c` 和 `APP/mytask.c` 整理，描述项目从应用入口启动后的整体流程，以及小球平衡控制的数据流、闭环结构和任务时序。

## 1. 项目整体流程

`app_main()` 是应用层入口，负责完成系统初始化、创建常驻控制任务、处理按键交互、启动赛题任务，并周期性向视觉模块发送当前工作指令。

```mermaid
flowchart TD
    A["应用入口"] --> B["系统硬件初始化"]
    B --> C["显示屏初始化"]
    C --> D["创建常驻任务"]
    D --> E["进入主循环"]

    E --> F["读取按键"]
    F --> G{按键类型}
    G -->|清除| H["停止当前任务"]
    G -->|确认| I["确认所选赛题"]
    G -->|平衡测试| J["开启钢球平衡测试"]
    G -->|方向键| K["选择赛题编号"]
    G -->|无操作| L["保持当前状态"]

    H --> M["刷新显示信息"]
    I --> M
    J --> N["调整视觉位置偏移"]
    K --> M
    L --> M
    N --> M

    M --> O{是否需要启动赛题}
    O -->|是| P["创建对应赛题任务"]
    O -->|否| Q["继续等待"]
    P --> R["发送视觉工作指令"]
    Q --> R
    R --> S["固定周期延时"]
    S --> E
```

常驻任务创建后并行运行：

```mermaid
flowchart LR
    A["应用入口"] --> B["轮组驱动任务"]
    A --> C["惯性测量任务"]
    A --> D["循迹处理任务"]
    A --> E["关节电机任务"]
    A --> F["视觉接收任务"]

    B --> G["底盘速度控制"]
    C --> H["姿态与加速度估计"]
    D --> I["路线跟踪"]
    E --> J["钢球平衡执行"]
    F --> K["钢球位置更新"]
```

整体执行关系：

- 启动阶段先完成硬件与显示初始化，再创建轮组、惯性测量、循迹、关节电机、视觉接收等常驻任务。
- 主循环负责用户交互和赛题调度，不直接执行复杂控制算法。
- 赛题任务启动后，会按对应流程打开视觉识别、钢球平衡、路线跟踪或运动轨迹。
- 视觉接收、惯性测量和关节电机任务共同构成钢球平衡的实时闭环。

## 2. 小球平衡总体闭环

小球平衡控制由视觉位置闭环、关节电机执行闭环和 IMU 前馈补偿共同组成。

```mermaid
flowchart TD
    A["K230 初始化<br/>(摄像头 / YOLO 模型 / 串口)"]
    B["YOLO 检测钢球<br/>(确定初始位置)"]
    C["逐列反差跟踪<br/>(在上帧位置附近跟踪球心)"]
    D["输出球心像素坐标"]

    A --> B
    B --> C
    C -->|跟踪到| D
    C -->|未跟踪到| B

    D --> E["K230RecvTask<br/>串口收帧/校验/重同步"]
    E --> F["k230_pack_parse<br/>像素位置换算为物理位置"]
    F --> G["ball_filter<br/>Kalman1D 估计小球位置/速度"]
    G --> H{enable_ball_pos_control}
    H -- 是 --> I["外环 ball_pos_pid<br/>位置误差 -> 期望速度"]
    I --> J["内环 ball_vel_pid<br/>速度误差 -> 摆杆角度修正量"]
    J --> K["stick_angle_to_motor_angle<br/>摆杆角度 -> 电机期望角度 motor_exp_pos"]
    H -- 否 --> L["ZDTDriver 将 motor_exp_pos<br/>回到空闲角度"]

    M["IMUTask<br/>yaw/yaw_rate/acc_x"] --> N["ZDTDriver<br/>计算 stick_angle_feedforward"]
    N --> K

    K --> O["ZDTDriver<br/>期望角限速/滤波/限幅"]
    L --> O
    O --> P["Emm_V5_Pos_ControlEx<br/>驱动 ZDT 关节电机"]
    P --> Q["摆杆/托盘倾角变化"]
    Q --> R["小球受力改变"]
    R --> C
```

核心思路：

- K230 先用 YOLO 确定钢球初始位置，后续优先在上帧位置附近做逐列反差跟踪；跟踪失败时回到 YOLO 检测。
- `K230RecvTask` 接收 K230 输出的球心像素坐标，并转换为 `ball_filter.position` 和 `ball_filter.velocity`。
- `k230_pack_parse()` 在视觉数据到来时执行小球位置双环 PID，更新 `motor_exp_pos`。
- `ZDTDriver` 以 4 ms 周期读取关节角，并把 `motor_exp_pos` 平滑后下发给 ZDT 电机。
- `IMUTask` 提供 `mpu6050_yaw_rate_dps` 和 `mpu6050_accel_x`，用于自旋和加速度前馈补偿。

## 3. 任务职责

| 任务 | 周期/触发 | 主要输入 | 主要输出 | 在小球平衡中的作用 |
| --- | --- | --- | --- | --- |
| `IMUTask` | 初始化阶段 5 ms，运行阶段 20 ms | MPU6050 四元数、陀螺仪、加速度 | `mpu6050_yaw`、`mpu6050_yaw_rate_dps`、`mpu6050_accel_x`、`mpu6050_success` | 提供自旋角速度和前向加速度补偿量 |
| `K230RecvTask` | 串口阻塞单字节接收 | K230 `RecvPack` 帧 | `raw_position`、`ball_filter`、`motor_exp_pos` | 解析视觉位置并执行小球位置/速度双闭环 |
| `ZDTDriver` | 4 ms | `motor_exp_pos`、`ball_filter.position`、IMU yaw rate、ZDT 当前角 | `filtered_motor_exp_pos`、ZDT 位置控制命令 | 执行关节电机控制，并叠加前馈与输出保护 |

## 4. IMUTask：姿态与加速度估计

```mermaid
flowchart TD
    A[MPU6050_Init] --> B[采集 100 次 IMU 数据]
    B --> C{read_imu 成功?}
    C -- 是 --> D[更新 yaw/yaw_rate]
    D --> E[累加 acc_x_offset<br/>用于加速度零偏]
    C -- 否 --> B
    E --> F{累计满 100 次?}
    F -- 否 --> B
    F -- 是 --> G[Kalman1D_Init acc_x_filter]
    G --> H[周期读取 IMU]
    H --> I{read_imu 成功?}
    I -- 是 --> J[四元数转 yaw<br/>gyro_z -> yaw_rate]
    J --> K[acc_x_raw = 9.8 * accel_x - offset]
    K --> L[Kalman1D_Update<br/>滤波前向加速度]
    L --> M[补偿自旋离心项<br/>mpu6050_accel_x = acc_filter.position - yaw_rate^2 * r]
    I -- 否 --> N[保持上次有效估计]
    M --> H
    N --> H
```

关键公式：

```text
acc_x_raw = 9.8 * mpu6050_accel_sample_g[0] - acc_x_offset
mpu6050_accel_x = acc_x_filter.position
                 - mpu6050_yaw_rate_dps^2 * DEG_TO_RAD^2 * 0.14
```

`mpu6050_accel_x` 后续可与轨迹期望加速度融合，形成 `acc_feedforward`。`mpu6050_yaw_rate_dps` 会被 `ZDTDriver` 用于小球自旋补偿。

## 5. K230RecvTask：视觉接收与小球双环 PID

K230 接收帧格式：

```c
typedef struct {
    uint8_t head;     // 0x5A
    float position;  // 视觉位置
    uint8_t check;   // 和校验
} RecvPack;
```

串口收帧流程：

```mermaid
flowchart TD
    A[SerialReceive 读取 1 字节<br/>timeout 10 ticks] --> B{是否收到 1 字节?}
    B -- 否 --> C[k230_comm_timeout_cnt++]
    C --> A
    B -- 是 --> D{recv_index == 0<br/>且 byte != 0x5A?}
    D -- 是 --> E[k230_comm_header_err_cnt++]
    E --> A
    D -- 否 --> F[写入 k230_comm_recv_buffer]
    F --> G{是否收满 sizeof RecvPack?}
    G -- 否 --> A
    G -- 是 --> H{帧头和校验有效?}
    H -- 是 --> I[k230_pack_parse]
    I --> J[k230_comm_valid_cnt++<br/>recv_index = 0]
    H -- 否 --> K[k230_comm_check_err_cnt++]
    K --> L[K230CommResync<br/>在缓冲区内寻找下一个 0x5A]
    L --> A
    J --> A
```

位置解析与控制流程：

```mermaid
flowchart TD
    A[k230_pack_parse] --> B[memcpy 得到 RecvPack]
    B --> C[raw_position = PIXEL2POSITION position]
    C --> D[计算视觉更新周期 dt]
    D --> E{dt > 0.08 s?}
    E -- 是 --> F[dt 限制为 0.08 s]
    E -- 否 --> G[使用原始 dt]
    F --> H{raw_position 是否在 -0.3 到 0.3 m?}
    G --> H
    H -- 是 --> I[Kalman1D_Update ball_filter<br/>更新位置和速度]
    H -- 否 --> J[丢弃异常视觉值]
    I --> K{enable_ball_pos_control?}
    J --> K
    K -- 是 --> L[ball_pos_pid<br/>current = ball_filter.position<br/>expected = exp_ball_pos]
    L --> M[ball_vel_pid<br/>current = ball_filter.velocity<br/>expected = ball_pos_pid.pid_out]
    M --> N[motor_exp_pos = stick_angle_to_motor_angle<br/>stick_angle_feedforward + ball_vel_pid.pid_out]
    K -- 否 --> O[不更新小球闭环输出]
```

控制结构为串级 PID：

```text
位置外环：exp_ball_pos - ball_filter.position -> ball_pos_pid.pid_out
速度内环：ball_pos_pid.pid_out - ball_filter.velocity -> ball_vel_pid.pid_out
摆杆期望角：stick_angle_feedforward + ball_vel_pid.pid_out
电机期望角：stick_angle_to_motor_angle(摆杆期望角)
```

当前参数：

```c
PID ball_pos_pid = {.Kp = 2.5f, .Kd = 0.0f, .Ki = 0.0f,
                    .limit = 5.0f, .output_limit = 0.09f};
PID ball_vel_pid = {.Kp = 30.0f, .Kd = 0.0f, .Ki = 1.0f,
                    .limit = 10.0f, .output_limit = 6.0f};
```

## 6. ZDTDriver：关节电机执行与前馈补偿

```mermaid
flowchart TD
    A[Kalman1D_Init ball_filter] --> B[MakeZDTSerialEnv]
    B --> C[filtered_motor_exp_pos 初始化为空闲角]
    C --> D[4 ms 周期循环]
    D --> E[读取 ZDT 当前角度<br/>joint_cur_pos = -angle_temp]
    E --> F{car_is_stop == false?}
    F -- 是 --> G[根据球位置和 yaw_rate 计算自旋/加速度前馈]
    G --> H[forward_acc = BALL_POS_TO_CENTER_DIS(position) * yaw_rate^2 - acc_feedforward]
    H --> I[gravity_ratio = clamp(forward_acc / 9.8, -1, 1)]
    I --> J[stick_angle_feedforward = asin(gravity_ratio)]
    F -- 否 --> K[保持当前前馈]
    J --> L{enable_ball_pos_control?}
    K --> L
    L -- 否 --> M[motor_exp_pos = 空闲角]
    L -- 是 --> N[使用 K230RecvTask 更新的 motor_exp_pos]
    M --> O[limit_motor_exp_pos_step<br/>限制期望角单步变化]
    N --> O
    O --> P[一阶滤波<br/>filtered = 0.6 old + 0.4 limited]
    P --> Q[安全限幅 0 到 50 deg]
    Q --> R[Emm_V5_Pos_ControlEx<br/>目标=-filtered_motor_exp_pos<br/>反馈=-joint_cur_pos]
    R --> D
```

前馈用于抵消加速运动和平台自旋对小球的等效加速度：

```text
yaw_rate_rad_s = mpu6050_yaw_rate_dps * DEG_TO_RAD
forward_acc = BALL_POS_TO_CENTER_DIS(ball_filter.position) * yaw_rate_rad_s^2
              - acc_feedforward
stick_angle_feedforward = asin(clamp(forward_acc / 9.8, -1, 1))
```

输出保护包括三层：

- `limit_motor_exp_pos_step()`：限制 `motor_exp_pos` 到 `filtered_motor_exp_pos` 的单周期变化，默认 `max_motor_exp_step = 1.0f`。
- `ball_pid_output_filter_gate = 0.6f`：对电机期望角做一阶低通。
- `filtered_motor_exp_pos` 限制在 `0.0f` 到 `50.0f`，避免异常数据损坏电机。

## 7. 控制时序

```mermaid
sequenceDiagram
    participant IMU as IMUTask
    participant K230 as K230RecvTask
    participant ZDT as ZDTDriver
    participant Task as 赛题/按键任务

    IMU->>IMU: 20 ms 更新 yaw_rate 和 acc_x
    Task->>ZDT: 更新 exp_ball_pos / enable_ball_pos_control / acc_feedforward
    K230->>K230: 收满有效视觉帧
    K230->>K230: 更新 ball_filter
    K230->>ZDT: 写 motor_exp_pos
    ZDT->>ZDT: 4 ms 读取关节角
    IMU->>ZDT: 提供 yaw_rate
    ZDT->>ZDT: 计算 stick_angle_feedforward
    ZDT->>ZDT: 滤波/限幅 motor_exp_pos
    ZDT->>ZDT: 下发 ZDT 位置控制命令
```

需要注意的是，小球双环 PID 在 `K230RecvTask` 收到有效视觉帧时更新，不是固定 4 ms 更新；`ZDTDriver` 则以固定 4 ms 周期不断执行最近一次得到的 `motor_exp_pos`。

## 8. 关键变量关系

```mermaid
flowchart LR
    raw[raw_position] --> bf[ball_filter.position / velocity]
    exp[exp_ball_pos] --> pospid[ball_pos_pid]
    bf --> pospid
    pospid --> velpid[ball_vel_pid]
    bf --> velpid
    imu[mpu6050_yaw_rate_dps] --> ff[stick_angle_feedforward]
    acc[acc_feedforward] --> ff
    bf --> ff
    ff --> angle[stick_angle_feedforward + ball_vel_pid.pid_out]
    velpid --> angle
    angle --> motor[motor_exp_pos]
    motor --> filt[filtered_motor_exp_pos]
    filt --> zdt[ZDT 电机]
```

| 变量 | 来源 | 含义 |
| --- | --- | --- |
| `raw_position` | `K230RecvTask` | K230 原始位置换算后的物理位置，单位 m |
| `ball_filter.position` | `Kalman1D_Update` | 小球滤波位置 |
| `ball_filter.velocity` | `Kalman1D_Update` | 小球滤波速度 |
| `exp_ball_pos` | 赛题任务/测试按键 | 小球目标位置 |
| `stick_angle_feedforward` | `ZDTDriver` | 自旋和运动加速度对应的摆杆前馈角 |
| `motor_exp_pos` | `k230_pack_parse` 或 `ZDTDriver` | ZDT 电机目标角度 |
| `filtered_motor_exp_pos` | `ZDTDriver` | 限速、滤波、限幅后的电机目标角 |
| `mpu6050_accel_x` | `IMUTask` | 补偿自旋后的前向加速度估计 |
| `acc_feedforward` | 赛题任务 | 轨迹运动加速度前馈 |

## 9. 平衡控制链路小结

当前算法可概括为：

```text
视觉测球位置
-> Kalman 估计球位置/速度
-> 位置 PID 生成球期望速度
-> 速度 PID 生成摆杆角修正量
-> IMU/运动加速度生成摆杆前馈角
-> 摆杆角映射为 ZDT 电机角
-> 限速、低通、限幅
-> ZDT 关节电机改变托盘倾角
-> 小球位置变化后再次被视觉测量
```

其中 `IMUTask` 负责估计运动引起的惯性补偿量，`K230RecvTask` 负责视觉闭环更新，`ZDTDriver` 负责将闭环结果稳定地下发到关节电机。
