# APP 程序整体流程与数据流向

本文档根据当前 `APP/app_main.c` 与 `APP/mytask.c` 整理，描述应用层整体调度、后台控制任务、Task1~Task5 赛题流程，以及关键数据在各任务之间的流向。`docs/algorithm_flow.md` 为旧版本参考，本文以当前代码为准。

## 1. 整体程序流程

`app_main()` 完成硬件与任务初始化，然后进入 50 ms UI 主循环。后台任务常驻运行；赛题任务由按键选择并确认后按需创建，同一时刻只允许一个赛题任务运行。

```mermaid
flowchart TD
    A[app_main] --> B[SetupConfig<br/>电机/编码器/串口/中断初始化]
    B --> C[OLED_Init]
    C --> D[创建常驻 FreeRTOS 任务]
    D --> D1[WheelTask<br/>5 ms]
    D --> D2[IMUTask<br/>5 ms]
    D --> D3[LineTrack<br/>40 ms]
    D --> D4[ZDTDriver<br/>4 ms]
    D --> D5[K230RecvTask<br/>串口阻塞接收]
    D --> E[UI 主循环<br/>每 50 ms]

    E --> F{按键扫描}
    F -->|K1 清零| G[current_task_id = 0<br/>force_exit = true]
    F -->|K2 确认| H[current_task_id = current_select_task_id]
    F -->|UP/DOWN/LEFT/RIGHT/MAIN| I[选择 Task1~Task5]
    F -->|K3| J[球平衡测试模式<br/>enable_ball_pos_control = true]

    H --> K{task_running == false?}
    I --> E
    J --> E
    G --> E
    K -->|Task1| T1[xTaskCreate Task1]
    K -->|Task2| T2[xTaskCreate Task2]
    K -->|Task3| T3[xTaskCreate Task3]
    K -->|Task4| T4[xTaskCreate Task4]
    K -->|Task5| T5[xTaskCreate Task5]
    K -->|无任务/任务运行中| L[继续刷新 OLED]
    T1 --> L
    T2 --> L
    T3 --> L
    T4 --> L
    T5 --> L
    L --> M[SerialTransmit(g_serial, k230_cmd)<br/>向 K230 更新视觉工作状态]
    M --> E
```

### 按键与全局状态

| 按键 | 当前作用 | 主要影响 |
| --- | --- | --- |
| `KEY3_K1` | 任务清零/强制退出 | `current_task_id = 0`，`force_exit = true` |
| `KEY3_K2` | 确认选题 | `current_task_id = current_select_task_id` |
| `KEY3_K3` | 球平衡测试或赛题内启动确认 | 主循环中进入 `ball_test`；Task3/4/5 中作为启动确认键 |
| `KEY5_UP` | 选择 Task1 | `current_select_task_id = 1` |
| `KEY5_DOWN` | 选择 Task2 | `current_select_task_id = 2` |
| `KEY5_LEFT` | 选择 Task3 | `current_select_task_id = 3` |
| `KEY5_RIGHT` | 选择 Task4 | `current_select_task_id = 4` |
| `KEY5_MAIN` | 选择 Task5 | `current_select_task_id = 5` |

## 2. 常驻任务分工

```mermaid
flowchart LR
    subgraph Sensor[传感与接收]
        IMU[IMUTask] -->|mpu6050_yaw<br/>mpu6050_yaw_rate_dps<br/>mpu6050_success| State[全局状态]
        ENC[编码器中断累计<br/>g_encoder1/2] --> Wheel[WheelTask]
        LINE[灰度模块<br/>GWGetState] --> Track[LineTrack]
        K230[K230 视觉串口帧] --> Recv[K230RecvTask]
        ZPOS[ZDT 当前位置回读] --> ZDT[ZDTDriver]
    end

    subgraph Estimate[估计与中间量]
        Recv -->|raw_position| Kalman[Kalman1D<br/>ball_filter.position/velocity]
        Wheel -->|sum_distance<br/>cur_robot_pos_x/y<br/>cur_robot_yaw| Odom[里程计]
        Track -->|line_trace_result| LineState[黑线检测状态]
    end

    subgraph Command[赛题任务写入指令]
        Tasks[Task1~Task5] -->|enable_line_track<br/>line_trace_exp_vel<br/>line_trace_exp_omega<br/>ignore_line_sensor| Track
        Tasks -->|k230_cmd| K230Cmd[K230 工作状态]
        Tasks -->|enable_ball_pos_control<br/>exp_ball_pos<br/>car_is_stop<br/>acc_feedforward| BallCmd[钢球控制状态]
    end

    Track -->|robot_exp_vel<br/>robot_exp_omega| Wheel
    State --> Wheel
    Wheel -->|L298N PWM| Chassis[双轮底盘]
    Kalman --> Recv
    Kalman --> ZDT
    BallCmd --> Recv
    BallCmd --> ZDT
    ZDT -->|Emm_V5_Pos_ControlEx| Joint[ZDT 关节电机]
    K230Cmd -->|app_main 周期发送| K230
```

| 任务 | 周期/触发 | 输入 | 输出 | 作用 |
| --- | --- | --- | --- | --- |
| `IMUTask` | 5 ms | MPU6050 四元数、陀螺仪 | `mpu6050_yaw`、`mpu6050_yaw_rate_dps`、`mpu6050_success` | 姿态读取与 yaw 更新 |
| `WheelTask` | 5 ms | 编码器、IMU yaw、`robot_exp_vel/omega` | 电机 PWM、`sum_distance`、`cur_robot_pos_x/y/yaw` | 轮速闭环、方向闭环、里程计 |
| `LineTrack` | 40 ms | 灰度状态、循迹期望速度/附加角速度 | `robot_exp_vel`、`robot_exp_omega`、`line_trace_result` | 灰度循迹和底盘速度指令生成 |
| `ZDTDriver` | 4 ms | ZDT 位置、钢球控制目标、IMU yaw rate | `motor_exp_pos` 位置控制命令 | 关节电机位置控制与前馈计算 |
| `K230RecvTask` | 串口阻塞接收 | K230 `RecvPack` | `ball_filter`、`raw_position`、`motor_exp_pos` | 视觉帧解析、钢球卡尔曼滤波、球控外环 |

## 3. 底盘控制链路

```mermaid
flowchart TD
    A[LineTrack 或赛题任务给出<br/>robot_exp_vel / robot_exp_omega] --> B[WheelTask]
    C[编码器累计值] --> B
    D[IMU yaw] --> B
    B --> E[编码器差分计算 m1/m2 当前角速度]
    E --> F[一阶低通滤波<br/>m1_cur_omega / m2_cur_omega]
    A --> G[底盘运动学换算<br/>左右轮期望角速度]
    D --> H{enable_dir_control<br/>且 IMU 有效}
    H -->|是| I[yaw_error 修正左右轮期望]
    H -->|否| J[直接使用期望轮速]
    I --> K[wheel1/2_vel_pid]
    J --> K
    K --> L[摩擦/死区补偿]
    L --> M[L298N_SetPWMValue]
    F --> N{enabl_odometer}
    N -->|是| O[用 IMU yaw 与轮速积分<br/>cur_robot_pos_x/y/yaw]
    O --> P[sum_distance / 位姿供赛题使用]
```

## 4. 循迹数据处理

```mermaid
flowchart TD
    A[LineTrack 40 ms] --> B[GWGetState 读取 8 路灰度状态]
    B --> C{ignore_line_sensor?}
    C -->|是| D[line_trace_result = 0b11100111<br/>调试/起步屏蔽]
    C -->|否| E[使用真实 line_trace_result]
    D --> F[逐位 is_line 判断黑线]
    E --> F
    F --> G{检测到黑线?}
    G -->|是| H[按 omega_weight 累加 detected_omega<br/>并记录 last_detected_omega]
    G -->|否| I[沿用 last_detected_omega]
    H --> J[低通滤波得到 line_track_omega]
    I --> J
    J --> K[line_track_vel = line_trace_exp_vel]
    K --> L{enable_line_track?}
    L -->|是| M[enable_dir_control = false<br/>robot_exp_vel = line_track_vel<br/>robot_exp_omega = line_track_omega + line_trace_exp_omega]
    L -->|否| N[robot_exp_vel = 0<br/>robot_exp_omega = 0]
```

当前灰度转向权重为：

```c
{-1.5f, -0.9f, -0.3f, -0.1f, 0.1f, 0.3f, 0.9f, 1.5f}
```

## 5. K230 视觉与钢球控制数据流

```mermaid
flowchart TD
    A[K230RecvTask<br/>SerialReceive 单字节] --> B{帧头 0x5A?}
    B -->|否| C[header_err++<br/>等待下一字节]
    B -->|是| D[收满 RecvPack]
    D --> E{和校验有效?}
    E -->|否| F[check_err++<br/>K230CommResync]
    E -->|是| G[k230_pack_parse]
    G --> H[PIXEL2POSITION<br/>raw_position]
    H --> I{位置合理<br/>-0.3~0.3 m?}
    I -->|是| J[Kalman1D_Update<br/>ball_filter.position/velocity]
    I -->|否| K[丢弃异常视觉值]
    J --> L{enable_ball_pos_control?}
    K --> L
    L -->|是| M[位置 PID<br/>ball_filter.position -> exp_ball_pos]
    M --> N[速度 PID<br/>ball_filter.velocity -> ball_pos_pid.pid_out]
    N --> O[叠加 stick_angle_feedforward<br/>换算 target_motor_exp_pos]
    O --> P[滤波 + 单步限幅<br/>更新 motor_exp_pos]
    L -->|否| Q[不更新球控外环]
```

`ZDTDriver` 每 4 ms 回读关节角度并执行电机位置命令：

```mermaid
flowchart TD
    A[ZDTDriver 初始化<br/>Kalman1D_Init / MakeZDTSerialEnv] --> B[回读 ZDT 当前角度<br/>joint_cur_pos]
    B --> C{car_is_stop == false?}
    C -->|是| D[由 yaw_rate 与 acc_feedforward<br/>计算 stick_angle_feedforward]
    C -->|否| E[保持前馈状态]
    D --> F{enable_ball_pos_control?}
    E --> F
    F -->|否| G[motor_exp_pos = 中性角]
    F -->|是| H[使用 K230RecvTask 已更新的 motor_exp_pos]
    G --> I[0~50 deg 安全限幅]
    H --> I
    I --> J[Emm_V5_Pos_ControlEx<br/>位置控制 ZDT 电机]
    J --> B
```

## 6. Task1 流程图

Task1 为基础循迹到停止线流程，不启用 K230 视觉和钢球位置控制。

```mermaid
flowchart TD
    A[启动准备<br/>关闭视觉检测，进入任务运行状态]
    A --> B[起步准备<br/>短暂等待后记录起点和开始时间]
    B --> C[高速循迹<br/>起步短时间屏蔽灰度干扰，随后按真实灰度沿线行驶到指定距离]
    C --> D[低速找停止线<br/>降低车速并持续识别停止线]
    D --> E[到线停车<br/>关闭循迹并显示任务耗时]
    E --> F[结束清理<br/>恢复任务状态并退出]
```

## 7. Task2 流程图

Task2 为钢球位置控制流程，底盘不循迹，等待强制退出。

```mermaid
flowchart TD
    A[启动准备<br/>开启视觉检测，进入任务运行状态]
    A --> B[进入球控<br/>底盘保持停止，启用钢球位置闭环]
    B --> C[移动到第一目标<br/>给出第一处目标位置并等待钢球靠近]
    C --> D[切换到第二目标<br/>给出反方向目标位置]
    D --> E[保持控制<br/>持续平衡钢球，直到收到退出指令]
    E --> F[结束清理<br/>关闭球控和视觉检测并退出]
```

## 8. Task3 流程图

Task3 为直线循迹运动加钢球平衡控制，使用五次多项式规划距离-速度-加速度。

```mermaid
flowchart TD
    A[启动准备<br/>开启视觉检测和钢球平衡，进入运动准备状态]
    A --> B[等待发车<br/>红灯提示，等待人工按键确认]
    B --> C[发车提示<br/>按键松开后亮绿灯]
    C --> D[轨迹规划<br/>生成直线行驶的平滑速度曲线]
    D --> E[直线循迹<br/>按规划速度沿线前进，同时保持钢球平衡]
    E --> F[任务收尾<br/>关闭循迹、球控和视觉检测]
    F --> G[结束清理<br/>恢复任务状态并退出]
```

## 9. Task4 流程图

Task4 为带钢球平衡的路径任务：直线、圆弧、直线、圆弧、末端减速停止。

```mermaid
flowchart TD
    A[启动准备<br/>开启视觉检测和钢球平衡，进入运动准备状态]
    A --> B[等待发车<br/>红灯提示，等待人工按键确认]
    B --> C[发车准备<br/>亮绿灯并短暂等待]
    C --> D[第一段直线<br/>按平滑轨迹沿线前进，起步阶段短暂屏蔽灰度干扰]
    D --> E[第一段圆弧<br/>保持循迹前进，并平滑叠加转弯动作]
    E --> F[第二段直线<br/>继续按平滑轨迹沿线前进]
    F --> G[第二段圆弧<br/>继续转弯，末段短暂屏蔽灰度干扰]
    G --> H[末端停车<br/>按减速曲线逐步停下]
    H --> I[任务收尾<br/>关闭循迹、球控和视觉检测，底盘停止输出]
    I --> J[结束清理<br/>恢复任务状态并退出]
```

## 10. Task5 流程图

Task5 的路径执行部分与 Task4 基本一致；不同点是启动阶段先等待人工确认当前钢球位置，并将钢球平衡目标锁定在当前位置。

```mermaid
flowchart TD
    A[启动准备<br/>开启视觉检测，进入运动准备状态]
    A --> B[第一次确认<br/>红灯提示，等待人工按键确认当前状态]
    B --> C[锁定球位<br/>亮蓝灯，并把钢球当前位置作为平衡目标]
    C --> D[第二次确认<br/>等待人工按键确认发车]
    D --> E[发车准备<br/>亮绿灯并短暂等待]
    E --> F[第一段直线<br/>按平滑轨迹沿线前进，起步阶段短暂屏蔽灰度干扰]
    F --> G[第一段圆弧<br/>保持循迹前进，并平滑叠加转弯动作]
    G --> H[第二段直线<br/>继续按平滑轨迹沿线前进]
    H --> I[第二段圆弧<br/>继续转弯，末段短暂屏蔽灰度干扰]
    I --> J[末端停车<br/>按减速曲线逐步停下]
    J --> K[任务收尾<br/>关闭循迹、球控和视觉检测，底盘停止输出]
    K --> L[结束清理<br/>恢复任务状态并退出]
```

## 11. 关键全局数据流向

| 数据/状态 | 写入者 | 读取者 | 含义 |
| --- | --- | --- | --- |
| `current_select_task_id` | `app_main` 按键扫描 | `app_main` 确认逻辑 | 当前 UI 选中的赛题编号 |
| `current_task_id` | `app_main`、赛题任务清理 | `app_main` 任务创建逻辑 | 当前待启动/正在执行的赛题编号 |
| `task_running` | `app_main`、Task1~5 | `app_main` | 防止重复创建赛题任务 |
| `force_exit` | `app_main` K1、Task1~5 | Task2~5 | 强制退出任务流程 |
| `k230_cmd` | `app_main`、Task1~5、测试逻辑 | `app_main` 串口发送、K230 外设 | 通知 K230 是否进入视觉检测状态 |
| `mpu6050_yaw` | `IMUTask` | `WheelTask`、OLED | 当前航向角 |
| `mpu6050_yaw_rate_dps` | `IMUTask` | `ZDTDriver` | 车体旋转前馈计算 |
| `g_encoder1/2.encoder_value` | 编码器中断 | `WheelTask` | 轮编码器累计值 |
| `sum_distance` | `WheelTask` | Task1/3/4/5、OLED | 底盘累计行驶距离 |
| `cur_robot_pos_x/y/yaw` | `WheelTask` | OLED/调试 | 里程计位姿 |
| `line_trace_result` | `LineTrack` | Task1 | 8 路灰度当前状态 |
| `line_trace_exp_vel` | Task1/3/4/5 | `LineTrack` | 循迹目标线速度 |
| `line_trace_exp_omega` | Task4/5 | `LineTrack` | 圆弧段叠加角速度 |
| `enable_line_track` | Task1/3/4/5、Task2 | `LineTrack` | 是否把循迹结果输出到底盘 |
| `ignore_line_sensor` | Task1/3/4/5 | `LineTrack` | 是否临时屏蔽真实灰度输入 |
| `robot_exp_vel/omega` | `LineTrack` | `WheelTask` | 底盘最终速度指令 |
| `raw_position` | `K230RecvTask` | VOFA/调试 | 视觉位置物理量 |
| `ball_filter.position/velocity` | `K230RecvTask` | Task2/5、ZDT/球控逻辑 | 钢球位置与速度估计 |
| `exp_ball_pos` | Task2/3/4/5 | `K230RecvTask` 球控外环 | 钢球期望位置 |
| `enable_ball_pos_control` | `app_main`、Task2~5 | `K230RecvTask`、`ZDTDriver` | 是否启用钢球位置闭环 |
| `car_is_stop` | Task3/4/5、清理逻辑 | `ZDTDriver` | 是否计算车体运动前馈 |
| `acc_feedforward` | Task3/4/5 | `ZDTDriver` | 直线加减速前馈 |
| `stick_angle_feedforward` | `ZDTDriver` | `K230RecvTask` | 钢球控制目标中的机构角度前馈 |
| `motor_exp_pos` | `K230RecvTask`、`ZDTDriver` | `ZDTDriver` | ZDT 电机目标位置 |

## 12. Task1~5 对主要控制开关的使用

| 任务 | `k230_cmd` | `enable_line_track` | `enable_ball_pos_control` | `car_is_stop` | 退出方式 |
| --- | --- | --- | --- | --- | --- |
| Task1 | 0 | 开启后循迹，到停止线关闭 | 不启用 | 默认保持 | 到停止线后自动退出 |
| Task2 | 1 | 关闭 | 开启 | 默认保持 | 等待 `force_exit` |
| Task3 | 1 | 五次轨迹直线段开启 | 开启 | `false` | 轨迹完成或 `force_exit` |
| Task4 | 1 | 五段路径期间开启 | 开启 | `false`，结束置 `true` | 路径完成或 `force_exit` |
| Task5 | 1 | 五段路径期间开启 | 先锁定当前球位后开启 | `false`，结束置 `true` | 路径完成或 `force_exit` |
