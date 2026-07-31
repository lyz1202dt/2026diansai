#!/usr/bin/env python3
import copy
import os
import zipfile
from xml.etree import ElementTree as ET

W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
XML_NS = "http://www.w3.org/XML/1998/namespace"
ET.register_namespace("w", W_NS)

TEMPLATE = "/home/lyz/桌面/25电赛报告（C题）姬晓洋111.docx"
OUT = "docs/2026_H题_车载平衡滚球运动控制系统设计报告.docx"


def wtag(name):
    return f"{{{W_NS}}}{name}"


def el(name, attrs=None, text=None):
    node = ET.Element(wtag(name), attrs or {})
    if text is not None:
        node.text = text
    return node


def text_run(text, bold=False, size=None, font="宋体"):
    r = el("r")
    rpr = el("rPr")
    if bold:
        rpr.append(el("b"))
    if size:
        rpr.append(el("sz", {wtag("val"): str(size)}))
        rpr.append(el("szCs", {wtag("val"): str(size)}))
    fonts = el(
        "rFonts",
        {
            wtag("ascii"): "Times New Roman",
            wtag("hAnsi"): "Times New Roman",
            wtag("eastAsia"): font,
            wtag("cs"): "Times New Roman",
        },
    )
    rpr.append(fonts)
    r.append(rpr)
    t = el("t", {f"{{{XML_NS}}}space": "preserve"}, text)
    r.append(t)
    return r


def para(text="", style=None, align=None, bold=False, size=None, first_line=True):
    p = el("p")
    ppr = el("pPr")
    if style:
        ppr.append(el("pStyle", {wtag("val"): style}))
    if align:
        ppr.append(el("jc", {wtag("val"): align}))
    if first_line and style is None and align != "center":
        ppr.append(el("ind", {wtag("firstLineChars"): "200"}))
    ppr.append(el("spacing", {wtag("line"): "360", wtag("lineRule"): "auto"}))
    p.append(ppr)
    if text:
        p.append(text_run(text, bold=bold, size=size))
    return p


def heading(text, level=1):
    style = {1: "2", 2: "3", 3: "4"}.get(level, "3")
    return para(text, style=style, bold=True, first_line=False)


def caption(text):
    return para(text, style="11", align="center", first_line=False)


def photo_placeholder(text):
    return table([["照片占位", text]], widths=[1800, 7600])


def table(rows, widths=None):
    tbl = el("tbl")
    tblpr = el("tblPr")
    tblpr.append(el("tblW", {wtag("w"): "0", wtag("type"): "auto"}))
    borders = el("tblBorders")
    for side in ["top", "left", "bottom", "right", "insideH", "insideV"]:
        borders.append(el(side, {wtag("val"): "single", wtag("sz"): "4", wtag("space"): "0", wtag("color"): "000000"}))
    tblpr.append(borders)
    tbl.append(tblpr)
    if widths is None and rows:
        widths = [9000 // len(rows[0])] * len(rows[0])
    grid = el("tblGrid")
    for width in widths or []:
        grid.append(el("gridCol", {wtag("w"): str(width)}))
    tbl.append(grid)
    for row in rows:
        tr = el("tr")
        for i, cell in enumerate(row):
            tc = el("tc")
            tcpr = el("tcPr")
            if widths:
                tcpr.append(el("tcW", {wtag("w"): str(widths[min(i, len(widths) - 1)]), wtag("type"): "dxa"}))
            tc.append(tcpr)
            for line in str(cell).split("\n"):
                tc.append(para(line, align="center", first_line=False))
            tr.append(tc)
        tbl.append(tr)
    return tbl


def section_pr_from_template():
    with zipfile.ZipFile(TEMPLATE) as zf:
        root = ET.fromstring(zf.read("word/document.xml"))
    body = root.find(wtag("body"))
    sect = body.find(wtag("sectPr"))
    return copy.deepcopy(sect)


def build_body():
    body = el("body")
    items = []

    items.append(para("车载平衡滚球运动控制系统（H题）", style="16", align="center", bold=True, size=32, first_line=False))
    items.append(para("【本科组】", style="14", align="center", bold=True, size=24, first_line=False))
    items.append(heading("摘要", 1))
    items.append(para("本系统设计并制作了一辆搭载摆杆滚球平衡机构的循线小车。系统以 MSPM0G3507 为主控制器，采用 FreeRTOS 组织多任务控制流程，利用 8 路红外灰度传感器完成黑线循迹，利用 MPU6050 获取车体航向与角速度，利用编码器完成轮速闭环和里程估计，利用 K230 视觉模块检测钢球在摆杆凹槽中的位置，并通过 ZDT 闭环电机调节摆杆角度，实现静止与行驶状态下的钢球位置稳定控制。程序采用速度 PID、钢球位置-速度级联 PID、一维 Kalman 滤波、五次多项式轨迹规划和圆弧角速度平滑控制，完成题目规定的按键选题、循线行驶、定点停车、钢球位置调节和行驶过程平衡控制等功能。"))
    items.append(para("关键词：MSPM0G3507；FreeRTOS；红外循迹；K230视觉；Kalman滤波；摆杆滚球控制"))

    items.append(heading("一、系统方案设计与论证", 1))
    items.append(heading("1.1 系统总体方案", 2))
    items.append(para("系统由差速小车底盘、摆杆滚球机构、主控板、红外循迹模块、MPU6050 姿态模块、K230 视觉模块、ZDT 闭环电机、OLED 显示屏和电源模块组成。主控板负责传感器采集、运动控制、任务调度和人机交互；K230 负责从摄像头画面中给出钢球位置；ZDT 电机驱动摆杆末端高度变化，使钢球保持在指定位置附近。"))
    items.append(table([
        ["模块", "选型/实现", "主要功能"],
        ["主控与调度", "MSPM0G3507 + FreeRTOS", "创建 IMU、轮速、循迹、视觉接收、摆杆控制和 Task1~5 任务"],
        ["底盘运动", "双轮差速 + L298N + 编码器", "轮速闭环、距离累计、循迹行驶和圆弧运动"],
        ["循迹检测", "8 路红外光电/灰度模块", "检测黑线位置并生成转向角速度"],
        ["钢球检测", "K230 摄像头视觉", "输出钢球像素位置，经比例换算得到物理位置"],
        ["摆杆控制", "ZDT 闭环电机 + 摆杆机构", "调节摆杆角度，控制钢球位置"],
        ["显示与按键", "OLED + KEY3/KEY5", "选题、启动、退出和运行数据显示"],
    ], widths=[2100, 3100, 4400]))
    items.append(caption("表1 系统组成与功能分配"))
    items.append(photo_placeholder("图1 系统整体实物照片：建议放置小车、摆杆、摄像头和主控板的整体照片。"))

    items.append(heading("1.2 小车循迹控制方案论证", 2))
    items.append(para("方案一采用摄像头识别赛道黑线，优点是信息量丰富，可同时估计偏移和曲率；缺点是算法复杂、延时较大，并且题目说明小车循迹模块只能使用红外光电模块。"))
    items.append(para("方案二采用多路红外光电传感器检测黑线，优点是响应快、实现简单、抗计算负载影响小，适合嵌入式实时控制。系统最终采用 8 路红外灰度模块，通过各传感器黑线状态加权求和得到循迹角速度，并在丢线时沿用上一次检测方向，保证运行连续性。"))
    items.append(heading("1.3 钢球位置检测方案论证", 2))
    items.append(para("钢球位于开槽 PPR 摆杆内，题目要求钢球位置检测必须采用摄像头。系统采用 K230 视觉模块采集摆杆上方图像，识别钢球中心相对刻度的位置，并通过串口向主控发送位置帧。主控对帧头和校验和进行检查，位置值经比例换算和 Kalman 滤波后用于控制。该方案将图像处理与实时控制解耦，减轻主控计算压力。"))
    items.append(heading("1.4 摆杆驱动方案论证", 2))
    items.append(para("摆杆末端驱动可采用舵机、直流电机或闭环步进电机。舵机控制简单但角度分辨率和刚度有限；直流电机需要额外位置反馈；闭环步进电机具备位置控制能力和较高保持力，适合摆杆角度调节。系统采用 ZDT 闭环电机，通过几何换算将期望摆杆角度转换为电机目标角度，再使用电机位置控制命令驱动。"))

    items.append(heading("二、系统理论分析与计算", 1))
    items.append(heading("2.1 差速底盘运动学与轮速闭环", 2))
    items.append(para("底盘采用左右两轮差速结构。设轮半径 R=0.033m，车体等效半径 l=0.104m，线速度指令为 v，角速度指令为 ω，则代码中左右轮期望角速度按下式计算："))
    items.append(para("ω1 = v/R + ω·l/R，ω2 = -v/R + ω·l/R", align="center", first_line=False))
    items.append(para("编码器每 5ms 差分一次，结合编码器线数 52 和减速比 28 计算当前轮速。轮速经一阶低通滤波后进入 PID 控制器，输出 PWM 到 L298N 驱动。系统还根据两轮速度与 IMU 航向积分得到当前位置和累计里程，为 Task1、Task3、Task4、Task5 的距离判断和轨迹跟踪提供反馈。"))
    items.append(heading("2.2 红外循迹转向计算", 2))
    items.append(para("8 路红外模块从左到右设置角速度权重 {-1.5,-0.9,-0.3,-0.1,0.1,0.3,0.9,1.5}。当某一路检测到黑线时，将对应权重累加为 detected_omega，再通过一阶滤波得到 line_track_omega。循迹任务开启时，底盘速度指令为 robot_exp_vel=line_trace_exp_vel，角速度指令为 robot_exp_omega=line_track_omega+line_trace_exp_omega。圆弧段通过 line_trace_exp_omega 叠加固定曲率角速度，使小车在循迹约束下完成半圆轨迹。"))
    items.append(heading("2.3 钢球位置滤波与级联控制", 2))
    items.append(para("K230 发送的钢球像素位置经 PIXEL2POSITION(x)=0.01x+kBallDistanceOffset 换算为米制位置。主控使用一维匀速 Kalman 模型估计钢球位置和速度，状态量为 x=[p,v]^T，预测关系为 p(k)=p(k-1)+v(k-1)dt，v(k)=v(k-1)。滤波后的 ball_filter.position 与 exp_ball_pos 进入位置 PID，输出期望速度；ball_filter.velocity 与期望速度进入速度 PID，输出期望摆杆角度。"))
    items.append(para("行驶时还需补偿车体旋转和直线加减速对钢球的等效惯性力。程序用 yaw_rate 和 acc_feedforward 计算前馈角 stick_angle_feedforward，并与速度环输出相加后转换为电机目标角度。"))
    items.append(heading("2.4 摆杆-电机几何换算", 2))
    items.append(para("摆杆长度取 L=0.25m，驱动支撑半径 r=0.05m，基座高度 BASE_HEIGHT=0.04m。代码中由摆杆角 angle 计算电机角度的关系为："))
    items.append(para("motor_angle = asin((L·sin(angle)+BASE_HEIGHT)/(2r)) + motor_base_angle_offset", align="center", first_line=False))
    items.append(para("为保护机构，电机目标角度限制在 0°~50°，目标变化还经过单步限幅和平滑滤波，避免视觉抖动或异常帧造成机构冲击。"))
    items.append(heading("2.5 五次多项式轨迹规划", 2))
    items.append(para("Task3、Task4、Task5 的直线段采用五次多项式规划距离曲线。设起点位置、速度为 p0、v0，终点位置、速度为 p1、v1，总时间为 T，加速度边界取 0，可生成位置、速度、加速度连续的轨迹。任务执行时每 20ms 采样一次，循迹目标速度由轨迹速度加距离误差反馈得到，轨迹加速度送入钢球前馈控制。"))

    items.append(heading("三、电路与程序设计", 1))
    items.append(heading("3.1 硬件电路设计", 2))
    items.append(para("主控板完成电机驱动、传感器采集和串口通信。双轮电机由 L298N 模块驱动，编码器 A/B 相通过 GPIO 外部中断计数；8 路红外循迹模块连接 GPIO 输入；MPU6050 提供姿态四元数和陀螺仪角速度；UART0 与 K230 通信，UART1 与 ZDT 电机通信，UART3 用于 VOFA 调试输出；OLED 用于显示任务状态、位姿、航向和累计距离。"))
    items.append(photo_placeholder("图2 控制板及电气连接照片：建议标注 MSPM0G3507、K230、ZDT、电机驱动、OLED 和电源接口。"))
    items.append(heading("3.2 程序总体流程", 2))
    items.append(para("系统入口 app_main() 依次执行 SetupConfig()、OLED_Init()，并创建 WheelTask、IMUTask、LineTrack、ZDTDriver 和 K230RecvTask。主循环每 50ms 扫描按键、刷新 OLED、按当前题号创建 Task1~Task5，并周期性向 K230 发送 k230_cmd。task_running 用于避免重复创建赛题任务，force_exit 用于强制退出当前流程。"))
    items.append(table([
        ["任务", "周期/触发", "主要功能"],
        ["IMUTask", "5ms", "读取 MPU6050，更新 yaw、yaw_rate 和数据有效标志"],
        ["WheelTask", "5ms", "编码器差分、轮速 PID、PWM 输出、里程计积分"],
        ["LineTrack", "40ms", "读取 8 路灰度，生成底盘线速度和角速度指令"],
        ["ZDTDriver", "4ms", "回读关节角，计算前馈并发送电机位置控制命令"],
        ["K230RecvTask", "串口阻塞", "接收视觉帧，校验、重同步、Kalman 滤波和球控外环"],
        ["Task1~Task5", "按键创建", "执行各赛题流程，写入循迹、球控和路径规划指令"],
    ], widths=[1900, 1900, 5800]))
    items.append(caption("表2 FreeRTOS 任务分工"))
    items.append(para("图3 系统软件流程图：app_main 初始化后创建常驻任务；按键选择 Task1~Task5；赛题任务写入 enable_line_track、line_trace_exp_vel、line_trace_exp_omega、enable_ball_pos_control、exp_ball_pos、car_is_stop 等全局指令；LineTrack 输出底盘速度指令到 WheelTask；K230RecvTask 和 ZDTDriver 共同完成钢球位置闭环。"))
    items.append(heading("3.3 Task1~Task5 程序流程", 2))
    items.append(table([
        ["赛题任务", "流程概述"],
        ["Task1", "关闭 K230，开启循迹；先以 0.5m/s 高速行驶至停止线前约 5.7m，再以 0.1m/s 寻找启停线；检测到至少 3 路黑线后停车并显示时间。"],
        ["Task2", "开启 K230 和钢球位置闭环；钢球先运行至 +5cm 附近，再切换到 -5cm 目标；保持控制直到按键强制退出。"],
        ["Task3", "钢球目标为中心 O；等待按键后生成 1.7m、7s 五次轨迹；循迹直线行驶并通过 B 点，轨迹加速度参与钢球前馈。"],
        ["Task4", "钢球目标为中心 O；按直线、圆弧、直线、圆弧、末端减速五段路径行驶一圈；圆弧半径 0.5m，巡航速度 0.27m/s。"],
        ["Task5", "先读取当前钢球滤波位置作为 exp_ball_pos；随后执行与 Task4 相同的一圈路径，实现任意指定位置的行驶稳定控制。"],
    ], widths=[1800, 7800]))
    items.append(caption("表3 Task1~Task5 流程说明"))

    items.append(heading("四、测试方案与测试结果", 1))
    items.append(heading("4.1 测试环境与通用方法", 2))
    items.append(para("测试场地按题目图示制作：黑线宽度 1.8±0.2cm，AB、CD 直线段长度 1.5m，BC、DA 为半径 0.5m 半圆弧，A 点设置 5cm 启停线。测试前检查电池电压、摆杆水平初始角、摄像头视野、K230 图传录像、OLED 显示和按键功能。每项测试至少重复 3 次，记录时间、停车偏差和钢球最大偏差。"))
    items.append(photo_placeholder("图4 测试场地与小车起点照片：建议放置 A 点、启停线、环形赛道和小车指定测试位置。"))
    items.append(heading("4.2 小车循迹一圈停车测试", 2))
    items.append(table([
        ["次数", "行驶时间/s", "停车偏差/cm", "是否脱线", "备注"],
        ["1", "待实测", "待实测", "否/是", ""],
        ["2", "待实测", "待实测", "否/是", ""],
        ["3", "待实测", "待实测", "否/是", ""],
    ], widths=[1200, 2200, 2200, 1800, 2200]))
    items.append(caption("表4 Task1 一圈循迹停车测试记录"))
    items.append(heading("4.3 静止钢球位置控制测试", 2))
    items.append(table([
        ["次数", "+5cm 到达时间/s", "+5cm 最大误差/cm", "-5cm 稳定误差/cm", "总时间/s"],
        ["1", "待实测", "待实测", "待实测", "待实测"],
        ["2", "待实测", "待实测", "待实测", "待实测"],
        ["3", "待实测", "待实测", "待实测", "待实测"],
    ], widths=[1000, 2400, 2400, 2400, 1400]))
    items.append(caption("表5 Task2 静止钢球位置控制测试记录"))
    items.append(heading("4.4 行驶过程钢球稳定测试", 2))
    items.append(table([
        ["任务", "测试目标", "行驶时间/s", "钢球最大偏差/cm", "是否完成", "备注"],
        ["Task3", "A 到 B，钢球在 O 点", "待实测", "待实测", "待实测", ""],
        ["Task4", "顺时针一圈，钢球在 O 点", "待实测", "待实测", "待实测", ""],
        ["Task5", "顺时针一圈，钢球在指定位置", "待实测", "待实测", "待实测", ""],
    ], widths=[1100, 2600, 1700, 2200, 1500, 1500]))
    items.append(caption("表6 Task3~Task5 行驶稳定性测试记录"))
    items.append(photo_placeholder("图5 钢球图传画面与录像截图：建议放置 K230/接收端画面，标出钢球位置和刻度。"))
    items.append(heading("4.5 测试结果分析", 2))
    items.append(para("测试时应重点观察三类误差：一是循迹误差，主要受红外模块高度、黑线对比度和速度影响；二是钢球位置误差，主要受视觉识别延时、摆杆摩擦和车体加速度影响；三是停车误差，主要受里程计累计误差和停止线检测阈值影响。若钢球在圆弧段偏差增大，可优先调整 stick_angle_feedforward、ball_vel_pid 和圆弧角速度斜坡时间；若停车偏差偏大，可调整 Task1 的高速距离门限和低速寻线速度。"))

    items.append(heading("参考文献", 1))
    refs = [
        "[1] 全国大学生电子设计竞赛组委会. 2026 年全国大学生电子设计竞赛赛区赛(TI 杯) H 题：车载平衡滚球运动控制系统.",
        "[2] Texas Instruments. MSPM0G3507 Microcontroller Technical Reference Manual.",
        "[3] InvenSense. MPU-6050 Register Map and Descriptions.",
        "[4] R. E. Kalman. A New Approach to Linear Filtering and Prediction Problems[J]. Transactions of the ASME, 1960.",
        "[5] 王晓明, 张永德. 移动机器人路径跟踪控制方法研究[J]. 自动化技术与应用, 2020.",
    ]
    for r in refs:
        items.append(para(r, first_line=False))

    items.append(heading("附录", 1))
    items.append(para("附录一：系统总流程图。"))
    items.append(para("附录二：Task1~Task5 程序流程图。"))
    items.append(para("附录三：控制板、电源、传感器与电机连接图。"))
    items.append(para("附录四：主要程序文件：APP/app_main.c、APP/mytask.c、Lib/pid/PID.c、Lib/kalman/kalman.c、Lib/quintic/quintic.c。"))

    for item in items:
        body.append(item)
    body.append(section_pr_from_template())
    return body


def build_document():
    root = el("document")
    root.append(build_body())
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    document_xml = build_document()
    with zipfile.ZipFile(TEMPLATE, "r") as zin, zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as zout:
        for info in zin.infolist():
            if info.filename == "word/document.xml":
                zout.writestr(info, document_xml)
            elif info.filename.startswith("word/media/"):
                # The new report leaves photo placeholders instead of embedding last year's images.
                continue
            else:
                zout.writestr(info, zin.read(info.filename))
    print(OUT)


if __name__ == "__main__":
    main()
