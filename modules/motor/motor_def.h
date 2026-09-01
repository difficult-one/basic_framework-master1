/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义
 *         本文件定义了所有电机模块共用的枚举、结构体，是电机系统的"数据类型字典"
 *         - 枚举：限定取值范围（闭环类型、电机型号、正反转等）
 *         - 结构体：打包电机的测量值、PID控制器、初始化配置等
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "controller.h"
#include "stdint.h"

/**
 * @brief 数值限幅宏：将 x 钳位在 [min, max] 区间内
 *        例如 x=150, min=0, max=100 → x 被截断为 100
 */
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/*==============================================================================
  闭环类型枚举 —— 决定电机的控制回路有几层、最外层是什么
  使用位编码（bit mask），可以用位或运算符 | 组合多个闭环
  例如 SPEED_LOOP | CURRENT_LOOP = 0b0011 表示速度环+电流环串级控制
==============================================================================*/

/**
 * @brief 闭环类型枚举
 *        电机的串级控制结构由内到外依次是：电流环 → 速度环 → 角度环
 *        - 电流环（最内层）：控制电机力矩/电流，响应最快，直接决定最终输出
 *        - 速度环（中  层）：控制电机转速，输出作为电流环的参考值
 *        - 角度环（最外层）：控制电机位置/角度，输出作为速度环的参考值
 *        不需要某层时可以只启用内层，如纯速度控制只需要 SPEED_LOOP | CURRENT_LOOP
 */
typedef enum
{
    OPEN_LOOP                  = 0b0000, /**< 开环控制：直接设定电流值，不经过PID计算 */

    CURRENT_LOOP               = 0b0001, /**< 电流环（最内层）：反馈=电机实际电流，输出=最终CAN发送值 */

    SPEED_LOOP                 = 0b0010, /**< 速度环（中层）：反馈=电机转速(角速度)，输出=电流环的参考值 */

    ANGLE_LOOP                 = 0b0100, /**< 角度环（最外层）：反馈=电机角度(总角度)，输出=速度环的参考值 */

    /* 以下为组合值，仅用于代码中的判断，不需要显式设置 */
    SPEED_AND_CURRENT_LOOP     = 0b0011, /**< 速度环+电流环 双环串级 */

    ANGLE_AND_SPEED_LOOP       = 0b0110, /**< 角度环+速度环 双环串级（电流环未启用时） */

    ALL_THREE_LOOP             = 0b0111, /**< 角度环+速度环+电流环 三环全开 */
} Closeloop_Type_e;


/*==============================================================================
  前馈类型枚举 —— 前馈是一种"预判"机制，不等PID算出误差就直接补偿
  例如云台pitch电机，在已知负载重力矩的情况下提前加上补偿电流
  ==============================================================================*/

typedef enum
{
    FEEDFORWARD_NONE                  = 0b00, /**< 不使用任何前馈 */

    CURRENT_FEEDFORWARD               = 0b01, /**< 电流前馈：直接将前馈值加到电流环输出上 */

    SPEED_FEEDFORWARD                 = 0b10, /**< 速度前馈：直接将前馈值加到速度环输出上 */

    CURRENT_AND_SPEED_FEEDFORWARD     = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD, /**< 同时开启速度和电流前馈 */
} Feedfoward_Type_e;


/*==============================================================================
  反馈来源枚举 —— 闭环控制需要知道"现在到底是什么状态"
  反馈可以来自电机自带的传感器，也可以来自外部传感器（如IMU、编码器）
  ==============================================================================*/

/**
 * @brief 反馈数据来源
 *        - MOTOR_FEED：使用电机自带编码器/电流传感器的数据（默认方式）
 *        - OTHER_FEED：使用外部传感器的数据（如云台电机用IMU的姿态角做角度反馈）
 *        设为 OTHER_FEED 时，必须同时设置 Motor_Controller_s 中对应的指针
 */
typedef enum
{
    MOTOR_FEED = 0, /**< 使用电机自带的编码器+电流传感器作为反馈 */

    OTHER_FEED,     /**< 使用外部数据来源（如IMU陀螺仪），需要额外指定数据指针 */
} Feedback_Source_e;


/*==============================================================================
  方向标志枚举 —— 电机转动方向和反馈方向可以独立设置
  为什么要分开？因为反馈值的正方向和电机转动的正方向可能不同
  ==============================================================================*/

/**
 * @brief 电机输出方向标志
 *        - NORMAL：PID计算出的正值→电机正转
 *        - REVERSE：PID计算出的正值→电机反转（自动取反）
 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL  = 0, /**< 电机正转方向与设定值正方向一致 */

    MOTOR_DIRECTION_REVERSE = 1, /**< 电机正转方向与设定值正方向相反，PID输出自动取反 */
} Motor_Reverse_Flag_e;

/**
 * @brief 反馈量方向标志
 *        - NORMAL：传感器读数增加=正方向
 *        - REVERSE：传感器读数增加=负方向，PID计算前自动取反
 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL  = 0, /**< 反馈值方向与预期一致，不做处理 */

    FEEDBACK_DIRECTION_REVERSE = 1, /**< 反馈值方向与预期相反，自动取反后再计算PID */
} Feedback_Reverse_Flag_e;

/**
 * @brief 电机工作状态标志
 *        - STOP：电机停止，PID不计算，CAN报文中该电机位填充0（完全断电）
 *        - ENABLED：电机正常响应设定值
 */
typedef enum
{
    MOTOR_STOP    = 0, /**< 电机停止：急停模式或模块离线时，强制输出0电流 */

    MOTOR_ENALBED = 1, /**< 电机使能：正常计算PID并输出控制量 */
} Motor_Working_Type_e;


/*==============================================================================
  电机型号枚举 —— 框架支持的所有电机类型
  不同的电机有不同的通信协议、反馈格式和物理参数
  ==============================================================================*/

/**
 * @brief 电机型号枚举
 *        - GM6020：大疆云台电机，扭矩大，自带编码器，常用于yaw/pitch两轴云台
 *        - M3508：大疆底盘/摩擦轮电机，转速高，常用于底盘轮组和发射摩擦轮
 *        - M2006：大疆拨盘电机，体积小，常用于发射机构的拨弹盘
 *        - LK9025：瓴控电机，扭矩更大，适合工程机器人/英雄等大功率场合
 *        - HT04：海军工程电机，特殊的CAN协议，需要"一发一收"模式
 */
typedef enum
{
    MOTOR_TYPE_NONE = 0, /**< 未指定电机类型（非法值，用于错误检测） */

    GM6020,              /**< DJI GM6020：云台电机，电调集成在电机内部 */

    M3508,               /**< DJI M3508：高速电机，搭配C620电调，减速比约19:1 */

    M2006,               /**< DJI M2006：小型电机，搭配C610电调，常用于拨弹盘 */

    LK9025,              /**< 瓴控 LK9025：大扭矩电机，CAN协议，需参考厂家手册 */

    HT04,                /**< 海泰 HT04：海军工程电机，CAN协议不同，反馈为应答式 */
} Motor_Type_e;


/*==============================================================================
  电机控制设置 —— 定义一个电机的"控制模式"
  包括：用几层闭环、每个闭环的反馈从哪里来、方向要不要反转、是否加前馈
  ==============================================================================*/

/**
 * @brief 电机控制设置结构体
 *        这个结构体描述了电机的"控制策略"，在电机初始化时配置，运行中也可动态修改
 *        例如云台在GYRO_MODE时需要把反馈来源切到IMU，就会修改 angle_feedback_source
 */
typedef struct
{
    /**
     * @brief 最外层闭环类型
     *        决定了设定值 ref 是角度、速度还是电流
     *        - 设为 ANGLE_LOOP：ref 表示目标角度，级联计算 角度→速度→电流 后输出
     *        - 设为 SPEED_LOOP：ref 表示目标速度，级联计算 速度→电流 后输出
     *        - 设为 CURRENT_LOOP：ref 表示目标电流，直接输出（开环等价于只设电流环）
     */
    Closeloop_Type_e outer_loop_type;

    /**
     * @brief 启用的闭环集合（位或组合）
     *        例如 ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP 表示三环全开
     *        注意：启用的闭环必须包含 outer_loop_type 和所有更内层的闭环
     */
    Closeloop_Type_e close_loop_type;

    Motor_Reverse_Flag_e motor_reverse_flag;       /**< 电机输出是否反转 */

    Feedback_Reverse_Flag_e feedback_reverse_flag; /**< 反馈数据是否反转 */

    Feedback_Source_e angle_feedback_source;       /**< 角度反馈来源：MOTOR_FEED(编码器) 或 OTHER_FEED(如IMU) */

    Feedback_Source_e speed_feedback_source;       /**< 速度反馈来源：MOTOR_FEED(编码器微分) 或 OTHER_FEED(如陀螺仪) */

    Feedfoward_Type_e feedforward_flag;            /**< 前馈使能标志：NONE / CURRENT / SPEED / BOTH */

} Motor_Control_Setting_s;


/*==============================================================================
  电机控制器 —— 电机PID计算的核心
  包含三环PID实例 + 外部反馈数据指针 + 前馈数据指针 + 参考值
  ==============================================================================*/

/**
 * @brief 电机控制器结构体（运行时的PID计算核心）
 *        每个电机实例内嵌一个 Motor_Controller_s，负责：
 *        1. 保存三环 PID 的状态（误差、积分项、上一次输出等）
 *        2. 指向外部反馈数据（如IMU），实现柔性反馈切换
 *        3. 通道变量 pid_ref 串起各层闭环：角度环输出→速度环输入→电流环输入→最终输出
 *
 *        PID串级数据流示意：
 *        pid_ref(角度设定值)
 *            → [角度环PID] → pid_ref(更新为速度设定值)
 *            → [速度环PID] → pid_ref(更新为电流设定值)
 *            → [电流环PID] → pid_ref(更新为最终电流输出值) → 填入CAN报文发送
 */
typedef struct
{
    float *other_angle_feedback_ptr; /**< 外部角度反馈数据指针，如指向 IMU 的 YawTotalAngle */

    float *other_speed_feedback_ptr; /**< 外部速度反馈数据指针，如指向 IMU 的 Gyro[Z] */

    float *speed_feedforward_ptr;    /**< 速度前馈数据指针，前馈值直接加在速度环输出上 */

    float *current_feedforward_ptr;  /**< 电流前馈数据指针，前馈值直接加在电流环输出上（如重力补偿） */

    PIDInstance current_PID; /**< 电流环PID实例：输入=电流误差，输出=最终控制量 */

    PIDInstance speed_PID;   /**< 速度环PID实例：输入=速度误差，输出=电流参考值 */

    PIDInstance angle_PID;   /**< 角度环PID实例：输入=角度误差，输出=速度参考值 */

    /**
     * @brief PID参考值通道变量
     *        在整个串级计算中 pid_ref 是"接力棒"：
     *        app 调用 DJIMotorSetRef() 写入初始设定值
     *        → 角度环拿它当设定值，计算后把输出写回 pid_ref
     *        → 速度环拿它当设定值，计算后把输出写回 pid_ref
     *        → 电流环拿它当设定值，计算后的输出就是最终CAN发送值
     */
    float pid_ref;

} Motor_Controller_s;


/*==============================================================================
  电机控制器初始化结构体 —— 注册电机时传入的参数包
  与 Motor_Controller_s 的区别：
  - Motor_Controller_Init_s：初始化时传入的配置（PID参数、外部指针）
  - Motor_Controller_s：运行时实际持有的控制数据（PID状态、实时参考值）
  ==============================================================================*/

/**
 * @brief 电机控制器初始化配置
 *        在 app 层调用 DJIMotorInit() 时，需要填写这个结构体
 *        每个PID环不需要时可以留空（全0），不会被初始化
 *        外部反馈指针只在 feedback_source == OTHER_FEED 时有效
 */
typedef struct
{
    float *other_angle_feedback_ptr; /**< 外部角度反馈源指针，注意填的是 total_angle 的地址 */

    float *other_speed_feedback_ptr; /**< 外部速度反馈源指针，单位是 角度/秒 */

    float *speed_feedforward_ptr;    /**< 速度前馈源指针，不需要时填 NULL */

    float *current_feedforward_ptr;  /**< 电流前馈源指针，不需要时填 NULL（如pitch重力补偿力矩） */

    PID_Init_Config_s current_PID; /**< 电流环PID初始化参数：Kp/Ki/Kd/积分限幅/输出限幅/优化选项 */

    PID_Init_Config_s speed_PID;   /**< 速度环PID初始化参数：Kp/Ki/Kd/积分限幅/输出限幅/优化选项 */

    PID_Init_Config_s angle_PID;   /**< 角度环PID初始化参数：Kp/Ki/Kd/积分限幅/输出限幅/优化选项 */

} Motor_Controller_Init_s;


/*==============================================================================
  电机统一初始化结构体 —— 各类电机注册的统一入口参数
  无论是 DJI、LK、HT 电机，初始化时都用这个结构体传参
  ==============================================================================*/

/**
 * @brief 电机初始化配置（最顶层，各类CAN电机通用）
 *        这个结构体是 app 层初始化电机时唯一需要填写的入口
 *        它把CAN通信配置 + PID参数 + 控制策略 + 电机型号 打包在一起
 *
 *        使用示例（在 chassis.c 中）：
 *        Motor_Init_Config_s config = {
 *            .can_init_config = { .can_handle = &hcan1, .tx_id = 1 },
 *            .controller_param_init_config = { .speed_PID = { .Kp = 4.5, ... } },
 *            .controller_setting_init_config = { .outer_loop_type = SPEED_LOOP, ... },
 *            .motor_type = M3508,
 *        };
 *        DJIMotorInstance *motor = DJIMotorInit(&config);
 */
typedef struct
{
    /**
     * @brief PID参数和外部反馈指针的初始化配置
     */
    Motor_Controller_Init_s controller_param_init_config;

    /**
     * @brief 控制策略配置：外层闭环类型、启用的闭环集合、反馈来源、方向等
     */
    Motor_Control_Setting_s controller_setting_init_config;

    /**
     * @brief 电机型号：GM6020 / M3508 / M2006 / LK9025 / HT04
     *        初始化函数会根据型号自动区分 CAN 分组和协议格式
     */
    Motor_Type_e motor_type;

    /**
     * @brief CAN通信配置：挂载的总线句柄(hcan1/hcan2)、发送ID(tx_id)
     *        接收ID(rx_id)会在初始化时根据电机型号自动计算，不需要手动填写
     */
    CAN_Init_Config_s can_init_config;

} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
