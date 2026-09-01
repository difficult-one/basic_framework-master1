#pragma once
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

// Board selection
#define ONE_BOARD  //单板控制整车
// #define CHASSIS_BOARD  //底盘板（双板模式）
// #define GIMBAL_BOARD   //云台板（双板模式）

// Vision transport
#define VISION_USE_VCP  //视觉数据通过VCP传输
// #define VISION_USE_UART

// Gimbal parameters（云台参数）
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台对准底盘时的yaw电机编码器值
#define YAW_ECD_GREATER_THAN_4096 0  // 云台yaw电机编码器值是否大于4096, 0=小于4096, 1=大于4096
#define PITCH_HORIZON_ECD 3412  //pitch水平时的编码器的值
#define PITCH_MAX_ANGLE 0  //pitch最大角度限制（先占位但不引用）
#define PITCH_MIN_ANGLE 0  //pitch最小角度设置（先占位但不引用）

// Shooter parameters
#define ONE_BULLET_DELTA_ANGLE 36  //拨盘发一发弹丸转过的角度
#define REDUCTION_RATIO_LOADER 19.0f  //拨盘电机减速比
#define NUM_PER_CIRCLE 10  //拨盘一圈装弹量

// Chassis geometry, unit: mm
#define WHEEL_BASE 350  //纵向轴距
#define TRACK_WIDTH 300  //横向轮距
#define CENTER_GIMBAL_OFFSET_X 0  //云台中心相对于底盘中心的X方向偏移
#define CENTER_GIMBAL_OFFSET_Y 0  //云台中心相对于底盘中心的Y方向偏移
#define RADIUS_WHEEL 60  //轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f  //轮子电机减速比

// IMU to gimbal axis directions（陀螺仪方向映射）
#define GYRO2GIMBAL_DIR_YAW 1   //1=同向，-1=反向
#define GYRO2GIMBAL_DIR_PITCH 1
#define GYRO2GIMBAL_DIR_ROLL 1

// Chassis actual yaw rate (deg/s), updated in chassis.c
extern float chassis_wz_for_gimbal;

#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

// Chassis motor CAN IDs
#define CHASSIS_MOTOR_LF_ID 1
#define CHASSIS_MOTOR_RF_ID 2
#define CHASSIS_MOTOR_LB_ID 4
#define CHASSIS_MOTOR_RB_ID 3

// INA226 power-meter board CAN feedback
#define POWER_METER_ENABLE 1    //
#define POWER_METER_USE_CAN1 1
#define POWER_METER_USE_CAN2 0
#define POWER_METER_TX_ID 0x605
#define POWER_METER_RX_ID 0x605
#define POWER_METER_DAEMON_COUNT 100
#define POWER_METER_TIMEOUT_MS 100U // 需大于实际报文周期；超过此时间即停止使用旧反馈
#define REFEREE_POWER_TIMEOUT_MS 300U
#define REFEREE_POWER_LIMIT_TIMEOUT_MS 1000U
#define POWER_METER_IDLE_POWER_W 0.0f
#define POWER_METER_CURRENT_SCALE 1.0f  // 电流缩放因子
#define POWER_METER_POWER_SCALE 1.0f    // 功率缩放因子
#define POWER_METER_DEBUG_PRINT_PERIOD 100
#define POWER_METER_UART_PRINT_ENABLE 1 // 1=串口打印功率计CAN原始报文
#define POWER_METER_UART_HANDLE (&huart1) // 默认USART1，若占用可改为其他UART句柄
#define POWER_METER_DEBUG_CAN_HANDLE (&hcan1)

// Chassis motor reverse flags: 0 = normal, 1 = reverse
#define CHASSIS_MOTOR_LF_REVERSE 1
#define CHASSIS_MOTOR_RF_REVERSE 1
#define CHASSIS_MOTOR_LB_REVERSE 1
#define CHASSIS_MOTOR_RB_REVERSE 1

// Chassis drive type and command trim
#define CHASSIS_DRIVE_MECANUM 0 // 麦克纳姆轮底盘
#define CHASSIS_DRIVE_OMNI_X 1  // 全向轮X型底盘
#define CHASSIS_DRIVE_STEERING 2 // 四舵轮底盘
#define CHASSIS_DRIVE_TYPE CHASSIS_DRIVE_MECANUM    //底盘的类型
#define CHASSIS_CMD_DIR_TRIM_DEG 0.0f   //坐标系方向修正角

// Steering chassis parameters
#define STEERING_MOTOR_LF_ID 1
#define STEERING_MOTOR_RF_ID 2
#define STEERING_MOTOR_LB_ID 3
#define STEERING_MOTOR_RB_ID 4
#define STEERING_MOTOR_LF_ZERO_DEG 0.0f
#define STEERING_MOTOR_RF_ZERO_DEG 0.0f
#define STEERING_MOTOR_LB_ZERO_DEG 0.0f
#define STEERING_MOTOR_RB_ZERO_DEG 0.0f
#define STEERING_POWER_RATIO 0.20f

// Single-wheel test mode
#define CHASSIS_WHEEL_TEST_DISABLE 0    //关闭单轮测试
#define CHASSIS_WHEEL_TEST_ENABLE 1   //开启单轮测试
#define CHASSIS_WHEEL_TEST_MODE CHASSIS_WHEEL_TEST_DISABLE  //是否开启单轮测试模式
#define CHASSIS_WHEEL_TEST_LF 0 // 测试左前轮
#define CHASSIS_WHEEL_TEST_RF 1 // 测试右前轮
#define CHASSIS_WHEEL_TEST_LB 2 // 测试左后轮
#define CHASSIS_WHEEL_TEST_RB 3 // 测试右后轮
#define CHASSIS_WHEEL_TEST_TARGET CHASSIS_WHEEL_TEST_LF // 测试目标轮子
#define CHASSIS_WHEEL_TEST_SPEED 1500.0f    // 测试轮子目标速度 (单位: °/s)

#pragma pack(1)

typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;
// 机器人控制模式枚举
typedef enum
{
    ROBOT_CONTROL_REMOTE = 0,
    ROBOT_CONTROL_MOUSE_KEY,
    ROBOT_CONTROL_VISION_AUTO,
} Robot_Control_Mode_e;

//应用程序的运行状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,// 底盘电机不输出力矩,自由转动，即断电模式
    CHASSIS_ROTATE,  //小陀螺
    CHASSIS_NO_FOLLOW,
    CHASSIS_FOLLOW_GIMBAL_YAW,
} chassis_mode_e;

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,// 云台电机不输出力矩,自由转动，即断电模式
    GIMBAL_FREE_MODE,   // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    GIMBAL_GYRO_MODE,   // 云台陀螺仪模式,使用IMU的姿态数据作为电机的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
} gimbal_mode_e;

typedef enum
{
    SHOOT_OFF = 0,  //关
    SHOOT_ON,
} shoot_mode_e;

//摩擦轮开关
typedef enum
{
    FRICTION_OFF = 0,
    FRICTION_ON,
} friction_mode_e;

//弹舱盖开关枚举
typedef enum
{
    LID_OPEN = 0,
    LID_CLOSE,
} lid_mode_e;

//拨盘模式枚举
typedef enum
{
    LOAD_STOP = 0,
    LOAD_REVERSE,//拨盘反转
    LOAD_1_BULLET,
    LOAD_3_BULLET,
    LOAD_BURSTFIRE,//连发模式
} loader_mode_e;

//裁判系统 UI 功率显示结构体
typedef struct
{
    float chassis_power_mx; //底盘功率缓冲能量条的显示值
} Chassis_Power_Data_s;

// 底盘控制命令结构体
typedef struct
{
    float vx;              // 云台坐标系下的前进速度
    float vy;              // 云台坐标系下的左移速度
    float wz;              // 旋转角速度（由 chassis.c 内部根据模式重新计算）
    float offset_angle;    // 云台 yaw 电机相对底盘正前方的夹角（°）
    chassis_mode_e chassis_mode;  // 底盘模式
    int chassis_speed_buff;       // 底盘速度档位（%）
} Chassis_Ctrl_Cmd_s;

// 云台控制命令结构体
typedef struct
{
    float yaw;                // yaw 轴目标角度（增量式累加）
    float pitch;              // pitch 轴目标角度（增量式累加）
    float chassis_rotate_wz;  // 底盘当前自旋角速度（预留，未使用）
    gimbal_mode_e gimbal_mode; // 云台控制模式
} Gimbal_Ctrl_Cmd_s;
//增量式：摇杆位置 = 转速，累加到角度（云台，松手就停在当前位置）

// 发射控制命令结构体
typedef struct
{
    shoot_mode_e shoot_mode;       // 发射总开关
    loader_mode_e load_mode;       // 拨弹模式
    lid_mode_e lid_mode;           // 弹舱盖模式
    friction_mode_e friction_mode; // 摩擦轮模式
    Bullet_Speed_e bullet_speed;   // 弹速档位
    uint8_t rest_heat;             // 枪口剩余热量（预留）
    float shoot_rate;              // 射频：发/秒
} Shoot_Ctrl_Cmd_s;

// 底盘回传的反馈数据结构体
typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)
    // attitude_t chassis_imu_data;   // 双板模式下底盘板的 IMU 数据（已注释）
#endif
    uint8_t rest_heat;               // 枪口剩余热量
    Bullet_Speed_e bullet_speed;     // 当前弹速限制
    Enemy_Color_e enemy_color;       // 敌方颜色（0=蓝, 1=红）
} Chassis_Upload_Data_s;

// 云台回传的反馈数据结构体
typedef struct
{
    attitude_t gimbal_imu_data;                // 云台 IMU 姿态数据（Yaw/Pitch/Roll + 角速度）
    uint16_t yaw_motor_single_round_angle;     // yaw 电机单圈编码器角度（0~360°）
} Gimbal_Upload_Data_s;

// 发射回传的反馈数据结构体
typedef struct
{
    // Reserved for future feedback fields.
} Shoot_Upload_Data_s;

#pragma pack()  // 恢复默认对齐方式

#endif // ROBOT_DEF_H
