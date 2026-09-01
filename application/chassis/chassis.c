/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "math.h"

#include "power_manager_api.h"
#include "super_cap.h"
#include "power_meter.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "bsp_usart.h"
#include "usart.h"
#include "infantry_control.h"
#include <stdio.h>

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘类型: 0=麦轮(Mecanum), 1=全向轮(Omni X型) */
#define OMNI_DRIVE (CHASSIS_DRIVE_TYPE == CHASSIS_DRIVE_OMNI_X)
#define STEERING_DRIVE (CHASSIS_DRIVE_TYPE == CHASSIS_DRIVE_STEERING)
#define MOTOR_UART2_DEBUG_PRINT_PERIOD 100
#define M3508_CURRENT_RAW_TO_A 819.2f

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
float chassis_wz_for_gimbal = 0;            // 底盘当前实际角速度(°/s), 由offset_angle微分得到, 供云台yaw前馈

static SuperCapInstance *cap;                                       // 超级电容
static PowerMeterInstance *power_meter_can1;                        // INA226 CAN power meter on CAN1
static PowerMeterInstance *power_meter_can2;                        // INA226 CAN power meter on CAN2
DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
#if STEERING_DRIVE
static DJIMotorInstance *steer_lf, *steer_rf, *steer_lb, *steer_rb;
#endif

// 底盘功率信息，每次 ChassisTask 末尾更新，调试时在这里看
ChassisPowerInfo chassis_power_info = {0};

static void PowerMeterApplyCalibration(PowerMeter_Data_s *data)
{
    if (data == NULL)
        return;

    data->current_a *= POWER_METER_CURRENT_SCALE;
    data->power_w *= POWER_METER_POWER_SCALE;
    // Keep raw int16 fields unchanged; calibrated values may exceed their range.
}

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅

static void FormatFloat3(char *buf, size_t buf_size, float value)
{
    const char *sign = "";
    if (value < 0.0f)
    {
        sign = "-";
        value = -value;
    }
    const int scaled = (int)(value * 1000.0f + 0.5f);
    snprintf(buf, buf_size, "%s%d.%03d", sign, scaled / 1000, scaled % 1000);
}

static void FormatMotorFeedback(const DJIMotorInstance *motor,
                                char *speed_rpm,
                                size_t speed_size,
                                char *current_a,
                                size_t current_size)
{
    if (motor == NULL)
    {
        snprintf(speed_rpm, speed_size, "0.000");
        snprintf(current_a, current_size, "0.000");
        return;
    }

    FormatFloat3(speed_rpm, speed_size, motor->measure.speed_aps / 6.0f);
    FormatFloat3(current_a, current_size,
                 (float)motor->measure.real_current / M3508_CURRENT_RAW_TO_A);
}

static void ChassisPrintMotorFeedbackUART2(void)
{
    DJIMotorInstance *selected_motor = motor_lf;
    const char *wheel_name = "LF";
    float target_speed_aps = vt_lf;
#if CHASSIS_WHEEL_TEST_MODE == CHASSIS_WHEEL_TEST_ENABLE
#if CHASSIS_WHEEL_TEST_TARGET == CHASSIS_WHEEL_TEST_RF
    selected_motor = motor_rf;
    wheel_name = "RF";
    target_speed_aps = vt_rf;
#elif CHASSIS_WHEEL_TEST_TARGET == CHASSIS_WHEEL_TEST_LB
    selected_motor = motor_lb;
    wheel_name = "LB";
    target_speed_aps = vt_lb;
#elif CHASSIS_WHEEL_TEST_TARGET == CHASSIS_WHEEL_TEST_RB
    selected_motor = motor_rb;
    wheel_name = "RB";
    target_speed_aps = vt_rb;
#endif
#endif

    char target_rpm[16];
    char speed_rpm[16];
    char current_a[16];
    char msg[128];

    FormatFloat3(target_rpm, sizeof(target_rpm), target_speed_aps / 6.0f);
    FormatMotorFeedback(selected_motor, speed_rpm, sizeof(speed_rpm), current_a, sizeof(current_a));

    const int len = snprintf(msg, sizeof(msg),
                             "wheel=%s target=%s rpm speed=%s rpm current=%s A\r\n",
                             wheel_name, target_rpm, speed_rpm, current_a);
    if (len > 0)
    {
        const uint16_t tx_len = (len < (int)sizeof(msg)) ? (uint16_t)len : (uint16_t)(sizeof(msg) - 1);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, tx_len, 20);
    }
}

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = CHASSIS_MOTOR_CAN_HANDLE,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5, // 4.5
                .Ki = 0.5,   // 0
                .Kd = 0,   // 0
                .IntegralLimit = 3000,// 积分限幅
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,// PID优化选项
                .MaxOut = 15000,// 输出限幅
                .Output_LPF_RC = 0.3,// 输出滤波器RC
            },
             .current_PID = {
                .Kp = 1, 
                .Ki = 0,   // 0
                .Kd = 0,   // 0
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP|CURRENT_LOOP, // 速度环+电流环 双环串级
        },
        .motor_type = CHASSIS_MOTOR_TYPE,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_LF_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
        CHASSIS_MOTOR_LF_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RF_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
        CHASSIS_MOTOR_RF_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_LB_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
        CHASSIS_MOTOR_LB_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RB_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
        CHASSIS_MOTOR_RB_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    motor_rb =DJIMotorInit(&chassis_motor_config);
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);

#if STEERING_DRIVE
    Motor_Init_Config_s steering_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.5f,
                .MaxOut = 1800.0f,
            },
            .speed_PID = {
                .Kp = 3.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 8000.0f,
            },
            .current_PID = {
                .Kp = 1.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 10000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M2006,
    };

    steering_motor_config.can_init_config.tx_id = STEERING_MOTOR_LF_ID;
    steer_lf = DJIMotorInit(&steering_motor_config);
    steering_motor_config.can_init_config.tx_id = STEERING_MOTOR_RF_ID;
    steer_rf = DJIMotorInit(&steering_motor_config);
    steering_motor_config.can_init_config.tx_id = STEERING_MOTOR_LB_ID;
    steer_lb = DJIMotorInit(&steering_motor_config);
    steering_motor_config.can_init_config.tx_id = STEERING_MOTOR_RB_ID;
    steer_rb = DJIMotorInit(&steering_motor_config);
    DJIMotorStop(steer_lf);
    DJIMotorStop(steer_rf);
    DJIMotorStop(steer_lb);
    DJIMotorStop(steer_rb);
#endif

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

/* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善 */
    PID_Init_Config_s Buffer_pid_conf = {
        .Kp = 0.1,
        .Ki = 0,
        .Kd = 0,
        .IntegralLimit = 1000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 1000,
    };
    PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD

    // 初始化底盘功率管理器（替换旧PowerControl）
    ChassisPower_Init(motor_lf, motor_rf, motor_lb, motor_rb);
    ChassisPower_SetModelIdlePower(POWER_METER_IDLE_POWER_W);
#if POWER_METER_ENABLE
    can_debug_watch_id = POWER_METER_RX_ID;
#if POWER_METER_UART_PRINT_ENABLE
    CANEnableDebugPassthrough(POWER_METER_DEBUG_CAN_HANDLE);
#endif
#if POWER_METER_USE_CAN1
    PowerMeter_Init_Config_s power_meter_can1_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = POWER_METER_TX_ID,
            .rx_id = POWER_METER_RX_ID,
        },
        .daemon_count = POWER_METER_DAEMON_COUNT,
        .timeout_ms = POWER_METER_TIMEOUT_MS,
    };
    power_meter_can1 = PowerMeterInit(&power_meter_can1_conf);
#endif
#if POWER_METER_USE_CAN2
    PowerMeter_Init_Config_s power_meter_can2_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = POWER_METER_TX_ID,
            .rx_id = POWER_METER_RX_ID,
        },
        .daemon_count = POWER_METER_DAEMON_COUNT,
        .timeout_ms = POWER_METER_TIMEOUT_MS,
    };
    power_meter_can2 = PowerMeterInit(&power_meter_can2_conf);
#endif
#endif
#if STEERING_DRIVE
    ChassisPower_InitSteering(steer_lf, steer_rf, steer_lb, steer_rb);
    ChassisPower_SetSteeringPowerRatio(STEERING_POWER_RATIO);
#endif

    // 初始化单电机功率控制：motor_lf 槽位0
    SingleMotorPower_Init(0);
    
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    InfantryMecanumCalculate(chassis_vx,
                             chassis_vy,
                             chassis_cmd_recv.wz,
                             LF_CENTER,
                             RF_CENTER,
                             LB_CENTER,
                             RB_CENTER,
                             &vt_lf,
                             &vt_rf,
                             &vt_lb,
                             &vt_rb);
}

#if OMNI_DRIVE
#define COS45 0.70710678f
// 全向轮 X 型排列: 四个轮子距中心等距 (用 LF_CENTER 近似, 四个 CENTER 宏值相等时成立)
#define OMNI_L (LF_CENTER)

/**
 * @brief 四轮全向轮运动学解算 (X 型排列, 轮子互成 90°)
 *
 * 轮子布局:  RF=45°, LF=135°, LB=225°, RB=315°
 * 公式: v_i = vx*cos(θ) + vy*sin(θ) - wz*L
 */
static void OmniCalculate()
{
    vt_rf =  chassis_vx * COS45 + chassis_vy * COS45 - chassis_cmd_recv.wz * OMNI_L;
    vt_lf = -chassis_vx * COS45 + chassis_vy * COS45 - chassis_cmd_recv.wz * OMNI_L;
    vt_lb = -chassis_vx * COS45 - chassis_vy * COS45 - chassis_cmd_recv.wz * OMNI_L;
    vt_rb =  chassis_vx * COS45 - chassis_vy * COS45 - chassis_cmd_recv.wz * OMNI_L;
}
#endif

#if STEERING_DRIVE
static float Wrap180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static float SteeringTarget(float current_total_angle, float target_angle, float *drive_speed)
{
    float current_single_angle = Wrap180(current_total_angle);
    float target_single_angle = Wrap180(target_angle);
    float error = Wrap180(target_single_angle - current_single_angle);

    if (error > 90.0f)
    {
        error -= 180.0f;
        *drive_speed = -*drive_speed;
    }
    else if (error < -90.0f)
    {
        error += 180.0f;
        *drive_speed = -*drive_speed;
    }

    return current_total_angle + error;
}

static void SteeringWheelCalculate(float wheel_x, float wheel_y, float zero_angle,
                                   DJIMotorInstance *steer_motor,
                                   float *drive_speed, float *steer_target)
{
    const float wz_rad = chassis_cmd_recv.wz * DEGREE_2_RAD;
    const float wheel_vx = chassis_vx - wz_rad * wheel_y;
    const float wheel_vy = chassis_vy + wz_rad * wheel_x;

    *drive_speed = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
    float target_angle = atan2f(wheel_vy, wheel_vx) * RAD_2_DEGREE + zero_angle;
    *steer_target = SteeringTarget(steer_motor->measure.total_angle, target_angle, drive_speed);
}

static void SteeringCalculate()
{
    const float front_x = HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y;
    const float rear_x = -HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y;
    const float left_y = -HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X;
    const float right_y = HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X;
    float steer_target_lf, steer_target_rf, steer_target_lb, steer_target_rb;

    SteeringWheelCalculate(front_x, left_y, STEERING_MOTOR_LF_ZERO_DEG,
                           steer_lf, &vt_lf, &steer_target_lf);
    SteeringWheelCalculate(front_x, right_y, STEERING_MOTOR_RF_ZERO_DEG,
                           steer_rf, &vt_rf, &steer_target_rf);
    SteeringWheelCalculate(rear_x, left_y, STEERING_MOTOR_LB_ZERO_DEG,
                           steer_lb, &vt_lb, &steer_target_lb);
    SteeringWheelCalculate(rear_x, right_y, STEERING_MOTOR_RB_ZERO_DEG,
                           steer_rb, &vt_rb, &steer_target_rb);

    DJIMotorSetRef(steer_lf, steer_target_lf);
    DJIMotorSetRef(steer_rf, steer_target_rf);
    DJIMotorSetRef(steer_lb, steer_target_lb);
    DJIMotorSetRef(steer_rb, steer_target_rb);
}
#endif

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    
   
#ifdef ONE_BOARD
SubGetMessage(chassis_sub, &chassis_cmd_recv);                     
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    PowerMeter_Data_s power_meter_data = {0};
    PowerMeter_Snapshot_s meter_sample = {0};
    ChassisPowerSource meter_source = CHASSIS_POWER_SOURCE_NONE;
    uint8_t power_meter_online = 0;
#if POWER_METER_UART_PRINT_ENABLE
    PowerMeterInstance *active_power_meter = NULL;
#endif
#if POWER_METER_ENABLE
#if POWER_METER_USE_CAN1
    meter_sample = PowerMeterGetSnapshot(power_meter_can1);
    if (meter_sample.online && meter_sample.data.bus_voltage_v > 0.0f)
    {
        power_meter_online = 1;
        power_meter_data = meter_sample.data;
        meter_source = CHASSIS_POWER_SOURCE_METER_CAN1;
#if POWER_METER_UART_PRINT_ENABLE
        active_power_meter = power_meter_can1;
#endif
    }
#endif
#if POWER_METER_USE_CAN2
    if (!power_meter_online)
        meter_sample = PowerMeterGetSnapshot(power_meter_can2);
    if (!power_meter_online && meter_sample.online && meter_sample.data.bus_voltage_v > 0.0f)
    {
        power_meter_online = 1;
        power_meter_data = meter_sample.data;
        meter_source = CHASSIS_POWER_SOURCE_METER_CAN2;
#if POWER_METER_UART_PRINT_ENABLE
        active_power_meter = power_meter_can2;
#endif
    }
#endif
#endif

    if (power_meter_online)
        PowerMeterApplyCalibration(&power_meter_data);
    if (!isfinite(power_meter_data.power_w))
        power_meter_online = 0;

    const uint8_t drive_ready_states[4] = {
        DJIMotorIsOnline(motor_lf),
        DJIMotorIsOnline(motor_rf),
        DJIMotorIsOnline(motor_lb),
        DJIMotorIsOnline(motor_rb),
    };
    uint8_t chassis_hardware_ready = InfantryAllReady(drive_ready_states, 4);
#if STEERING_DRIVE
    const uint8_t steering_ready_states[4] = {
        DJIMotorIsOnline(steer_lf),
        DJIMotorIsOnline(steer_rf),
        DJIMotorIsOnline(steer_lb),
        DJIMotorIsOnline(steer_rb),
    };
    chassis_hardware_ready = chassis_hardware_ready &&
        InfantryAllReady(steering_ready_states, 4);
#endif
    if (!chassis_hardware_ready)
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;

#if CHASSIS_WHEEL_TEST_MODE == CHASSIS_WHEEL_TEST_ENABLE
    const uint8_t wheel_test_allowed = InfantryWheelTestAllowed(
        (uint8_t)chassis_cmd_recv.chassis_mode);
    if (wheel_test_allowed)
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_recv.vx = 0.0f;
        chassis_cmd_recv.vy = 0.0f;
        chassis_cmd_recv.wz = 0.0f;
        chassis_cmd_recv.offset_angle = 0.0f;
    }
#endif

    const RefereePowerSnapshot referee_power = RefereeGetPowerSnapshot();
    const uint32_t power_now_ms = HAL_GetTick();
    const uint8_t referee_power_fresh = referee_power.power_received &&
        (uint32_t)(power_now_ms - referee_power.power_sample_ms) < REFEREE_POWER_TIMEOUT_MS;
    // Only fresh, legal limits replace the last setting. A lost referee must not
    // release a previous lower limit; startup retains the manager's default 80W.
    if (referee_power.limit_received &&
        (uint32_t)(power_now_ms - referee_power.limit_sample_ms) < REFEREE_POWER_LIMIT_TIMEOUT_MS &&
        isfinite(referee_power.power_limit_w) && referee_power.power_limit_w > 0.0f &&
        referee_power.power_limit_w <= 200.0f)
        ChassisPower_SetLimit(referee_power.power_limit_w);

    ChassisPowerFeedback power_feedback = {0};
    power_feedback.buffer_valid = referee_power_fresh;
    power_feedback.buffer_energy_j = referee_power.buffer_energy_j;
    power_feedback.buffer_sequence = referee_power.power_sequence;
    power_feedback.buffer_sample_ms = referee_power.power_sample_ms;
    power_feedback.buffer_timeout_ms = REFEREE_POWER_TIMEOUT_MS;
    if (power_meter_online)
    {
        power_feedback.power_w = power_meter_data.power_w;
        power_feedback.power_source = meter_source;
        power_feedback.power_sequence = meter_sample.sequence;
        power_feedback.power_sample_ms = meter_sample.sample_ms;
        power_feedback.power_timeout_ms = POWER_METER_TIMEOUT_MS;
    }
    else if (referee_power_fresh && isfinite(referee_power.power_w))
    {
        power_feedback.power_w = referee_power.power_w;
        power_feedback.power_source = CHASSIS_POWER_SOURCE_REFEREE;
        power_feedback.power_sequence = referee_power.power_sequence;
        power_feedback.power_sample_ms = referee_power.power_sample_ms;
        power_feedback.power_timeout_ms = REFEREE_POWER_TIMEOUT_MS;
    }
    ChassisPower_SetFeedback(&power_feedback);
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
#if STEERING_DRIVE
        DJIMotorStop(steer_lf);
        DJIMotorStop(steer_rf);
        DJIMotorStop(steer_lb);
        DJIMotorStop(steer_rb);
#endif
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
#if STEERING_DRIVE
        DJIMotorEnable(steer_lf);
        DJIMotorEnable(steer_rf);
        DJIMotorEnable(steer_lb);
        DJIMotorEnable(steer_rb);
#endif
    }

    // 根据有效 offset_angle 的连续变化率估算底盘实际角速度，供云台 yaw 前馈。
    float actual_chassis_wz = 0.0f;
    {
        static float last_wz_offset = 0, last_wz_time = 0;
        static uint8_t rate_initialized = 0;
        float now = DWT_GetTimeline_ms();
        const uint8_t rate_needed =
            chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW ||
            chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE;
        if (!rate_needed)
        {
            rate_initialized = 0;
            chassis_wz_for_gimbal = 0.0f;
        }
        else if (!rate_initialized)
        {
            last_wz_offset = chassis_cmd_recv.offset_angle;
            last_wz_time = now;
            rate_initialized = 1;
            chassis_wz_for_gimbal = 0.0f;
        }
        else
        {
            const float dt = now - last_wz_time;
            if (dt > 1.0f)
            {
                float delta = chassis_cmd_recv.offset_angle - last_wz_offset;
                if (delta > 180.0f)  delta -= 360.0f;
                if (delta < -180.0f) delta += 360.0f;
                actual_chassis_wz = delta / dt * 1000.0f;
                chassis_wz_for_gimbal = actual_chassis_wz;
                last_wz_offset = chassis_cmd_recv.offset_angle;
                last_wz_time = now;
            }
        }
    }

    // 根据控制模式设定旋转速度
    static float rotate_last_time = 0.0f;
    static float rotate_iout = 0.0f;
    static uint8_t rotate_initialized = 0;
    if (chassis_cmd_recv.chassis_mode != CHASSIS_ROTATE)
    {
        rotate_last_time = 0.0f;
        rotate_iout = 0.0f;
        rotate_initialized = 0;
    }
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * fabsf(chassis_cmd_recv.offset_angle);
         LIMIT_MIN_MAX(chassis_cmd_recv.wz, -800, 800);  // 限制最大旋转速度
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;用 offset_angle 变化率做旋转闭环
    {
        float now = DWT_GetTimeline_ms();
        float dt = now - rotate_last_time;
        if (!rotate_initialized)
        {
            rotate_iout = 0.0f;
            dt = 0.0f;
            rotate_initialized = 1;
        }

        const float error = CHASSIS_ROTATE_TARGET_WZ - actual_chassis_wz;
        float kp = 3.0f, ki = 0.3f;
        rotate_iout += error * dt * ki * 0.001f;
        LIMIT_MIN_MAX(rotate_iout, -500, 500);

        chassis_cmd_recv.wz = CHASSIS_ROTATE_TARGET_WZ + kp * error + rotate_iout;
        LIMIT_MIN_MAX(chassis_cmd_recv.wz,
                      -CHASSIS_ROTATE_OUTPUT_MAX_WZ,
                      CHASSIS_ROTATE_OUTPUT_MAX_WZ);

        rotate_last_time = now;
        break;
    }
    default:
        chassis_cmd_recv.vx = 0.0f;
        chassis_cmd_recv.vy = 0.0f;
        chassis_cmd_recv.wz = 0.0f;
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
#if STEERING_DRIVE
        DJIMotorStop(steer_lf);
        DJIMotorStop(steer_rf);
        DJIMotorStop(steer_lb);
        DJIMotorStop(steer_rb);
#endif
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    float motion_angle = CHASSIS_CMD_DIR_TRIM_DEG * DEGREE_2_RAD;
    if (chassis_cmd_recv.chassis_mode != CHASSIS_NO_FOLLOW)
    {
        motion_angle = (chassis_cmd_recv.offset_angle + CHASSIS_CMD_DIR_TRIM_DEG) * DEGREE_2_RAD;
    }
    /*当底盘需要跟随云台时，用云台与底盘的夹角来旋转速度向量，实现“前进方向始终以云台指向为准”；
    当底盘不跟随时，则不叠加 offset 角度，直接用默认方向角进行解算。*/
    cos_theta = arm_cos_f32(motion_angle);
    sin_theta = arm_sin_f32(motion_angle);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
#if STEERING_DRIVE
    SteeringCalculate();
#elif OMNI_DRIVE
    OmniCalculate();
#else
    MecanumCalculate();
#endif

#if CHASSIS_WHEEL_TEST_MODE == CHASSIS_WHEEL_TEST_ENABLE
    vt_lf = 0.0f;
    vt_rf = 0.0f;
    vt_lb = 0.0f;
    vt_rb = 0.0f;
#if CHASSIS_WHEEL_TEST_TARGET == CHASSIS_WHEEL_TEST_LF
    if (wheel_test_allowed) vt_lf = CHASSIS_WHEEL_TEST_SPEED;
#elif CHASSIS_WHEEL_TEST_TARGET == CHASSIS_WHEEL_TEST_RF
    if (wheel_test_allowed) vt_rf = CHASSIS_WHEEL_TEST_SPEED;
#elif CHASSIS_WHEEL_TEST_TARGET == CHASSIS_WHEEL_TEST_LB
    if (wheel_test_allowed) vt_lb = CHASSIS_WHEEL_TEST_SPEED;
#elif CHASSIS_WHEEL_TEST_TARGET == CHASSIS_WHEEL_TEST_RB
    if (wheel_test_allowed) vt_rb = CHASSIS_WHEEL_TEST_SPEED;
#endif
#endif

    float wheel_speeds[4] = {vt_lf, vt_rf, vt_lb, vt_rb};
    InfantryLimitWheelSpeeds(wheel_speeds, CHASSIS_MAX_WHEEL_SPEED_APS);
    vt_lf = wheel_speeds[0];
    vt_rf = wheel_speeds[1];
    vt_lb = wheel_speeds[2];
    vt_rb = wheel_speeds[3];

   
    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 更新功率管理器的各电机误差（速度设定值 - 反馈值）
    // 误差越大 → 分配的功率预算越多
    ChassisPower_UpdateError(0, vt_lf - motor_lf->measure.speed_aps);
    ChassisPower_UpdateError(1, vt_rf - motor_rf->measure.speed_aps);
    ChassisPower_UpdateError(2, vt_lb - motor_lb->measure.speed_aps);
    ChassisPower_UpdateError(3, vt_rb - motor_rb->measure.speed_aps);

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD

    // 更新底盘功率信息结构体（调试用）
    const ChassisPowerFeedbackStatus feedback_status = ChassisPower_GetFeedbackStatus();
    chassis_power_info.raw_predict_power_w = ChassisPower_GetPredictPower();
    chassis_power_info.predict_power_w = ChassisPower_GetCorrectedPredictPower();
    chassis_power_info.actual_power_w  = ChassisPower_GetMeasuredPower();
    chassis_power_info.raw_model_feedback_power_w = ChassisPower_GetFeedbackPower();
    chassis_power_info.model_feedback_power_w = ChassisPower_GetCorrectedFeedbackPower();
    chassis_power_info.power_limit_w   = ChassisPower_GetLimit();
    chassis_power_info.model_correction = ChassisPower_GetModelCorrection();
    chassis_power_info.measured_power_w = ChassisPower_GetMeasuredPower();
    chassis_power_info.buffer_energy_j = ChassisPower_GetBufferEnergy();
    chassis_power_info.buffer_attenuation = ChassisPower_GetBufferAttenuation();
    chassis_power_info.effective_power_limit_w = ChassisPower_GetEffectiveLimit();
    chassis_power_info.power_meter_online = power_meter_online;
    chassis_power_info.meter_bus_voltage_v = power_meter_data.bus_voltage_v;
    chassis_power_info.meter_current_a = power_meter_data.current_a;
    chassis_power_info.meter_power_w = power_meter_data.power_w;
    chassis_power_info.feedback_source = (uint8_t)feedback_status.power_source;
    chassis_power_info.feedback_valid = feedback_status.power_valid;
    chassis_power_info.buffer_feedback_valid = feedback_status.buffer_valid;
    chassis_power_info.feedback_age_ms = feedback_status.power_age_ms;
    for (int i = 0; i < 4; i++)
    {
        chassis_power_info.slip_score[i] = ChassisPower_GetSlipScore(i);
        chassis_power_info.dynamic_weight[i] = ChassisPower_GetDynamicWeight(i);
    }

    // RTT log: motor speed, power-meter current and power.
    {
        static int log_cnt = 0;
        if (++log_cnt >= POWER_METER_DEBUG_PRINT_PERIOD)
        {
            log_cnt = 0;
            char spd[16], cur[16], pwr[16], bus[16];
            Float2Str(spd, motor_lf->measure.speed_aps);
            Float2Str(cur, power_meter_data.current_a);
            Float2Str(pwr, power_meter_data.power_w);
            Float2Str(bus, power_meter_data.bus_voltage_v);
            LOG("pm online=%d  bus=%s V  current=%s A  power=%s W  LF speed=%s deg/s",
                power_meter_online, bus, cur, pwr, spd);
#if POWER_METER_UART_PRINT_ENABLE
            if (power_meter_online && active_power_meter != NULL)
                PowerMeterPrintCanFrameUART(active_power_meter, POWER_METER_UART_HANDLE);
            else
                CANPrintDebugUART(POWER_METER_DEBUG_CAN_HANDLE, POWER_METER_UART_HANDLE);
#endif
        }
    }

    {
        static int uart2_motor_log_cnt = 0;
        if (++uart2_motor_log_cnt >= MOTOR_UART2_DEBUG_PRINT_PERIOD)
        {
            uart2_motor_log_cnt = 0;
            ChassisPrintMotorFeedbackUART2();
        }
    }
}
