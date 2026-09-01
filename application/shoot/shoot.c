#include "shoot.h"
#include "robot_def.h"
#include "jam_detector.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "infantry_control.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static JamDetectorInstance *jam_detector;
// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
static float friction_ready_since = 0;
static InfantryLoaderRequest loader_request;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = FRICTION_MOTOR_CAN_HANDLE,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = FRICTION_MOTOR_LEFT_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = FRICTION_MOTOR_TYPE};
    friction_config.can_init_config.tx_id = FRICTION_MOTOR_LEFT_ID;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = FRICTION_MOTOR_RIGHT_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag =
        FRICTION_MOTOR_RIGHT_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = LOADER_MOTOR_CAN_HANDLE,
            .tx_id = LOADER_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 4.5, // 10
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = ANGLE_LOOP | CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = LOADER_MOTOR_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LOADER_MOTOR_TYPE
    };
    loader = DJIMotorInit(&loader_config);
    // 初始化阶段保持零力矩，等待robot_cmd发布有效且非急停的指令。
    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    DJIMotorStop(loader);
    InfantryLoaderRequestReset(&loader_request, LOAD_STOP);

    // 卡弹检测器初始化（必须在 loader 初始化之后）
#if 0  // 卡弹检测：改为 0 关闭
    JamDetector_Init_Config_s jam_detector_config = {
        .current_threshold = 1000,
        .suspect_timeout_ms = 300,
        .handling_timeout_ms = 400,
        .reverse_speed = 1500,
    };
    jam_detector = JamDetectorInit(&jam_detector_config, loader);
#endif

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    const uint8_t shooter_ready_states[3] = {
        DJIMotorIsOnline(friction_l),
        DJIMotorIsOnline(friction_r),
        DJIMotorIsOnline(loader),
    };
    const uint8_t shooter_hardware_ready = InfantryAllReady(shooter_ready_states, 3);

    const float friction_target = shooter_hardware_ready &&
                                          shoot_cmd_recv.shoot_mode == SHOOT_ON &&
                                          shoot_cmd_recv.friction_mode == FRICTION_ON
        ? InfantryFrictionTarget((uint8_t)shoot_cmd_recv.bullet_speed,
                                 FRICTION_SPEED_15_APS,
                                 FRICTION_SPEED_18_APS,
                                 FRICTION_SPEED_30_APS)
        : 0.0f;

    if (friction_target > 0.0f)
    {
        DJIMotorSetRef(friction_l, friction_target);
        DJIMotorSetRef(friction_r, friction_target);
    }
    else
    {
        DJIMotorSetRef(friction_l, 0.0f);
        DJIMotorSetRef(friction_r, 0.0f);
    }

    const float now = DWT_GetTimeline_ms();
    const uint8_t friction_at_speed = InfantryFrictionReady(
        friction_target,
        friction_l->measure.speed_aps,
        friction_r->measure.speed_aps,
        FRICTION_READY_TOLERANCE_APS);
    if (!friction_at_speed)
    {
        friction_ready_since = 0.0f;
        shoot_feedback_data.friction_ready = 0;
    }
    else
    {
        if (friction_ready_since <= 0.0f)
            friction_ready_since = now;
        shoot_feedback_data.friction_ready =
            (now - friction_ready_since) >= FRICTION_READY_HOLD_MS;
    }

    // ====== 测试模式：无遥控器时强制拨盘旋转 ======
    // 正式上场前把 1 改为 0 或删除此段
#if 0
    shoot_cmd_recv.shoot_mode = SHOOT_ON;
    shoot_cmd_recv.load_mode = LOAD_BURSTFIRE;
    shoot_cmd_recv.shoot_rate = 1;               // 1发/秒, 慢速安全
    shoot_cmd_recv.friction_mode = FRICTION_OFF;  // 摩擦轮关闭
#endif
    // =============================================

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode != SHOOT_ON || !shooter_hardware_ready)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        shoot_feedback_data.friction_ready = 0;
        friction_ready_since = 0.0f;
        InfantryLoaderRequestSync(&loader_request,
                                  (uint8_t)shoot_cmd_recv.load_mode,
                                  LOAD_STOP);
#if 0  // 卡弹检测
        JamDetectorReset(jam_detector);
#endif
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 卡弹检测（必须在 loader 控制之前调用）
#if 0  // 卡弹检测：改为 0 关闭
    JamState_e jam_state = JamDetectorTask(jam_detector);

    // 非卡弹处理状态下，正常运行拨盘电机控制
    if (jam_state != JAM_CONFIRMED && jam_state != JAM_HANDLING)
#endif
    {
        // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
        // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
        // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        //     return;

        // 根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
        const uint8_t loader_control_allowed =
            shoot_cmd_recv.shoot_mode == SHOOT_ON && shooter_hardware_ready;
        const loader_mode_e safe_requested_load_mode = (loader_mode_e)InfantrySafeLoadMode(
            (uint8_t)shoot_cmd_recv.load_mode,
            shoot_cmd_recv.shoot_mode == SHOOT_ON,
            shooter_hardware_ready,
            LOAD_STOP);
        const loader_mode_e effective_load_mode = loader_control_allowed
            ? (loader_mode_e)InfantryLoaderSelectMode(
                  &loader_request,
                  (uint8_t)safe_requested_load_mode,
                  shoot_feedback_data.friction_ready,
                  shoot_feedback_data.friction_ready && now - hibernate_time >= dead_time,
                  LOAD_STOP)
            : LOAD_STOP;

        switch (effective_load_mode)
        {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
        case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, loader->measure.total_angle +
                                      InfantryLoaderProjectileAngle(
                                          1,
                                          REDUCTION_RATIO_LOADER,
                                          NUM_PER_CIRCLE));
            hibernate_time = now;
            dead_time = LOADER_SINGLE_SHOT_DEAD_TIME_MS;
            break;
        // 三连发,如果不需要后续可能删除
        case LOAD_3_BULLET:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, loader->measure.total_angle +
                                      InfantryLoaderProjectileAngle(
                                          3,
                                          REDUCTION_RATIO_LOADER,
                                          NUM_PER_CIRCLE));
            hibernate_time = now;
            dead_time = 300.0f;
            break;
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, InfantryLoaderBurstSpeed(
                                       shoot_cmd_recv.shoot_rate,
                                       REDUCTION_RATIO_LOADER,
                                       NUM_PER_CIRCLE));
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
        // 也有可能需要从switch-case中独立出来
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            // ...
            break;
        default:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
            break;
        }
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
