#include "robot_cmd.h"

#include "bsp_dwt.h"
#include "bmi088.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "remote_control.h"
#include "robot_def.h"

#ifdef GIMBAL_BOARD
#include "can_comm.h"
#endif

#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
#define PITCH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)

#define RC_DEADBAND_VALUE 30
#define RC_CHASSIS_SCALE 10.0f
#define RC_GIMBAL_YAW_SCALE 0.0003f
#define RC_GIMBAL_PITCH_SCALE 0.0003f

#define KEY_CHASSIS_SPEED_SLOW 1500.0f
#define KEY_CHASSIS_SPEED_NORMAL 3000.0f
#define KEY_CHASSIS_SPEED_FAST 6000.0f
#define MOUSE_GIMBAL_YAW_SCALE 0.003f
#define MOUSE_GIMBAL_PITCH_SCALE 0.003f

#define DEFAULT_CHASSIS_SPEED_BUFF 60
#define DEFAULT_SHOOT_RATE 1.0f

#ifdef GIMBAL_BOARD
static CANCommInstance *cmd_can_comm;
#endif

#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;
static Subscriber_t *chassis_feed_sub;
#endif
static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;

RC_ctrl_t *rc_data;
static Vision_Recv_s *vision_recv_data;

static Publisher_t *gimbal_cmd_pub;
static Subscriber_t *gimbal_feed_sub;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;

static Publisher_t *shoot_cmd_pub;
static Subscriber_t *shoot_feed_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_send;
static Shoot_Upload_Data_s shoot_fetch_data;

static Robot_Status_e robot_state;
static Robot_Control_Mode_e robot_control_mode;

BMI088Instance *bmi088_test;
BMI088_Data_t bmi088_data;

static int16_t Deadband(int16_t value)
{
    if (value > -RC_DEADBAND_VALUE && value < RC_DEADBAND_VALUE)
        return 0;
    return value;
}

static Robot_Control_Mode_e SelectControlMode(void)
{
    if (rc_data == NULL)
        return ROBOT_CONTROL_REMOTE;

    if (switch_is_up(rc_data[TEMP].rc.switch_left))
        return ROBOT_CONTROL_MOUSE_KEY;
    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
        return ROBOT_CONTROL_VISION_AUTO;
    return ROBOT_CONTROL_REMOTE;
}

static void CalcOffsetAngle(void)
{
    float offset_angle = (float)gimbal_fetch_data.yaw_motor_single_round_angle - YAW_ALIGN_ANGLE;

    while (offset_angle > 180.0f)
        offset_angle -= 360.0f;
    while (offset_angle < -180.0f)
        offset_angle += 360.0f;

    chassis_cmd_send.offset_angle = offset_angle;
}

static void ResetCommandDefaults(void)
{
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    chassis_cmd_send.chassis_speed_buff = DEFAULT_CHASSIS_SPEED_BUFF;

    gimbal_cmd_send.chassis_rotate_wz = 0.0f;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.lid_mode = LID_CLOSE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.bullet_speed = chassis_fetch_data.bullet_speed;
    if (shoot_cmd_send.bullet_speed == BULLET_SPEED_NONE)
        shoot_cmd_send.bullet_speed = SMALL_AMU_15;
    shoot_cmd_send.rest_heat = chassis_fetch_data.rest_heat;
    shoot_cmd_send.shoot_rate = DEFAULT_SHOOT_RATE;
}

static void StopAllCommand(void)
{
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.chassis_speed_buff = 0;

    gimbal_cmd_send.chassis_rotate_wz = 0.0f;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.lid_mode = LID_CLOSE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.shoot_rate = 0.0f;
}

static void SetChassisFromRemote(void)
{
    chassis_cmd_send.vx = RC_CHASSIS_SCALE * (float)Deadband(rc_data[TEMP].rc.rocker_r1);
    chassis_cmd_send.vy = RC_CHASSIS_SCALE * (float)Deadband(rc_data[TEMP].rc.rocker_r_);
}

static void SetGimbalFromRemote(void)
{
    gimbal_cmd_send.yaw += RC_GIMBAL_YAW_SCALE * (float)Deadband(rc_data[TEMP].rc.rocker_l_);
    gimbal_cmd_send.pitch += RC_GIMBAL_PITCH_SCALE * (float)Deadband(rc_data[TEMP].rc.rocker_l1);
}

static void SetChassisFromKeyboard(void)
{
    float speed = KEY_CHASSIS_SPEED_NORMAL;

    if (rc_data[TEMP].key[KEY_STATE].shift)
        speed = KEY_CHASSIS_SPEED_FAST;
    else if (rc_data[TEMP].key[KEY_STATE].ctrl)
        speed = KEY_CHASSIS_SPEED_SLOW;

    if (rc_data[TEMP].key[KEY_STATE].w)
        chassis_cmd_send.vx += speed;
    if (rc_data[TEMP].key[KEY_STATE].s)
        chassis_cmd_send.vx -= speed;
    if (rc_data[TEMP].key[KEY_STATE].a)
        chassis_cmd_send.vy += speed;
    if (rc_data[TEMP].key[KEY_STATE].d)
        chassis_cmd_send.vy -= speed;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4)
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }

    if (rc_data[TEMP].key[KEY_STATE].q)
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
}

static void SetGimbalFromMouse(void)
{
    gimbal_cmd_send.yaw += MOUSE_GIMBAL_YAW_SCALE * (float)rc_data[TEMP].mouse.x;
    gimbal_cmd_send.pitch -= MOUSE_GIMBAL_PITCH_SCALE * (float)rc_data[TEMP].mouse.y;
}

static void SetShootFromMouseKey(void)
{
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode =
        (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) ? FRICTION_ON : FRICTION_OFF;
    shoot_cmd_send.lid_mode =
        (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) ? LID_OPEN : LID_CLOSE;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3)
    {
    case 0:
        shoot_cmd_send.bullet_speed = SMALL_AMU_15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = SMALL_AMU_18;
        break;
    default:
        shoot_cmd_send.bullet_speed = SMALL_AMU_30;
        break;
    }

    if (shoot_cmd_send.friction_mode == FRICTION_OFF)
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    if (rc_data[TEMP].mouse.press_l)
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    else if (rc_data[TEMP].mouse.press_r)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
    {
        switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4)
        {
        case 1:
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
            break;
        case 2:
            shoot_cmd_send.load_mode = LOAD_3_BULLET;
            break;
        case 3:
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            break;
        default:
            shoot_cmd_send.load_mode = LOAD_STOP;
            break;
        }
    }
}

static void RemoteControlSet(void)  //遥控器开关控制
{
    SetChassisFromRemote();
    SetGimbalFromRemote();

    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        shoot_cmd_send.lid_mode = LID_OPEN;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }

    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = (rc_data[TEMP].rc.dial < -100) ? FRICTION_ON : FRICTION_OFF;
    if (shoot_cmd_send.friction_mode == FRICTION_ON && rc_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
}

static void MouseKeySet(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    SetChassisFromKeyboard();
    SetGimbalFromMouse();
    SetShootFromMouseKey();
}

static void VisionAutoSet(void)
{
    uint8_t has_target = 0;

    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    SetChassisFromRemote();

    if (vision_recv_data != NULL && vision_recv_data->target_state != NO_TARGET)
    {
        has_target = 1;
        gimbal_cmd_send.yaw += vision_recv_data->yaw;
        gimbal_cmd_send.pitch += vision_recv_data->pitch;
    }
    else
    {
        SetGimbalFromRemote();
    }

    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = has_target ? FRICTION_ON : FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;

    if (has_target &&
        vision_recv_data->target_state == READY_TO_FIRE &&
        vision_recv_data->fire_mode == AUTO_FIRE)
    {
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    }

    VisionSetFlag(chassis_fetch_data.enemy_color, VISION_MODE_AIM, shoot_cmd_send.bullet_speed);
    VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.Yaw,
                      gimbal_fetch_data.gimbal_imu_data.Pitch,
                      gimbal_fetch_data.gimbal_imu_data.Roll);
}

static void EmergencyHandler(void)
{
    if (rc_data == NULL || !RemoteControlIsOnline())
    {
        robot_state = ROBOT_STOP;
        StopAllCommand();
        return;
    }

    if (rc_data[TEMP].rc.dial > 300)
        robot_state = ROBOT_STOP;
    else if (rc_data[TEMP].rc.dial < 100)
        robot_state = ROBOT_READY;

#ifdef GIMBAL_BOARD
    if (cmd_can_comm != NULL && !CANCommIsOnline(cmd_can_comm))
        robot_state = ROBOT_STOP;
#endif

    if (robot_state == ROBOT_STOP)
        StopAllCommand();
}

void RobotCMDInit(void)
{
    rc_data = RemoteControlInit(&huart3);
    vision_recv_data = VisionInit(&huart1);

#ifdef ONE_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif

#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    robot_state = ROBOT_READY;
    robot_control_mode = ROBOT_CONTROL_REMOTE;

    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    gimbal_cmd_send.yaw = 0.0f;
    gimbal_cmd_send.pitch = 0.0f;
    gimbal_cmd_send.chassis_rotate_wz = 0.0f;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.lid_mode = LID_CLOSE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.bullet_speed = SMALL_AMU_15;
    shoot_cmd_send.shoot_rate = 0.0f;

    (void)PITCH_HORIZON_ANGLE;
}

void RobotCMDTask(void)
{
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
#endif
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);

    CalcOffsetAngle();
    robot_control_mode = SelectControlMode();
    ResetCommandDefaults();

    switch (robot_control_mode)
    {
    case ROBOT_CONTROL_MOUSE_KEY:
        MouseKeySet();
        break;
    case ROBOT_CONTROL_VISION_AUTO:
        VisionAutoSet();
        break;
    case ROBOT_CONTROL_REMOTE:
    default:
        RemoteControlSet();
        break;
    }

    EmergencyHandler();

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (uint8_t *)&chassis_cmd_send);
#endif
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
}
