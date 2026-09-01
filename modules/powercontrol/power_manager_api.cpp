/**
 * @file power_manager_api.cpp
 * @brief C++桥接实现 — 将power_manager的MotorPower/ChassisPowerManager适配到C框架
 *
 * M3508电机模型参数来自Fitting.py拟合,原始电流换算比例 = 16384/20 = 819.2
 * 如果你对M3508重新拟合了数据,请更新下面m3508_fitting中的6个k值
 */

#include "power_manager_api.h"
#include "power_manager.h"
#include "FreeRTOS.h"           // pvPortMalloc / vPortFree
#include <cmath>
#include "task.h"

// ==================== M3508 电机功率模型拟合参数 ====================
// 模型: P = k0 + k1*abs(I) + k2*abs(W) + k3*I*W + k4*I² + k5*W²  (I=A, W=RPM)
static const motor_power_init_t m3508_fitting = {
    .k0 = -0.270186,        // 静态偏置(电调+电机待机)
    .k1 = 0.171115,         // abs(I) 电流线性损耗项
    .k2 = 0.001479,         // abs(W) 转速线性损耗项
    .k3 = 0.002097,         // signed I*W 机械输出功率
    .k4 = 0.180328,         // I² 铜损（绕组发热）
    .k5 = -0.000000,           // W² 铁损+摩擦(≈0)
    .real_current_conversion = 819.2  // M3508: 16384映射20A
};

// 其他DJI电机暂复用同形模型，仅替换电流换算系数；精确控制前建议重新拟合对应电机。
static const motor_power_init_t m2006_fitting = {
    .k0 = -0.270186,
    .k1 = 0.171115,
    .k2 = 0.001479,
    .k3 = 0.002097,
    .k4 = 0.180328,
    .k5 = -0.000000,
    .real_current_conversion = 1000.0
};

static const motor_power_init_t gm6020_fitting = {
    .k0 = -0.270186,
    .k1 = 0.171115,
    .k2 = 0.001479,
    .k3 = 0.002097,
    .k4 = 0.180328,
    .k5 = -0.000000,
    .real_current_conversion = 1500.0
};

static const float CHASSIS_POWER_DEFAULT_LIMIT_W = 80.0f;
static const float CHASSIS_POWER_MAX_LIMIT_W = 200.0f;
static const float SLIP_SPEED_MIN_APS = 120.0f;
static const float SLIP_ERROR_RATIO_ON = 0.55f;
static const float SLIP_POWER_MIN_W = 10.0f;
static const float SLIP_CURRENT_MIN_A = 3.0f;
static const float SLIP_SCORE_ATTACK = 0.05f;
static const float SLIP_SCORE_RELEASE = 0.01f;
static const float SLIP_MIN_POWER_WEIGHT = 0.35f;
static const double DYNAMIC_POWER_ERROR_RATIO = 0.65;
static const double DYNAMIC_POWER_PREDICT_RATIO = 0.35;

static const motor_power_init_t &GetMotorPowerInit(Motor_Type_e motor_type)
{
    switch (motor_type)
    {
    case M2006:
        return m2006_fitting;
    case GM6020:
        return gm6020_fitting;
    case M3508:
    default:
        return m3508_fitting;
    }
}

static float GetMotorCurrentConversion(Motor_Type_e motor_type)
{
    switch (motor_type)
    {
    case M2006:
        return 1000.0f;
    case GM6020:
        return 1500.0f;
    case M3508:
    default:
        return 819.2f;
    }
}

static float GetMotorOutputLimit(Motor_Type_e motor_type)
{
    switch (motor_type)
    {
    case M2006:
        return 10000.0f;
    case GM6020:
        return 30000.0f;
    case M3508:
    default:
        return 16384.0f;
    }
}

static float ClampOutputCurrent(double desired_current, Motor_Type_e motor_type)
{
    if (!std::isfinite(desired_current))
        return 0.0f;

    const float limit = GetMotorOutputLimit(motor_type);
    if (desired_current > limit)
        return limit;
    if (desired_current < -limit)
        return -limit;
    return (float)desired_current;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (!std::isfinite(value))
        return min_value;
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

// ==================== 全局实例 ====================
static MotorPower *motor_power[4] = {nullptr};
static ChassisPowerManager *chassis_pm = nullptr;
static DJIMotorInstance *chassis_motors[4] = {nullptr};
static MotorPower *steering_motor_power[4] = {nullptr};
static DJIMotorInstance *steering_motors[4] = {nullptr};

// 默认功率上限80W（裁判系统未就绪时使用）
static float total_power_limit_w = CHASSIS_POWER_DEFAULT_LIMIT_W;
static float steering_power_ratio = 0.0f;
static float model_feedback_idle_power_w = 0.0f;
static std::array<float, 4> drive_speed_error_aps = {0.0f, 0.0f, 0.0f, 0.0f};
static std::array<float, 4> wheel_slip_score = {0.0f, 0.0f, 0.0f, 0.0f};
static std::array<float, 4> wheel_dynamic_weight = {1.0f, 1.0f, 1.0f, 1.0f};

static PowerFeedbackController feedback_controller;
static ChassisPowerFeedback feedback_mailbox = {};
static ChassisPowerFeedbackStatus feedback_status = {0, 0, 1, 1, 0, 0, CHASSIS_POWER_SOURCE_NONE, 0, 0};

static float ApplyModelCorrection(float raw_power_w)
{
    if (!std::isfinite(raw_power_w) || raw_power_w <= 0.0f)
        return 0.0f;
    return raw_power_w * feedback_status.model_correction;
}
static void UpdateSlipAndDynamicWeight(const std::array<double, 4> &predict_want_power)
{
    double total_error = 0.0;
    double total_predict = 0.0;
    for (int i = 0; i < 4; i++)
    {
        total_error += std::fabs((double)drive_speed_error_aps[i]);
        if (predict_want_power[i] > 0.0)
            total_predict += predict_want_power[i];
    }

    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *m = chassis_motors[i];
        if (!m)
        {
            wheel_slip_score[i] = 0.0f;
            wheel_dynamic_weight[i] = 1.0f;
            continue;
        }

        const float feedback_speed = m->measure.speed_aps;
        const float target_speed = feedback_speed + drive_speed_error_aps[i];
        const float target_abs = std::fabs(target_speed);
        const float feedback_abs = std::fabs(feedback_speed);
        const float speed_base = target_abs > feedback_abs ? target_abs : feedback_abs;
        const float error_ratio = std::fabs(drive_speed_error_aps[i]) / (speed_base + 1.0f);
        const float current_a = std::fabs((float)m->measure.real_current /
                                          GetMotorCurrentConversion(m->motor_type));
        const bool high_effort = predict_want_power[i] > SLIP_POWER_MIN_W ||
                                 current_a > SLIP_CURRENT_MIN_A;
        const bool slip_like = speed_base > SLIP_SPEED_MIN_APS &&
                               error_ratio > SLIP_ERROR_RATIO_ON &&
                               high_effort;

        if (slip_like)
            wheel_slip_score[i] += SLIP_SCORE_ATTACK * (1.0f - wheel_slip_score[i]);
        else
            wheel_slip_score[i] += SLIP_SCORE_RELEASE * (0.0f - wheel_slip_score[i]);

        wheel_slip_score[i] = ClampFloat(wheel_slip_score[i], 0.0f, 1.0f);

        const double error_part = total_error > 1e-6 ?
                                  std::fabs((double)drive_speed_error_aps[i]) / total_error :
                                  0.25;
        const double predict_part = total_predict > 1e-6 ?
                                    (predict_want_power[i] > 0.0 ? predict_want_power[i] / total_predict : 0.0) :
                                    0.25;
        const double traction_weight = 1.0 -
            (double)wheel_slip_score[i] * (1.0 - (double)SLIP_MIN_POWER_WEIGHT);
        double weight = (DYNAMIC_POWER_ERROR_RATIO * error_part +
                         DYNAMIC_POWER_PREDICT_RATIO * predict_part) * traction_weight;

        if (!std::isfinite(weight) || weight < 1e-6)
            weight = 1e-6;
        wheel_dynamic_weight[i] = (float)weight;
    }
}

// ==================== C接口实现 ====================

void ChassisPower_Init(DJIMotorInstance *motor_lf, DJIMotorInstance *motor_rf,
                       DJIMotorInstance *motor_lb, DJIMotorInstance *motor_rb)
{
    chassis_motors[0] = motor_lf;
    chassis_motors[1] = motor_rf;
    chassis_motors[2] = motor_lb;
    chassis_motors[3] = motor_rb;

    for (int i = 0; i < 4; i++)
    {
        if (chassis_motors[i])
            motor_power[i] = new MotorPower(GetMotorPowerInit(chassis_motors[i]->motor_type));
        else
            motor_power[i] = nullptr;
    }

    chassis_pm = new ChassisPowerManager(
        motor_power[0], motor_power[1],
        motor_power[2], motor_power[3]);
}

void ChassisPower_InitSteering(DJIMotorInstance *steer_lf, DJIMotorInstance *steer_rf,
                               DJIMotorInstance *steer_lb, DJIMotorInstance *steer_rb)
{
    steering_motors[0] = steer_lf;
    steering_motors[1] = steer_rf;
    steering_motors[2] = steer_lb;
    steering_motors[3] = steer_rb;

    for (int i = 0; i < 4; i++)
    {
        if (steering_motors[i] && !steering_motor_power[i])
        {
            steering_motor_power[i] = new MotorPower(GetMotorPowerInit(steering_motors[i]->motor_type));
        }
    }
}

void ChassisPower_SetSteeringPowerRatio(float ratio)
{
    if (!std::isfinite(ratio))
    {
        steering_power_ratio = 0.0f;
        return;
    }
    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 0.5f)
        ratio = 0.5f;
    steering_power_ratio = ratio;
}

void ChassisPower_SetModelIdlePower(float idle_power_w)
{
    if (!std::isfinite(idle_power_w) || idle_power_w < 0.0f)
    {
        model_feedback_idle_power_w = 0.0f;
        return;
    }

    model_feedback_idle_power_w = idle_power_w;
}

void ChassisPower_UpdateError(int index, float error)
{
    if (index < 0 || index > 3 || !chassis_pm) return;
    drive_speed_error_aps[index] = error;
    chassis_pm->updateMotorError(index, error);
}

void ChassisPower_Update(void)
{
    if (!chassis_pm) return;

    double model_feedback_power_w = 0.0;
    std::array<double, 4> drive_predict_want_power = {0.0, 0.0, 0.0, 0.0};

    // ① 用实时电流+转速更新每个电机的功率模型
    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *m = chassis_motors[i];
        if (!m) continue;

        // 获取实际电流(原始值)和转速(°/s)
        float raw_current = (float)m->measure.real_current;
        float speed_aps   = m->measure.speed_aps;

        // 转为RPM供功率模型使用（与Fitting.py单位对齐）
        float speed_rpm = speed_aps / 6.0f;

        // 更新功率模型（用反馈数据）
        if (motor_power[i])
        {
            double feedback_power = motor_power[i]->update(raw_current, speed_rpm,
                                                           E_disabled_predict,       // 存入feedback_power
                                                           E_enable_negative);       // 允许负功率
            if (feedback_power > 0.0)
                model_feedback_power_w += feedback_power;

            const double predicted = motor_power[i]->update((double)m->output_current,
                                                            (double)speed_rpm,
                                                            E_enable_not_limit_predict,
                                                            E_enable_negative);
            if (predicted > 0.0)
                drive_predict_want_power[i] = predicted;
        }
    }

    double steering_predict_want_power = 0.0;
    int steering_motor_count = 0;
    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *m = steering_motors[i];
        MotorPower *pm = steering_motor_power[i];
        if (!m || !pm) continue;

        const float raw_current = (float)m->measure.real_current;
        const float speed_rpm = m->measure.speed_aps / 6.0f;
        double feedback_power = pm->update(raw_current, speed_rpm, E_disabled_predict, E_enable_negative);
        if (feedback_power > 0.0)
            model_feedback_power_w += feedback_power;

        const double predicted = pm->update((double)m->output_current,
                                            (double)speed_rpm,
                                            E_enable_not_limit_predict,
                                            E_enable_negative);
        if (predicted > 0.0)
        {
            steering_predict_want_power += predicted;
        }
        steering_motor_count++;
    }

    UpdateSlipAndDynamicWeight(drive_predict_want_power);

    // ② 按误差比例分配功率预算
    ChassisPowerFeedback input;
    taskENTER_CRITICAL();
    input = feedback_mailbox;
    taskEXIT_CRITICAL();
    const ChassisPowerFeedbackStatus status = feedback_controller.update(
        input, HAL_GetTick(), (float)model_feedback_power_w,
        total_power_limit_w, model_feedback_idle_power_w);
    taskENTER_CRITICAL();
    feedback_status = status;
    taskEXIT_CRITICAL();
    const double effective_total_power = status.effective_limit_w;
    const std::array<double, 2> steering_drive_power =
        allocate_SW_power(effective_total_power, (double)steering_power_ratio, steering_predict_want_power);
    const double steering_total_power = steering_motor_count > 0 ? steering_drive_power[0] : 0.0;
    const double drive_total_power = steering_motor_count > 0 ? steering_drive_power[1] : effective_total_power;
    const std::array<double, 4> dynamic_weights = {
        (double)wheel_dynamic_weight[0],
        (double)wheel_dynamic_weight[1],
        (double)wheel_dynamic_weight[2],
        (double)wheel_dynamic_weight[3],
    };
    chassis_pm->allocatePower(drive_total_power, 1.0, &dynamic_weights);

    // ③ 对每个电机执行功率限幅
    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *m = chassis_motors[i];
        if (!m) continue;
        if (!motor_power[i]) continue;

        float speed_rpm = m->measure.speed_aps / 6.0f;
        double desired_current = (double)m->output_current;
        double power_limit_for_motor = motor_power[i]->power_limit;

        // limiter()会修改desired_current以遵守功率限制
        motor_power[i]->limiter(&desired_current, speed_rpm, power_limit_for_motor);

        // 写回限幅后的电流值
        m->output_current = ClampOutputCurrent(desired_current, m->motor_type);

        // 更新电功率数据，供调试/拟合使用
        m->power_data.current_a          = m->measure.real_current / GetMotorCurrentConversion(m->motor_type);
        m->power_data.speed_rpm          = speed_rpm;
        m->power_data.electrical_power_w      = (float)motor_power[i]->feedback_power;
        m->power_data.predict_power_w         = (float)motor_power[i]->predict_not_limit_power;
        m->power_data.predict_power_after_w   = (float)motor_power[i]->predict_power;
        m->power_data.power_budget_w          = (float)power_limit_for_motor;
    }

    if (steering_motor_count > 0)
    {
        const double steering_power_per_motor = steering_total_power / (double)steering_motor_count;
        for (int i = 0; i < 4; i++)
        {
            DJIMotorInstance *m = steering_motors[i];
            MotorPower *pm = steering_motor_power[i];
            if (!m || !pm) continue;

            const float speed_rpm = m->measure.speed_aps / 6.0f;
            double desired_current = (double)m->output_current;
            pm->limiter(&desired_current, speed_rpm, steering_power_per_motor);
            m->output_current = ClampOutputCurrent(desired_current, m->motor_type);

            m->power_data.current_a = m->measure.real_current / GetMotorCurrentConversion(m->motor_type);
            m->power_data.speed_rpm = speed_rpm;
            m->power_data.electrical_power_w = (float)pm->feedback_power;
            m->power_data.predict_power_w = (float)pm->predict_not_limit_power;
            m->power_data.predict_power_after_w = (float)pm->predict_power;
            m->power_data.power_budget_w = (float)steering_power_per_motor;
        }
    }
}

void ChassisPower_SetLimit(float power_limit)
{
    // 裁判系统未就绪时 chassis_power_limit 为 0，此时回退到默认上限；
    // 其余合法正值（含裁判系统降功率惩罚时给出的小功率）都接受，
    // 仅在非有限值或超过安全上限时拒绝，避免误把真实的小功率限制当成无效。
    if (std::isfinite(power_limit) && power_limit > 0.0f &&
        power_limit <= CHASSIS_POWER_MAX_LIMIT_W)
    {
        total_power_limit_w = power_limit;
    }
    else
    {
        total_power_limit_w = CHASSIS_POWER_DEFAULT_LIMIT_W;
    }
}

void ChassisPower_SetFeedback(const ChassisPowerFeedback *feedback)
{
    taskENTER_CRITICAL();
    feedback_mailbox = feedback ? *feedback : ChassisPowerFeedback{};
    taskEXIT_CRITICAL();
}

ChassisPowerFeedbackStatus ChassisPower_GetFeedbackStatus(void)
{
    ChassisPowerFeedbackStatus status;
    taskENTER_CRITICAL();
    status = feedback_status;
    taskEXIT_CRITICAL();
    return status;
}
float ChassisPower_GetPredictPower(void)
{
    if (!chassis_pm) return 0.0f;
    double total = chassis_pm->getTotalPredictNotLimitPower();
    for (int i = 0; i < 4; i++)
    {
        if (steering_motor_power[i])
            total += steering_motor_power[i]->predict_not_limit_power;
    }
    return (float)total;
}

float ChassisPower_GetFeedbackPower(void)
{
    double total = 0.0;
    for (int i = 0; i < 4; i++)
    {
        if (motor_power[i])
            total += motor_power[i]->feedback_power;
        if (steering_motor_power[i])
            total += steering_motor_power[i]->feedback_power;
    }
    return (float)total;
}

float ChassisPower_GetCorrectedPredictPower(void)
{
    return ApplyModelCorrection(ChassisPower_GetPredictPower());
}

float ChassisPower_GetCorrectedFeedbackPower(void)
{
    return ApplyModelCorrection(ChassisPower_GetFeedbackPower());
}

float ChassisPower_GetLimit(void)
{
    return total_power_limit_w;
}

float ChassisPower_GetModelCorrection(void)
{
    return feedback_status.model_correction;
}

float ChassisPower_GetBufferAttenuation(void)
{
    return feedback_status.buffer_attenuation;
}

float ChassisPower_GetEffectiveLimit(void)
{
    return feedback_status.effective_limit_w;
}

float ChassisPower_GetMeasuredPower(void)
{
    return feedback_status.measured_power_w;
}

float ChassisPower_GetBufferEnergy(void)
{
    return feedback_status.buffer_energy_j;
}

float ChassisPower_GetSlipScore(int index)
{
    if (index < 0 || index > 3) return 0.0f;
    return wheel_slip_score[index];
}

float ChassisPower_GetDynamicWeight(int index)
{
    if (index < 0 || index > 3) return 0.0f;
    return wheel_dynamic_weight[index];
}

// ==================== 单电机功率控制 ====================
// 不经过 ChassisPowerManager 分配，直接对单个电机限幅
// 适用于摩擦轮、拨盘等独立电机，每个有自己的功率预算

static MotorPower *single_motor_power[4] = {nullptr};

void SingleMotorPower_Init(int slot)
{
    if (slot < 0 || slot > 3) return;
    if (single_motor_power[slot]) return;  // 已初始化
    single_motor_power[slot] = new MotorPower(m3508_fitting);
}

void SingleMotorPower_Limit(DJIMotorInstance *motor, int slot, float power_budget_w)
{
    if (!motor) return;
    if (slot < 0 || slot > 3) return;
    MotorPower *pm = single_motor_power[slot];
    if (!pm)
    {
        single_motor_power[slot] = new MotorPower(GetMotorPowerInit(motor->motor_type));
        pm = single_motor_power[slot];
    }
    if (!pm) return;

    float speed_rpm   = motor->measure.speed_aps / 6.0f;
    float raw_current = (float)motor->measure.real_current;

    // ① 用反馈数据更新功率模型
    pm->update(raw_current, speed_rpm,
               E_disabled_predict,   // 存为 feedback_power
               E_enable_negative);   // 允许负功率（刹车回收）

    // ② 对 PID 输出的目标电流做功率限幅
    double desired = (double)motor->output_current;
    pm->limiter(&desired, speed_rpm, (double)power_budget_w);
    motor->output_current = ClampOutputCurrent(desired, motor->motor_type);

    // ③ 更新功率数据，供调试查看
    motor->power_data.current_a          = raw_current / GetMotorCurrentConversion(motor->motor_type);
    motor->power_data.speed_rpm          = speed_rpm;
    motor->power_data.electrical_power_w      = (float)pm->feedback_power;
    motor->power_data.predict_power_w         = (float)pm->predict_not_limit_power;
    motor->power_data.predict_power_after_w   = (float)pm->predict_power;
    motor->power_data.power_budget_w          = power_budget_w;
}

// ==================== 嵌入式C++桩函数 ====================
// arm-none-eabi-g++ 裸机环境需要手动提供

void *operator new(size_t size)
{
    void *ptr = pvPortMalloc(size);
    if (!ptr)
        while (1);
    return ptr;
}

void *operator new[](size_t size)
{
    void *ptr = pvPortMalloc(size);
    if (!ptr)
        while (1);
    return ptr;
}

void operator delete(void *ptr)
{
    vPortFree(ptr);
}

void operator delete[](void *ptr)
{
    vPortFree(ptr);
}

// 纯虚函数桩（如果意外调用则死循环，方便调试定位）
extern "C" void __cxa_pure_virtual()
{
    while (1);
}

// libstdc++ 需要的系统调用桩
extern "C" int _kill(int pid, int sig)
{
    (void)pid; (void)sig;
    return -1;
}

extern "C" int _getpid(void)
{
    return 1;
}

extern "C" int _close(int file)
{
    (void)file;
    return -1;
}

extern "C" int _lseek(int file, int ptr, int dir)
{
    (void)file; (void)ptr; (void)dir;
    return 0;
}

extern "C" int _read(int file, char *ptr, int len)
{
    (void)file; (void)ptr; (void)len;
    return 0;
}

extern "C" int _write(int file, char *ptr, int len)
{
    (void)file; (void)ptr;
    return len;
}
