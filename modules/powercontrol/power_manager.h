//
// Created by 3545 on 25-11-1.
// 底盘功率管理系统：电机功率建模、功率限制、多电机功率分配
//

#ifndef POWERCTRL_FORFRAMEWORK_H
#define POWERCTRL_FORFRAMEWORK_H

#include <array>
#include <cmath>
#include <cstdint>

// 误差总和阈值：低于此值四轮均分功率
#define M_Too_Small_AllErrors 500.0

// 启用功率补偿（预留2%安全余量）
#define M_Enable_PowerCompensation
#define M_Power_Compensation_Alpha 0.02

// 总功率阈值：>=54W 时启用"保底+比例"分配，每电机保底8W
#define M_Motor_ReservedPower_Border 54.0
#define M_PerMotor_ReservedPower 8.0

// 负功率处理模式
typedef enum {
    E_disabled_negative,  // 功率始终为正
    E_enable_negative     // 发电/制动时功率为负
}E_CalMotorPower_Negative_Status_Type;

// 功率预测模式：控制 update() 结果存入哪个变量
typedef enum {
    E_disabled_predict,         // → feedback_power（用反馈电流）
    E_enable_predict,           // → predict_power（用目标电流）
    E_enable_not_limit_predict  // → predict_not_limit_power（未经限制）
}E_Predict_Status_Type;

// 电机模型初始化参数（由 Fitting.py 拟合得到）
typedef struct{
    double k0, k1, k2, k3, k4, k5;  // 六项二次多项式系数
    double real_current_conversion;  // 原始读数 → 真实电流(A) 的换算比例
}motor_power_init_t;

class MotorPower{
protected:
    const double K0, K1, K2, K3, K4, K5, Current_Conversion;

public:
    double feedback_power = 0;
    double predict_not_limit_power = 0;
    double predict_power = 0;
    double power_limit = 0;

    explicit MotorPower(const motor_power_init_t &motor_power_init_data);

    // 原始电流 → 真实电流
    [[nodiscard]] double getMotorRealCurrent(double current) const;

    // P = K0 + K1*abs(I) + K2*abs(W) + K3*I*W + K4*I^2 + K5*W^2
    double update(double current, double speed,
                  E_Predict_Status_Type Predict_status = E_disabled_predict,
                  E_CalMotorPower_Negative_Status_Type Negative_Status = E_disabled_negative);

    // 功率限制：超限时求解衰减系数 k 缩减目标电流
    double limiter(double *desired_current, double current_speed, double motor_power_limit);
};

// 底盘功率管理器（四轮）
class ChassisPowerManager {
protected:
    std::array<MotorPower*, 4> motors_;
    std::array<float, 4> motor_errors_;  // 各电机误差（作为分配权重）；用 float 避免跨任务读写撕裂

public:
    ChassisPowerManager(MotorPower* motor1, MotorPower* motor2,
                        MotorPower* motor3, MotorPower* motor4);

    void updateMotorError(size_t index, float error);
    void allocatePower(double total_power_limit, double buffer_power_attenuation = 1.0,
                       const std::array<double, 4> *dynamic_weights = nullptr);

    [[nodiscard]] double getTotalPredictNotLimitPower() const;
    [[nodiscard]] double getTotalPowerLimit() const;
    [[nodiscard]] double getTotalPredictPower() const;
};

// 按误差比例分配功率（误差大 → 分得多）
std::array<double, 4> power_allocation_by_error(
    std::array<float, 4> motor_errors,
    double total_power_limit,
    double buffer_power_attenuation = 1.0);

// 舵轮功率分配：[舵机功率, 轮电机功率]
std::array<double, 2> allocate_SW_power(
    double total_power, double servo_rate, double servo_predict_want_power);

// 旋转速度衰减：平移越快越抑制旋转
double rotate_speed_allocation(int16_t vx, int16_t vy, int16_t rotate, double alpha);

// 旋转角度前馈补偿
void rotate_theta_forwardfeed(double* theta, double rotate, double translation, double kp);

// 滑动平均滤波器（环形缓冲区，O(1) 复杂度）
#endif //POWERCTRL_FORFRAMEWORK_H
