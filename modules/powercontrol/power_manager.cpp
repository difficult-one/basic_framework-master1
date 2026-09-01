//
// Created by 3545 on 25-11-1.
//

#include "power_manager.h"
#include <algorithm>

// ==================== MotorPower ====================

MotorPower::MotorPower(const motor_power_init_t &motor_power_init_data)
    :K0(motor_power_init_data.k0),
     K1(motor_power_init_data.k1),
     K2(motor_power_init_data.k2),
     K3(motor_power_init_data.k3),
     K4(motor_power_init_data.k4),
     K5(motor_power_init_data.k5),
     Current_Conversion(motor_power_init_data.real_current_conversion) {}

double MotorPower::getMotorRealCurrent(const double current) const {
    return current / Current_Conversion;
}

// Runtime formula matches Fitting.py:
// P = K0 + K1*abs(I) + K2*abs(W) + K3*I*W + K4*I^2 + K5*W^2
double MotorPower::update(double current, double speed,
                          const E_Predict_Status_Type Predict_status,
                          const E_CalMotorPower_Negative_Status_Type Negative_Status) {
    const double signed_current = getMotorRealCurrent(current);
    const double signed_speed = speed;
    const double abs_current = std::abs(signed_current);
    const double abs_speed = std::abs(signed_speed);
    const double direction_power = (Negative_Status == E_enable_negative)
        ? signed_current * signed_speed
        : abs_current * abs_speed;

    const double power = (K0 +
                   K1 * abs_current +
                   K2 * abs_speed +
                   K3 * direction_power +
                   K4 * abs_current * abs_current +
                   K5 * abs_speed * abs_speed);

    if (Predict_status == E_enable_predict) {
        predict_power = power;
    } else if (Predict_status == E_disabled_predict) {
        feedback_power = power;
    } else if (Predict_status == E_enable_not_limit_predict) {
        predict_not_limit_power = power;
    }

    return power;
}

// 求解 a*k² + b*k + c = 0，找 k∈[0,1] 使 P(k*I) ≤ P_limit
// a = K4*I², b = (K1+K3*W)*I, c = K0+K2*W+K5*W²-P_limit
double MotorPower::limiter(double *desired_current, const double current_speed,
                           const double motor_power_limit) {

    if (desired_current == nullptr) return 0.0;

    power_limit = motor_power_limit;

    // 异常情况：负功率限制 → 清零
    if (motor_power_limit < 0) {
        *desired_current = 0;
        update(*desired_current, current_speed, E_enable_predict);
        return 0.0;
    }

    const double desired_current_ = *desired_current;
    const double real_desired_current = std::abs(getMotorRealCurrent(desired_current_));
    const double real_current_speed = std::abs(current_speed);

    // 预测功率未超标，不需要限制
    if (const double predicted_power = update(desired_current_, current_speed,
                                               E_enable_not_limit_predict);
        predicted_power <= motor_power_limit) {
        update(*desired_current, current_speed, E_enable_predict);
        return 1.0;
    }

    // 构建一元二次方程系数
    const double a = K4 * real_desired_current * real_desired_current;
    const double b = (K1 + K3 * real_current_speed) * real_desired_current;
    const double c = K0 + K2 * real_current_speed
                     + K5 * real_current_speed * real_current_speed - motor_power_limit;
    const double discriminant = b * b - 4 * a * c;
    auto fallback_limit = [&]() {
        double low = 0.0;
        double high = 1.0;
        for (int i = 0; i < 24; ++i) {
            const double mid = (low + high) * 0.5;
            const double test_current = desired_current_ * mid;
            const double test_power = update(test_current, current_speed, E_enable_not_limit_predict);
            if (std::isfinite(test_power) && test_power <= motor_power_limit) {
                low = mid;
            } else {
                high = mid;
            }
        }

        *desired_current = desired_current_ * low;
        update(desired_current_, current_speed, E_enable_not_limit_predict);
        update(*desired_current, current_speed, E_enable_predict);
        return low;
    };

    // a≈0：退化为一次方程 b*k + c = 0
    if (std::abs(a) < 1e-9) {
        if (std::abs(b) < 1e-9) {
            if (c <= 1e-9) {
                update(*desired_current, current_speed, E_enable_predict);
                return 1.0;
            }
            *desired_current = 0;
            update(*desired_current, current_speed, E_enable_predict);
            return 0.0;
        }
        double k = -c / b;
        if (k < 0.0) {
            *desired_current = 0;
            update(*desired_current, current_speed, E_enable_predict);
            return 0.0;
        }
        if (k > 1.0) {
            update(*desired_current, current_speed, E_enable_predict);
            return 1.0;
        }
        *desired_current *= k;
        update(*desired_current, current_speed, E_enable_predict);
        return k;
    }

    // 无解 → 清零
    if (discriminant < 0) {
        return fallback_limit();
    }

    // 重根
    if (std::abs(discriminant) < 1e-9) {
        double k = -b / (2 * a);
        if (k < 0.0) {
            *desired_current = 0;
            update(*desired_current, current_speed, E_enable_predict);
            return 0.0;
        }
        if (k > 1.0) {
            update(*desired_current, current_speed, E_enable_predict);
            return 1.0;
        }
        *desired_current *= k;
        update(*desired_current, current_speed, E_enable_predict);
        return k;
    }

    // 两个根，选 [0,1] 范围内的
    const double k1 = (-b - std::sqrt(discriminant)) / (2 * a);
    const double k2 = (-b + std::sqrt(discriminant)) / (2 * a);

    auto in_range = [](double k) { return k > 0.0 && k < 1.0; };

    if (in_range(k1) && in_range(k2)) {
        *desired_current *= std::max(k1, k2);
        update(*desired_current, current_speed, E_enable_predict);
        return std::max(k1, k2);
    }
    if (in_range(k1)) {
        *desired_current *= k1;
        update(*desired_current, current_speed, E_enable_predict);
        return k1;
    }
    if (in_range(k2)) {
        *desired_current *= k2;
        update(*desired_current, current_speed, E_enable_predict);
        return k2;
    }

    return fallback_limit();
}

// ==================== 功率分配算法 ====================
// 情况1: total_error ≤ 500  → 均分
// 情况2: total_power < 54W → 按误差比例分
// 情况3: total_power ≥ 54W → 每电机保底8W，剩余按比例分
std::array<double, 4> power_allocation_by_error(
    std::array<float, 4> motor_errors,
    double total_power_limit,
    const double buffer_power_attenuation) {

#ifdef M_Enable_PowerCompensation
    total_power_limit *= (1 - M_Power_Compensation_Alpha);
#endif
    total_power_limit *= buffer_power_attenuation;

    if (total_power_limit <= 1e-9) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    for (float& error : motor_errors) {
        error = std::abs(error);
    }

    const double total_error = motor_errors[0] + motor_errors[1]
                             + motor_errors[2] + motor_errors[3];

    // 误差太小 → 均分
    if (total_error <= M_Too_Small_AllErrors) {
        double equal_share = total_power_limit / 4;
        return {equal_share, equal_share, equal_share, equal_share};
    }

    std::array<double, 4> motor_power_limits = {0.0, 0.0, 0.0, 0.0};

    // 按误差比例分配
    for (int i = 0; i < 4; ++i) {
        const double ratio = motor_errors[i] / total_error;
        if (total_power_limit < M_Motor_ReservedPower_Border) {
            motor_power_limits[i] = ratio * total_power_limit;
        } else {
            motor_power_limits[i] = ratio * (total_power_limit - 4 * M_PerMotor_ReservedPower)
                                  + M_PerMotor_ReservedPower;
        }
    }

    return motor_power_limits;
}

// ==================== ChassisPowerManager 计算每个轮子的误差====================

ChassisPowerManager::ChassisPowerManager(MotorPower* motor1, MotorPower* motor2,
                                         MotorPower* motor3, MotorPower* motor4)
    : motors_(), motor_errors_() {
    motors_[0] = motor1;
    motors_[1] = motor2;
    motors_[2] = motor3;
    motors_[3] = motor4;
    motor_errors_.fill(0.0f);
}

void ChassisPowerManager::updateMotorError(const size_t index, const float error) {
    if (index >= 4) return;
    motor_errors_[index] = std::abs(error);
}

void ChassisPowerManager::allocatePower(const double total_power_limit,
                                         const double buffer_power_attenuation,
                                         const std::array<double, 4> *dynamic_weights) {
    std::array<double, 4> allocated_power = {0.0, 0.0, 0.0, 0.0};

    if (dynamic_weights == nullptr) {
        allocated_power = power_allocation_by_error(motor_errors_, total_power_limit, buffer_power_attenuation);
    } else {
        double adjusted_total_power = total_power_limit * buffer_power_attenuation;
        if (adjusted_total_power > 1e-9) {
            double weight_sum = 0.0;
            size_t active_count = 0;
            for (size_t i = 0; i < 4; ++i) {
                if (!motors_[i]) continue;

                const double weight = (*dynamic_weights)[i];
                weight_sum += (std::isfinite(weight) && weight > 0.0) ? weight : 1.0;
                active_count++;
            }
            if (active_count == 0) {
                weight_sum = 0.0;
            } else if (weight_sum <= 1e-9) {
                weight_sum = (double)active_count;
            }

            const bool enable_reserved_power = adjusted_total_power >= M_Motor_ReservedPower_Border;
            const double allocatable_power = enable_reserved_power ?
                adjusted_total_power - (double)active_count * M_PerMotor_ReservedPower :
                adjusted_total_power;

            for (size_t i = 0; i < 4; ++i) {
                if (!motors_[i] || weight_sum <= 1e-9) continue;

                const double raw_weight = (*dynamic_weights)[i];
                const double weight = (std::isfinite(raw_weight) && raw_weight > 0.0) ? raw_weight : 1.0;
                allocated_power[i] =
                    (enable_reserved_power ? M_PerMotor_ReservedPower : 0.0) +
                    allocatable_power * weight / weight_sum;
            }
        }
    }

    for (size_t i = 0; i < 4; ++i) {
        if (motors_[i]) {
            motors_[i]->power_limit = allocated_power[i];
        }
    }
}

double ChassisPowerManager::getTotalPredictNotLimitPower() const {
    double total = 0.0;
    for (const auto* motor : motors_) {
        if (motor) total += motor->predict_not_limit_power;
    }
    return total;
}

double ChassisPowerManager::getTotalPowerLimit() const {
    double total = 0.0;
    for (const auto* motor : motors_) {
        if (motor) total += motor->power_limit;
    }
    return total;
}

double ChassisPowerManager::getTotalPredictPower() const {
    double total = 0.0;
    for (const auto* motor : motors_) {
        if (motor) total += motor->predict_power;
    }
    return total;
}

// ==================== 舵轮分配 ====================

std::array<double, 2> allocate_SW_power(const double total_power,
                                       const double servo_rate,
                                       const double servo_predict_want_power) {
    if (servo_rate > 1 || servo_rate <= 0) {
        return {0.0, 0.0};
    }

    std::array<double, 2> servo_wheel_power = {0.0, 0.0};
    double servo_max = total_power * servo_rate;

    servo_wheel_power[0] = (servo_predict_want_power >= servo_max)
                           ? servo_max : servo_predict_want_power;
    servo_wheel_power[1] = total_power - servo_wheel_power[0];

    return servo_wheel_power;
}

// ==================== 旋转速度衰减 ====================
// adjusted_rotate = |rotate| - alpha * translation
double rotate_speed_allocation(const int16_t vx, const int16_t vy,
                                const int16_t rotate, const double alpha) {
    const double translation = sqrt(vx * vx + vy * vy);

    if (std::abs(translation) <= 1e-9) return rotate;

    double adjusted_rotate = std::abs(rotate) - alpha * translation;
    if (adjusted_rotate < 0) adjusted_rotate = 0;

    return (rotate < 0) ? -adjusted_rotate : adjusted_rotate;
}

// ==================== 旋转前馈补偿 ====================
void rotate_theta_forwardfeed(double* theta, double rotate,
                                double translation, double kp) {
    if (translation != 0) {
        *theta -= kp * rotate;
    }
}

// ==================== 滑动平均滤波器 ====================
