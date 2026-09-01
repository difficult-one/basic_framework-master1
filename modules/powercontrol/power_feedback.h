#ifndef POWER_FEEDBACK_H
#define POWER_FEEDBACK_H

#include <stdint.h>

typedef enum
{
    CHASSIS_POWER_SOURCE_NONE = 0,
    CHASSIS_POWER_SOURCE_METER_CAN1,
    CHASSIS_POWER_SOURCE_METER_CAN2,
    CHASSIS_POWER_SOURCE_REFEREE
} ChassisPowerSource;

/* Sample times are HAL_GetTick() milliseconds, never the time of task polling. */
typedef struct
{
    float power_w;
    float buffer_energy_j;
    uint32_t power_sequence;
    uint32_t power_sample_ms;
    uint32_t power_timeout_ms;
    uint32_t buffer_sequence;
    uint32_t buffer_sample_ms;
    uint32_t buffer_timeout_ms;
    ChassisPowerSource power_source;
    uint8_t buffer_valid;
} ChassisPowerFeedback;

typedef struct
{
    float measured_power_w;
    float buffer_energy_j;
    float model_correction;
    float buffer_attenuation;
    float effective_limit_w;
    uint32_t power_age_ms;
    ChassisPowerSource power_source;
    uint8_t power_valid;
    uint8_t buffer_valid;
} ChassisPowerFeedbackStatus;

#ifdef __cplusplus
/* Single owner: the motor task. No HAL/RTOS dependencies. */
class PowerFeedbackController
{
public:
    ChassisPowerFeedbackStatus update(const ChassisPowerFeedback &input, uint32_t now_ms,
                                     float model_power_w, float limit_w, float idle_w);
    const ChassisPowerFeedbackStatus &status() const { return state_; }

private:
    ChassisPowerFeedbackStatus state_ = {0, 0, 1, 1, 0, 0, CHASSIS_POWER_SOURCE_NONE, 0, 0};
    ChassisPowerSource last_source_ = CHASSIS_POWER_SOURCE_NONE;
    uint32_t last_power_sequence_ = 0, last_power_ms_ = 0;
    uint32_t last_buffer_sequence_ = 0, last_buffer_ms_ = 0;
    uint32_t last_update_ms_ = 0;
    float buffer_integral_ = 0;
    bool have_power_ = false, have_buffer_ = false, initialized_ = false;
};
#endif
#endif
