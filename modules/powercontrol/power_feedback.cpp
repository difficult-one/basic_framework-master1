#include "power_feedback.h"
#include <algorithm>
#include <cmath>

namespace
{
float clamp(float value, float low, float high)
{
    return std::max(low, std::min(value, high));
}

bool fresh(uint32_t now, uint32_t sample, uint32_t timeout)
{
    return timeout > 0 && (uint32_t)(now - sample) < timeout;
}

float alpha(uint32_t dt_ms, float tau_ms)
{
    return 1.0f - std::exp(-(float)dt_ms / tau_ms);
}
}

ChassisPowerFeedbackStatus PowerFeedbackController::update(
    const ChassisPowerFeedback &input, uint32_t now_ms,
    float model_power_w, float limit_w, float idle_w)
{
    idle_w = std::isfinite(idle_w) ? std::max(0.0f, idle_w) : 0;
    state_.power_age_ms = (uint32_t)(now_ms - input.power_sample_ms);
    state_.power_valid = input.power_source >= CHASSIS_POWER_SOURCE_METER_CAN1 &&
                         input.power_source <= CHASSIS_POWER_SOURCE_REFEREE &&
                         std::isfinite(input.power_w) &&
                         fresh(now_ms, input.power_sample_ms, input.power_timeout_ms);
    state_.buffer_valid = input.buffer_valid && std::isfinite(input.buffer_energy_j) &&
                          input.buffer_energy_j >= 0 &&
                          fresh(now_ms, input.buffer_sample_ms, input.buffer_timeout_ms);
    state_.power_source = state_.power_valid ? input.power_source : CHASSIS_POWER_SOURCE_NONE;
    state_.measured_power_w = state_.power_valid ? input.power_w : 0;
    state_.buffer_energy_j = state_.buffer_valid ? input.buffer_energy_j : 0;

    if (state_.power_valid)
    {
        const bool continuous = have_power_ && last_source_ == input.power_source;
        const bool new_sample = !continuous || last_power_sequence_ != input.power_sequence;
        if (new_sample)
        {
            // Do not learn across a source change or dropout; preserve the old correction.
            const uint32_t dt = continuous ? (uint32_t)(input.power_sample_ms - last_power_ms_) : 0;
            const float measured_drive = input.power_w - idle_w;
            if (dt > 0 && dt < input.power_timeout_ms &&
                std::isfinite(model_power_w) && model_power_w >= 5 && measured_drive >= 5)
            {
                const float target = clamp(measured_drive / model_power_w, 0.2f, 20.0f);
                state_.model_correction += alpha(dt, 200.0f) * (target - state_.model_correction);
            }
            last_power_ms_ = input.power_sample_ms;
            last_power_sequence_ = input.power_sequence;
            last_source_ = input.power_source;
            have_power_ = true;
        }
    }
    else
    {
        have_power_ = false;
    }

    if (state_.buffer_valid)
    {
        if (!have_buffer_ || last_buffer_sequence_ != input.buffer_sequence)
        {
            uint32_t dt = have_buffer_ ? (uint32_t)(input.buffer_sample_ms - last_buffer_ms_) : 0;
            if (dt >= input.buffer_timeout_ms) dt = 0;
            const float energy = input.buffer_energy_j;
            const float error = energy - 45.0f;
            // Equivalent units to the previous 5ms PI, now integrated once per sample.
            buffer_integral_ = clamp(buffer_integral_ + error * ((float)dt * 0.001f), -1.5f, 1.5f);
            float target = 1.0f + 0.018f * error + 0.07f * buffer_integral_;
            if (energy <= 10.0f)
            {
                target = 0.35f;
                state_.buffer_attenuation = 0.35f;
            }
            else if (energy < 30.0f)
            {
                target = std::min(target, 0.35f + 0.4f * (energy - 10.0f) / 20.0f);
            }
            target = clamp(target, 0.35f, 1.0f);
            // First valid low-energy frame must protect without waiting for a second frame.
            if (!have_buffer_ && target < state_.buffer_attenuation)
                state_.buffer_attenuation = target;
            const float tau = target < state_.buffer_attenuation ? 22.4f : 247.5f;
            state_.buffer_attenuation += alpha(dt, tau) * (target - state_.buffer_attenuation);
            last_buffer_ms_ = input.buffer_sample_ms;
            last_buffer_sequence_ = input.buffer_sequence;
            have_buffer_ = true;
        }
    }
    else
    {
        // A lost referee is not evidence that the energy buffer has recovered.
        have_buffer_ = false;
        buffer_integral_ = 0;
    }

    limit_w = std::isfinite(limit_w) ? std::max(0.0f, limit_w) : 0;
    float attenuation = state_.buffer_attenuation;
    if (state_.power_valid)
    {
        if (input.power_w > limit_w && input.power_w > 1)
            attenuation *= limit_w / input.power_w;
    }
    else if (std::isfinite(model_power_w) && model_power_w + idle_w > limit_w && model_power_w + idle_w > 1)
    {
        attenuation *= limit_w / (model_power_w + idle_w);
    }
    // The meter measures the whole branch; reserve non-motor idle load even
    // during sensor dropout, before converting the motor budget into model units.
    const float motor_budget = std::max(0.0f, limit_w * attenuation * 0.98f - idle_w);
    const float target_limit = motor_budget / std::max(1.0f, state_.model_correction);
    // Tighten immediately. A long task pause must not cause a large recovery step.
    const uint32_t dt = std::min((uint32_t)(now_ms - last_update_ms_), (uint32_t)20);
    state_.effective_limit_w = !initialized_ ? target_limit :
        std::min(target_limit, state_.effective_limit_w + 0.08f * (float)dt);
    last_update_ms_ = now_ms;
    initialized_ = true;
    return state_;
}
