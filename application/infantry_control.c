#include "infantry_control.h"

#include <math.h>
#include <stddef.h>

enum
{
    INFANTRY_CHASSIS_ZERO_FORCE = 0,
    INFANTRY_CHASSIS_ROTATE = 1,
    INFANTRY_CHASSIS_NO_FOLLOW = 2,
    INFANTRY_CHASSIS_FOLLOW = 3,
    INFANTRY_LOAD_ONE = 2,
    INFANTRY_LOAD_THREE = 3,
};

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

void InfantryGimbalTargetsReset(InfantryGimbalTargets *targets)
{
    if (targets == NULL)
        return;

    targets->yaw = 0.0f;
    targets->pitch = 0.0f;
    targets->pitch_min = 0.0f;
    targets->pitch_max = 0.0f;
    targets->initialized = 0;
}

uint8_t InfantryGimbalTargetsStep(InfantryGimbalTargets *targets,
                                  uint8_t feedback_ready,
                                  float measured_yaw,
                                  float measured_pitch,
                                  float yaw_increment,
                                  float pitch_increment,
                                  float pitch_safe_range)
{
    if (targets == NULL || !feedback_ready)
        return 0;

    if (!targets->initialized)
    {
        const float safe_range = fabsf(pitch_safe_range);
        targets->yaw = measured_yaw;
        targets->pitch = measured_pitch;
        targets->pitch_min = measured_pitch - safe_range;
        targets->pitch_max = measured_pitch + safe_range;
        targets->initialized = 1;
        return 1;
    }

    targets->yaw += yaw_increment;
    targets->pitch = Clamp(targets->pitch + pitch_increment,
                           targets->pitch_min,
                           targets->pitch_max);
    return 1;
}

uint8_t InfantrySafeChassisMode(uint8_t requested_mode, uint8_t gimbal_ready)
{
    if (gimbal_ready)
        return requested_mode;
    if (requested_mode == INFANTRY_CHASSIS_ZERO_FORCE)
        return INFANTRY_CHASSIS_ZERO_FORCE;
    if (requested_mode == INFANTRY_CHASSIS_ROTATE || requested_mode == INFANTRY_CHASSIS_FOLLOW)
        return INFANTRY_CHASSIS_NO_FOLLOW;
    return requested_mode;
}

uint8_t InfantrySafeLoadMode(uint8_t requested_mode,
                             uint8_t shooter_enabled,
                             uint8_t hardware_ready,
                             uint8_t stop_mode)
{
    return shooter_enabled && hardware_ready ? requested_mode : stop_mode;
}

float InfantryFrictionTarget(uint8_t bullet_speed,
                             float speed_15,
                             float speed_18,
                             float speed_30)
{
    switch (bullet_speed)
    {
    case 15:
        return speed_15;
    case 18:
        return speed_18;
    case 30:
        return speed_30;
    default:
        return 0.0f;
    }
}

uint8_t InfantryFrictionReady(float target_speed,
                              float left_speed,
                              float right_speed,
                              float tolerance)
{
    if (target_speed <= 0.0f)
        return 0;
    return fabsf(left_speed - target_speed) <= fabsf(tolerance) &&
           fabsf(right_speed + target_speed) <= fabsf(tolerance);
}

float InfantryLoaderBurstSpeed(float shots_per_second,
                               float reduction_ratio,
                               uint16_t projectiles_per_circle)
{
    if (shots_per_second <= 0.0f || reduction_ratio <= 0.0f || projectiles_per_circle == 0)
        return 0.0f;
    return shots_per_second * 360.0f * reduction_ratio / (float)projectiles_per_circle;
}

float InfantryLoaderProjectileAngle(uint8_t projectile_count,
                                    float reduction_ratio,
                                    uint16_t projectiles_per_circle)
{
    if (projectile_count == 0 || reduction_ratio <= 0.0f || projectiles_per_circle == 0)
        return 0.0f;
    return (float)projectile_count * 360.0f * reduction_ratio /
           (float)projectiles_per_circle;
}

uint8_t InfantryLoadModeRisingEdge(uint8_t current_mode, uint8_t previous_mode)
{
    const uint8_t is_position_command =
        current_mode == INFANTRY_LOAD_ONE || current_mode == INFANTRY_LOAD_THREE;
    return is_position_command && current_mode != previous_mode;
}

void InfantryLoaderRequestReset(InfantryLoaderRequest *request, uint8_t stop_mode)
{
    if (request == NULL)
        return;
    request->previous_requested_mode = stop_mode;
    request->pending_position_mode = stop_mode;
}

void InfantryLoaderRequestSync(InfantryLoaderRequest *request,
                               uint8_t current_mode,
                               uint8_t stop_mode)
{
    if (request == NULL)
        return;
    request->previous_requested_mode = current_mode;
    request->pending_position_mode = stop_mode;
}

uint8_t InfantryLoaderSelectMode(InfantryLoaderRequest *request,
                                 uint8_t requested_mode,
                                 uint8_t continuous_ready,
                                 uint8_t position_ready,
                                 uint8_t stop_mode)
{
    if (request == NULL)
        return stop_mode;

    if (InfantryLoadModeRisingEdge(requested_mode, request->previous_requested_mode))
        request->pending_position_mode = requested_mode;
    request->previous_requested_mode = requested_mode;

    if (requested_mode != INFANTRY_LOAD_ONE && requested_mode != INFANTRY_LOAD_THREE)
        request->pending_position_mode = stop_mode;

    if (request->pending_position_mode != stop_mode)
    {
        if (!position_ready)
            return stop_mode;
        const uint8_t pending_mode = request->pending_position_mode;
        request->pending_position_mode = stop_mode;
        return pending_mode;
    }

    if (!continuous_ready || requested_mode == INFANTRY_LOAD_ONE || requested_mode == INFANTRY_LOAD_THREE)
        return stop_mode;
    return requested_mode;
}

uint8_t InfantryControlLinkReady(uint8_t frame_received, uint8_t watchdog_online)
{
    return frame_received && watchdog_online;
}

uint8_t InfantryAllReady(const uint8_t *states, size_t count)
{
    if (states == NULL || count == 0)
        return 0;
    for (size_t i = 0; i < count; ++i)
        if (!states[i])
            return 0;
    return 1;
}

uint8_t InfantryWheelTestAllowed(uint8_t chassis_mode)
{
    return chassis_mode == INFANTRY_CHASSIS_ROTATE ||
           chassis_mode == INFANTRY_CHASSIS_NO_FOLLOW ||
           chassis_mode == INFANTRY_CHASSIS_FOLLOW;
}

void InfantryLimitChassisVector(float *vx, float *vy, float maximum_speed)
{
    if (vx == NULL || vy == NULL)
        return;

    const float speed = sqrtf(*vx * *vx + *vy * *vy);
    if (maximum_speed <= 0.0f)
    {
        *vx = 0.0f;
        *vy = 0.0f;
    }
    else if (speed > maximum_speed)
    {
        const float scale = maximum_speed / speed;
        *vx *= scale;
        *vy *= scale;
    }
}

void InfantryLimitWheelSpeeds(float wheel_speeds[4], float maximum_speed)
{
    if (wheel_speeds == NULL)
        return;

    float largest = 0.0f;
    for (size_t i = 0; i < 4; ++i)
    {
        const float magnitude = fabsf(wheel_speeds[i]);
        if (magnitude > largest)
            largest = magnitude;
    }

    if (maximum_speed <= 0.0f)
    {
        for (size_t i = 0; i < 4; ++i)
            wheel_speeds[i] = 0.0f;
    }
    else if (largest > maximum_speed)
    {
        const float scale = maximum_speed / largest;
        for (size_t i = 0; i < 4; ++i)
            wheel_speeds[i] *= scale;
    }
}

void InfantryMecanumCalculate(float vx,
                              float vy,
                              float wz,
                              float lf_center,
                              float rf_center,
                              float lb_center,
                              float rb_center,
                              float *lf,
                              float *rf,
                              float *lb,
                              float *rb)
{
    if (lf == NULL || rf == NULL || lb == NULL || rb == NULL)
        return;
    *lf = -vx - vy - wz * lf_center;
    *rf = -vx + vy - wz * rf_center;
    *lb = vx - vy - wz * lb_center;
    *rb = vx + vy - wz * rb_center;
}
