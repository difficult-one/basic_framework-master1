#ifndef INFANTRY_CONTROL_H
#define INFANTRY_CONTROL_H

#include <stdint.h>
#include <stddef.h>

typedef struct
{
    float yaw;
    float pitch;
    float pitch_min;
    float pitch_max;
    uint8_t initialized;
} InfantryGimbalTargets;

typedef struct
{
    uint8_t previous_requested_mode;
    uint8_t pending_position_mode;
} InfantryLoaderRequest;

void InfantryGimbalTargetsReset(InfantryGimbalTargets *targets);
uint8_t InfantryGimbalTargetsStep(InfantryGimbalTargets *targets,
                                  uint8_t feedback_ready,
                                  float measured_yaw,
                                  float measured_pitch,
                                  float yaw_increment,
                                  float pitch_increment,
                                  float pitch_safe_range);
uint8_t InfantrySafeChassisMode(uint8_t requested_mode, uint8_t gimbal_ready);
uint8_t InfantrySafeLoadMode(uint8_t requested_mode,
                             uint8_t shooter_enabled,
                             uint8_t hardware_ready,
                             uint8_t stop_mode);
float InfantryFrictionTarget(uint8_t bullet_speed,
                             float speed_15,
                             float speed_18,
                             float speed_30);
uint8_t InfantryFrictionReady(float target_speed,
                              float left_speed,
                              float right_speed,
                              float tolerance);
float InfantryLoaderBurstSpeed(float shots_per_second,
                               float reduction_ratio,
                               uint16_t projectiles_per_circle);
float InfantryLoaderProjectileAngle(uint8_t projectile_count,
                                    float reduction_ratio,
                                    uint16_t projectiles_per_circle);
uint8_t InfantryLoadModeRisingEdge(uint8_t current_mode, uint8_t previous_mode);
void InfantryLoaderRequestReset(InfantryLoaderRequest *request, uint8_t stop_mode);
void InfantryLoaderRequestSync(InfantryLoaderRequest *request,
                               uint8_t current_mode,
                               uint8_t stop_mode);
uint8_t InfantryLoaderSelectMode(InfantryLoaderRequest *request,
                                 uint8_t requested_mode,
                                 uint8_t continuous_ready,
                                 uint8_t position_ready,
                                 uint8_t stop_mode);
uint8_t InfantryControlLinkReady(uint8_t frame_received, uint8_t watchdog_online);
uint8_t InfantryAllReady(const uint8_t *states, size_t count);
uint8_t InfantryWheelTestAllowed(uint8_t chassis_mode);
void InfantryLimitChassisVector(float *vx, float *vy, float maximum_speed);
void InfantryLimitWheelSpeeds(float wheel_speeds[4], float maximum_speed);
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
                              float *rb);

#endif
