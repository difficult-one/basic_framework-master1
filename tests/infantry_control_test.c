#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "infantry_control.h"

enum
{
    TEST_CHASSIS_ZERO_FORCE = 0,
    TEST_CHASSIS_ROTATE = 1,
    TEST_CHASSIS_NO_FOLLOW = 2,
    TEST_CHASSIS_FOLLOW = 3,
    TEST_LOAD_STOP = 0,
    TEST_LOAD_ONE = 2,
    TEST_LOAD_THREE = 3,
    TEST_LOAD_BURST = 4,
};

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static void test_gimbal_targets_wait_for_feedback_and_capture_current_pose(void)
{
    InfantryGimbalTargets targets = {0};

    assert(InfantryGimbalTargetsStep(&targets, 0, 120.0f, 5.0f, 10.0f, 10.0f, 15.0f) == 0);
    assert(targets.initialized == 0);

    assert(InfantryGimbalTargetsStep(&targets, 1, 120.0f, 5.0f, 10.0f, 10.0f, 15.0f) == 1);
    assert(targets.initialized == 1);
    assert_close(targets.yaw, 120.0f);
    assert_close(targets.pitch, 5.0f);
    assert_close(targets.pitch_min, -10.0f);
    assert_close(targets.pitch_max, 20.0f);
}

static void test_gimbal_targets_apply_increments_and_clamp_pitch(void)
{
    InfantryGimbalTargets targets = {0};
    InfantryGimbalTargetsStep(&targets, 1, 30.0f, 2.0f, 0.0f, 0.0f, 15.0f);

    InfantryGimbalTargetsStep(&targets, 1, 30.0f, 2.0f, 4.0f, 40.0f, 15.0f);
    assert_close(targets.yaw, 34.0f);
    assert_close(targets.pitch, 17.0f);

    InfantryGimbalTargetsStep(&targets, 1, 30.0f, 2.0f, 0.0f, -80.0f, 15.0f);
    assert_close(targets.pitch, -13.0f);

    InfantryGimbalTargetsReset(&targets);
    assert(targets.initialized == 0);
}

static void test_unready_gimbal_disables_angle_dependent_chassis_modes(void)
{
    assert(InfantrySafeChassisMode(TEST_CHASSIS_FOLLOW, 0) == TEST_CHASSIS_NO_FOLLOW);
    assert(InfantrySafeChassisMode(TEST_CHASSIS_ROTATE, 0) == TEST_CHASSIS_NO_FOLLOW);
    assert(InfantrySafeChassisMode(TEST_CHASSIS_ZERO_FORCE, 0) == TEST_CHASSIS_ZERO_FORCE);
    assert(InfantrySafeChassisMode(TEST_CHASSIS_FOLLOW, 1) == TEST_CHASSIS_FOLLOW);
}

static void test_friction_targets_and_readiness(void)
{
    assert_close(InfantryFrictionTarget(15, 5000.0f, 6000.0f, 8000.0f), 5000.0f);
    assert_close(InfantryFrictionTarget(18, 5000.0f, 6000.0f, 8000.0f), 6000.0f);
    assert_close(InfantryFrictionTarget(30, 5000.0f, 6000.0f, 8000.0f), 8000.0f);
    assert_close(InfantryFrictionTarget(0, 5000.0f, 6000.0f, 8000.0f), 0.0f);

    assert(InfantryFrictionReady(5000.0f, 4900.0f, -5100.0f, 200.0f) == 1);
    assert(InfantryFrictionReady(5000.0f, 4700.0f, -5100.0f, 200.0f) == 0);
    assert(InfantryFrictionReady(5000.0f, 4900.0f, 5100.0f, 200.0f) == 0);
    assert(InfantryFrictionReady(0.0f, 0.0f, 0.0f, 200.0f) == 0);
}

static void test_loader_conversion_and_edges(void)
{
    assert_close(InfantryLoaderBurstSpeed(1.0f, 36.0f, 10), 1296.0f);
    assert_close(InfantryLoaderBurstSpeed(1.0f, 36.0f, 0), 0.0f);
    assert_close(InfantryLoaderProjectileAngle(1, 36.0f, 10), 1296.0f);
    assert_close(InfantryLoaderProjectileAngle(3, 36.0f, 10), 3888.0f);
    assert(InfantryLoadModeRisingEdge(TEST_LOAD_ONE, TEST_LOAD_STOP) == 1);
    assert(InfantryLoadModeRisingEdge(TEST_LOAD_ONE, TEST_LOAD_ONE) == 0);
    assert(InfantryLoadModeRisingEdge(TEST_LOAD_THREE, TEST_LOAD_STOP) == 1);
    assert(InfantryLoadModeRisingEdge(TEST_LOAD_STOP, TEST_LOAD_ONE) == 0);
}

static void test_loader_request_is_queued_until_ready_and_consumed_once(void)
{
    InfantryLoaderRequest request = {0};
    InfantryLoaderRequestReset(&request, TEST_LOAD_STOP);

    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 0, 0, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_ONE);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_STOP);

    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_STOP, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 1, 0, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_ONE);

    InfantryLoaderRequestReset(&request, TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 0, 0, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_STOP, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_STOP);

    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_BURST, 0, 0, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_BURST, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_BURST);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_BURST, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_BURST);
}

static void test_mecanum_signs(void)
{
    float lf, rf, lb, rb;

    InfantryMecanumCalculate(100.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                             &lf, &rf, &lb, &rb);
    assert_close(lf, -100.0f);
    assert_close(rf, -100.0f);
    assert_close(lb, 100.0f);
    assert_close(rb, 100.0f);

    InfantryMecanumCalculate(0.0f, 100.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                             &lf, &rf, &lb, &rb);
    assert_close(lf, -100.0f);
    assert_close(rf, 100.0f);
    assert_close(lb, -100.0f);
    assert_close(rb, 100.0f);

    InfantryMecanumCalculate(0.0f, 0.0f, 100.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                             &lf, &rf, &lb, &rb);
    assert_close(lf, -100.0f);
    assert_close(rf, -100.0f);
    assert_close(lb, -100.0f);
    assert_close(rb, -100.0f);
}

static void test_chassis_vector_limit_preserves_direction(void)
{
    float vx = 3000.0f;
    float vy = 4000.0f;
    InfantryLimitChassisVector(&vx, &vy, 2500.0f);
    assert_close(vx, 1500.0f);
    assert_close(vy, 2000.0f);

    vx = 1000.0f;
    vy = 500.0f;
    InfantryLimitChassisVector(&vx, &vy, 2500.0f);
    assert_close(vx, 1000.0f);
    assert_close(vy, 500.0f);
}

static void test_control_link_requires_a_received_frame_and_live_watchdog(void)
{
    assert(InfantryControlLinkReady(0, 0) == 0);
    assert(InfantryControlLinkReady(0, 1) == 0);
    assert(InfantryControlLinkReady(1, 0) == 0);
    assert(InfantryControlLinkReady(1, 1) == 1);
}

static void test_all_required_feedback_must_be_ready(void)
{
    const uint8_t all_ready[4] = {1, 1, 1, 1};
    const uint8_t one_missing[4] = {1, 0, 1, 1};
    assert(InfantryAllReady(all_ready, 4) == 1);
    assert(InfantryAllReady(one_missing, 4) == 0);
    assert(InfantryAllReady(all_ready, 0) == 0);
}

static void test_wheel_test_respects_zero_force_and_wheel_targets_are_limited(void)
{
    assert(InfantryWheelTestAllowed(TEST_CHASSIS_ZERO_FORCE) == 0);
    assert(InfantryWheelTestAllowed(TEST_CHASSIS_NO_FOLLOW) == 1);
    assert(InfantryWheelTestAllowed(99) == 0);

    float wheels[4] = {12000.0f, -6000.0f, 3000.0f, -12000.0f};
    InfantryLimitWheelSpeeds(wheels, 6000.0f);
    assert_close(wheels[0], 6000.0f);
    assert_close(wheels[1], -3000.0f);
    assert_close(wheels[2], 1500.0f);
    assert_close(wheels[3], -6000.0f);
}

static void test_loader_request_is_forced_to_stop_on_disabled_or_offline_shooter(void)
{
    assert(InfantrySafeLoadMode(TEST_LOAD_ONE, 0, 1, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantrySafeLoadMode(TEST_LOAD_ONE, 1, 0, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantrySafeLoadMode(TEST_LOAD_ONE, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_ONE);
}

static void test_loader_fault_sync_clears_pending_but_preserves_held_input(void)
{
    InfantryLoaderRequest request = {0};
    InfantryLoaderRequestSync(&request, TEST_LOAD_ONE, TEST_LOAD_STOP);

    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_STOP, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_STOP);
    assert(InfantryLoaderSelectMode(&request, TEST_LOAD_ONE, 1, 1, TEST_LOAD_STOP) == TEST_LOAD_ONE);
}

int main(void)
{
    test_gimbal_targets_wait_for_feedback_and_capture_current_pose();
    test_gimbal_targets_apply_increments_and_clamp_pitch();
    test_unready_gimbal_disables_angle_dependent_chassis_modes();
    test_friction_targets_and_readiness();
    test_loader_conversion_and_edges();
    test_loader_request_is_queued_until_ready_and_consumed_once();
    test_mecanum_signs();
    test_chassis_vector_limit_preserves_direction();
    test_control_link_requires_a_received_frame_and_live_watchdog();
    test_all_required_feedback_must_be_ready();
    test_wheel_test_respects_zero_force_and_wheel_targets_are_limited();
    test_loader_request_is_forced_to_stop_on_disabled_or_offline_shooter();
    test_loader_fault_sync_clears_pending_but_preserves_held_input();
    puts("infantry control tests passed");
    return 0;
}
