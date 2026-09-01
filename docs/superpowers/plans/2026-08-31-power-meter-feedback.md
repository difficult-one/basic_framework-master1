# Power Meter Feedback Implementation Plan

> Execute inline in this task; the user has authorized implementation. Request an independent code review after implementation.

**Goal:** Make fresh chassis-branch meter measurements reliably participate in chassis power limiting without requiring referee buffer feedback.

**Architecture:** Keep existing motor models and allocation. Isolate time-dependent feedback calculation from hardware, publish coherent meter/referee snapshots, and consume a feedback mailbox in the motor task.

**Tech Stack:** STM32F407, C11/C++17, FreeRTOS, ARM GNU and host MinGW.

## Global Constraints

- No flashing, motor actuation, unrelated refactoring or changes to fitted coefficients.
- Meter timeout 100ms; referee power timeout 300ms; upward budget slew 80W/s; configurable timeouts in robot_def.h.
- Valid zero power and zero buffer are distinct from missing data.
- No usable Git metadata; retain changes without commits.

### Task 1: Feedback state and regression tests

Files: modules/powercontrol/power_feedback.h/.cpp, tests/power_feedback_test.cpp, tests/run_power_tests.ps1.

Interface: `ChassisPowerFeedback` contains independent power/buffer values, source, sequence, timestamp, timeout and validity. `PowerFeedbackController::update(input, now_ms, model_power_w, limit_w, idle_w)` returns status with effective limit, correction, attenuation and validity.

- [x] Write and run failing behavioral fixtures: meter 40W/model 160W/limit 80W without referee must retain initial 78.4W budget; valid zero buffer must reduce attenuation; repeated frame must not change correction; sample older than timeout must be invalid; UINT32 wrap must retain freshness; source switch must not increase budget by more than 0.08W per millisecond.
- [x] Implement finite-value validation, unsigned timestamp subtraction, per-source frame tracking, sample-time smoothing and independent buffer PI; hold correction on invalid/low power samples and cap upward budget rate.
- [x] Run `powershell -File tests/run_power_tests.ps1` and require success.

### Task 2: Hardware snapshots and integration

Files: modules/power_meter/power_meter.h/.c, modules/referee/rm_referee.h/.c, application/chassis/chassis.c/.h, application/robot_def.h, modules/powercontrol/power_manager_api.h/.cpp, Makefile.

- [x] Add meter timestamp/received marker/timeout and `PowerMeterGetSnapshot`; snapshot under task critical section, reject short frames without refreshing timestamp, keep last sample after daemon timeout without treating it as valid.
- [x] Publish referee power metadata only after valid CRC and exact payload length; expose `RefereeGetPowerSnapshot` under critical section. Keep metadata outside packed wire types.
- [x] Select fresh calibrated meter snapshot before referee; publish independent buffer validity and original sample times into `ChassisPower_SetFeedback(const ChassisPowerFeedback *)`.
- [x] Consume mailbox in motor task; replace former duplicated correction/buffer state with controller outputs while leaving allocation and limiters unchanged. Add source/validity/age debug fields.
- [x] Test actual meter driver with host HAL/CAN/daemon stubs, including short frame, zero, expiry and wrap. Build firmware with `make -j4`.

### Task 3: Review and delivery

- [x] Independent review focused on stale feedback, zero handling, ISR/task consistency and sudden budget increases; resolve actionable findings.
- [x] Re-run host tests and firmware build. Record exact results and remaining physical verification requirements in design document.
