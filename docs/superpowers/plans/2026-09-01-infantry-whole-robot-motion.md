# Infantry Whole-Robot Motion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Safely enable the mecanum chassis, GM6020 gimbal, friction wheels, and M2006 loader while leaving the lid servo disabled.

**Architecture:** Keep the existing `robot_cmd` publisher and `chassis`, `gimbal`, and `shoot` subscribers. Put deterministic safety calculations in a small `infantry_control` module so host tests can exercise startup target capture, pitch clamping, mode fallback, friction speed selection, readiness, and loader conversion without HAL dependencies.

**Tech Stack:** STM32F407, C11, FreeRTOS/CMSIS-RTOS, DJI CAN motors, PowerShell host-test runner, GCC/G++ host tests, GNU Arm Embedded firmware build.

## Global Constraints

- Single RoboMaster C board; four M3508 mecanum drive motors; two GM6020 gimbal motors; two M3508 friction motors; one M2006 loader.
- The lid servo remains uninitialized and produces no PWM.
- Startup and offline behavior must command zero force or zero speed.
- Initial pitch travel is limited to 15 degrees above and below the captured power-on pitch.
- Initial friction speeds are conservative no-projectile commissioning values, not calibrated muzzle-speed claims.
- Hardware validation is performed with the chassis suspended and no projectiles installed.
- Do not create Git commits, per the user's latest instruction.

---

### Task 1: Pure infantry control policies

**Files:**
- Create: `application/infantry_control.h`
- Create: `application/infantry_control.c`
- Create: `tests/infantry_control_test.c`
- Modify: `tests/run_power_tests.ps1`
- Modify: `Makefile`

**Interfaces:**
- Produces `InfantryGimbalTargets`, `InfantryGimbalTargetsReset`, `InfantryGimbalTargetsStep`, `InfantrySafeChassisMode`, `InfantryFrictionTarget`, `InfantryFrictionReady`, `InfantryLoaderBurstSpeed`, and `InfantryLoadModeRisingEdge`.

- [ ] Write tests for first-feedback target capture, incremental motion, ±15-degree pitch clamping, unsafe chassis-mode fallback, all friction speed selections, friction readiness tolerance, loader conversion using `NUM_PER_CIRCLE`, and single-shot rising edges.
- [ ] Run the new test binary and verify compilation fails because `infantry_control.h` and its functions do not exist.
- [ ] Implement the minimal pure functions and parameter macros required by the tests.
- [ ] Add the source to the firmware Makefile and the test to `run_power_tests.ps1`.
- [ ] Run the host suite and verify all tests pass.

### Task 2: Feedback validity and safe gimbal startup

**Files:**
- Modify: `modules/motor/motor_def.h`
- Modify: `modules/motor/DJImotor/dji_motor.h`
- Modify: `modules/motor/DJImotor/dji_motor.c`
- Modify: `application/robot_def.h`
- Modify: `application/gimbal/gimbal.c`
- Modify: `application/cmd/robot_cmd.c`
- Modify: `application/robot.c`

**Interfaces:**
- Produces `DJIMotorIsOnline(const DJIMotorInstance *)` and `Gimbal_Upload_Data_s.gimbal_ready`.
- Consumes the target and chassis-mode policies from Task 1.

- [ ] Add a test assertion that a false gimbal-ready input preserves zero-force gimbal control and converts follow/rotate requests to no-follow.
- [ ] Run the test and verify it fails on the missing policy behavior.
- [ ] Record whether each DJI motor has received at least one feedback frame and expose an online query that combines that flag with its daemon.
- [ ] Make the gimbal publish readiness only when yaw and pitch feedback have both been received; keep motors stopped until the command mode becomes active.
- [ ] Capture current IMU yaw/pitch on first valid feedback, clamp later pitch increments, and reset capture after emergency/offline state.
- [ ] Enable `GimbalInit()` and `GimbalTask()` in the whole-robot entry point.
- [ ] Run host tests and `make -j4`.

### Task 3: Safe friction-wheel and loader operation

**Files:**
- Modify: `application/robot_def.h`
- Modify: `application/shoot/shoot.c`

**Interfaces:**
- Consumes `InfantryFrictionTarget`, `InfantryFrictionReady`, `InfantryLoaderBurstSpeed`, and `InfantryLoadModeRisingEdge`.
- Produces `Shoot_Upload_Data_s.friction_ready` for telemetry.

- [ ] Add tests that each supported bullet-speed enum maps to a nonzero commissioning target, that tolerance is symmetric, and that a held single-shot mode only produces one rising edge.
- [ ] Run tests and verify the new expectations fail before implementation.
- [ ] Configure the loader as M2006 with centralized CAN ID and direction macros.
- [ ] Apply nonzero commissioning targets to the friction motors and track readiness for the configured hold time.
- [ ] Force loader stop until both friction motors are stable; implement edge-triggered single/three-shot commands and use `NUM_PER_CIRCLE` for burst conversion.
- [ ] Keep all lid-control branches output-free.
- [ ] Run host tests and `make -j4`.

### Task 4: Four-wheel direction verification support

**Files:**
- Modify: `application/robot_def.h`
- Modify: `application/chassis/chassis.c`
- Modify: `application/chassis/chassis.md`

**Interfaces:**
- Uses existing `CHASSIS_WHEEL_TEST_MODE` and a selected LF/RF/LB/RB target.
- Produces clear UART2 feedback for the selected physical wheel.

- [ ] Add a host-test table for expected mecanum wheel signs during forward, strafe, and rotation commands.
- [ ] Run the test and verify it fails until the pure mecanum calculation is exposed through Task 1.
- [ ] Move the mecanum sign calculation into the tested pure helper and make `chassis.c` call it.
- [ ] Print the selected wheel name, target speed, measured speed, and current instead of always printing M1/LF.
- [ ] Document the suspended-chassis LF → RF → LB → RB verification sequence and the exact compile-time selector values.
- [ ] Run the full host suite and firmware build.

### Task 5: Final verification and documentation consistency

**Files:**
- Modify: `README.md`
- Modify: `application/gimbal/gimbal.md`
- Modify: `application/shoot/shoot.md`
- Modify: `docs/superpowers/specs/2026-09-01-infantry-whole-robot-motion-design.md`

**Interfaces:**
- Documents all configuration macros and safe hardware commissioning steps.

- [ ] Update documentation to match the implemented motor types, controls, safety interlocks, pitch limit, and friction calibration disclaimer.
- [ ] Correct the hardware-validation sequence so loader testing occurs only after friction readiness, with no projectiles installed.
- [ ] Run `./tests/run_power_tests.ps1` and require every binary to pass.
- [ ] Run `make -j4` and require successful ELF, HEX, and BIN output with no new warnings.
- [ ] Review the final diff for unrelated changes and report any parameters that still require physical calibration.
