---
name: power-manager-integration
description: Replacing old power_control.c with new C++ power_manager in the framework
metadata:
  type: project
---

Replaced old `modules/motor/power_control.c` (PowerControl function) with new C++ `modules/powercontrol/power_manager.cpp` (MotorPower + ChassisPowerManager classes).

## Files modified (7 total):
1. `modules/motor/DJImotor/dji_motor.h` — added `float output_current` field + `DJIMotorTransmit()` declaration
2. `modules/motor/DJImotor/dji_motor.c` — split `DJIMotorControl()` (PID only, store in output_current) from new `DJIMotorTransmit()` (fill tx_buff + CAN send)
3. `modules/motor/motor_task.c` — inserted `ChassisPower_Update()` between PID compute and CAN transmit; added `#include "power_manager_api.h"`
4. `modules/powercontrol/power_manager_api.h` — NEW C-compatible wrapper header
5. `modules/powercontrol/power_manager_api.cpp` — NEW C++/C bridge implementation (ChassisPower_Init, ChassisPower_UpdateError, ChassisPower_Update, ChassisPower_SetLimit, ChassisPower_GetPredictPower)
6. `application/chassis/chassis.c` — added `ChassisPower_Init()` in init, `ChassisPower_UpdateError()` after LimitChassisOutput, `ChassisPower_SetLimit()` in ChassisTask
7. `Makefile` — added CXX (g++) compiler, CXX_SOURCES, CXXFLAGS (-std=c++17), -lstdc++, .cpp build rule, powercontrol include path

## Data flow:
```
ChassisTask (200Hz):
  DJIMotorSetRef() → sets motor->motor_controller.pid_ref
  ChassisPower_UpdateError() → updates error weights

MotorTask (1kHz):
  DJIMotorControl() → PID → motor->output_current
  ChassisPower_Update() → power model + allocate + limiter() → modifies output_current
  DJIMotorTransmit() → output_current → tx_buff → CANTransmit
```

## TODO:
- Run Fitting.py to get proper M3508 coefficients for `m3508_fitting` in power_manager_api.cpp
- The old `PowerControl()` in motor_task.c is still called but does nothing (no PowerControlInit motors)
- power_control.c can be removed from Makefile and deleted once verified working

## To revert:
- Restore dji_motor.h, dji_motor.c, motor_task.c, chassis.c, Makefile from git
- Delete power_manager_api.h and power_manager_api.cpp
