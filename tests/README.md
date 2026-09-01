# Power feedback host tests

Run from the project root in PowerShell:

```powershell
./tests/run_power_tests.ps1
```

Requires host `gcc` and `g++` on PATH. Executables are written under `build/host-tests`.

- `power_feedback_test.cpp` runs the real feedback controller, with hand-checked budgets and time/validity cases.
- `power_drivers_test.c` compiles the real meter/referee parsers and real referee CRC implementation. Only HAL/RTOS and CAN/USART registration are stubbed; wire protocol and module headers are production headers. Assertions fail via `exit`, without Windows assertion dialogs.
- `infantry_control_test.c` verifies safe gimbal target capture, pitch limiting, chassis mode fallback and speed limiting, mecanum signs, friction direction/readiness, M2006 loader conversion, and one-shot request consumption.
- `remote_control_contract_test.c` verifies that normal keyboard state reads the current key frame and that invalid DBUS channel/switch values are rejected.
- Host tests are single-threaded. They do not simulate ARM interrupt masking, motor dynamics or bus latency. Check these on hardware; see the power feedback design document.

Firmware build: `make -j4`. No test or build command flashes a board.
