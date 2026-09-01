$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force build/host-tests | Out-Null
    & g++ -std=c++17 -Wall -Wextra -Werror -Imodules/powercontrol tests/power_feedback_test.cpp modules/powercontrol/power_feedback.cpp -o build/host-tests/power_feedback_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Feedback test compilation failed' }
    & ./build/host-tests/power_feedback_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Feedback tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -Wno-unused-parameter -Itests/stubs -Imodules/power_meter -Imodules/referee -Imodules/daemon -Ibsp/can -Ibsp/usart tests/power_drivers_test.c modules/power_meter/power_meter.c modules/referee/rm_referee.c modules/referee/crc_ref.c -o build/host-tests/power_drivers_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Driver test compilation failed' }
    & ./build/host-tests/power_drivers_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Driver tests failed' }
} finally {
    Pop-Location
}
