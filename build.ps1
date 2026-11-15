# Build and run the S1/S2 host tests with MinGW GCC (no CMake required).
# Produces SIX binaries:
#   test_s1      - Controller-role full suite (default profile)
#   test_device  - Device-role build: smoke + device-line E2E
#   matrix_c_full / matrix_c_min / matrix_d_full / matrix_d_min
#                - the 4-configuration CI matrix (role x profile) with the
#                  mandatory-subset parity test and the G-MEM RAM report.
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
New-Item -ItemType Directory -Force -Path build | Out-Null

$src = @('src\core.c','src\submit.c','src\results.c','src\ledger.c','src\executor.c','src\trace.c','src\proto.c','src\lineexec.c','src\mod4882.c','src\epoch.c','src\port_null.c')
$warn = @('-std=c11','-O2','-Wall','-Wextra','-Wpedantic','-Werror','-Wshadow','-Wundef','-Iinclude')
$min = @('-DGPIB_CFG_MAX_OPERATIONS=2','-DGPIB_CFG_MAX_BLOCKS=2','-DGPIB_CFG_TRACE_EVENTS=8','-DGPIB_CFG_DEVICE_EVENTS=2','-DGPIB_CFG_PROCESS_BUDGET=4')

# Controller-role full suite
gcc @warn @src `
    tests\test_main.c tests\test_addr.c tests\test_lifecycle.c tests\test_ledger.c tests\test_txn.c `
    tests\test_mbx.c tests\test_emergency.c tests\test_stream.c tests\test_vbus.c tests\test_trace.c tests\test_deadline.c `
    tests\test_interleave.c tests\test_litmus.c tests\test_proto.c tests\test_linesim.c tests\test_lineexec.c tests\test_tlc.c tests\test_spoll.c tests\test_devmode.c tests\test_dualcore.c tests\test_rldcdt.c tests\test_m4882.c tests\test_s2oracle.c tests\test_optfeatures.c tests\oracle_peer.c tests\test_oracle.c tests\test_stress.c `
    tests\mock_port.c tests\vbus.c tests\line_sim.c tests\mock_lines.c tests\mock_bus.c `
    -o build\test_s1.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& .\build\test_s1.exe
$rc1 = $LASTEXITCODE

# Device-role build: smoke + full device-line E2E against a mock controller
gcc @warn '-DGPIB_CFG_ROLE=GPIB_ROLE_DEVICE' '-DDEVLINE_DEVICE_BUILD' @src `
    tests\test_device_main.c tests\test_devmode.c tests\mock_ctrl.c tests\vbus.c `
    -o build\test_device.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& .\build\test_device.exe
$rc2 = $LASTEXITCODE

# 4-configuration CI matrix: role x profile, mandatory-subset parity + RAM.
gcc @warn '-DGPIB_MATRIX_PROFILE=full' @src tests\test_matrix_main.c -o build\matrix_c_full.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& .\build\matrix_c_full.exe
$rc3 = $LASTEXITCODE

gcc @warn @min '-DGPIB_MATRIX_PROFILE=min' @src tests\test_matrix_main.c -o build\matrix_c_min.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& .\build\matrix_c_min.exe
$rc4 = $LASTEXITCODE

gcc @warn '-DGPIB_CFG_ROLE=GPIB_ROLE_DEVICE' '-DGPIB_MATRIX_PROFILE=full' @src tests\test_matrix_main.c -o build\matrix_d_full.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& .\build\matrix_d_full.exe
$rc5 = $LASTEXITCODE

gcc @warn @min '-DGPIB_CFG_ROLE=GPIB_ROLE_DEVICE' '-DGPIB_MATRIX_PROFILE=min' @src tests\test_matrix_main.c -o build\matrix_d_min.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& .\build\matrix_d_min.exe
$rc6 = $LASTEXITCODE

# T14: compile-time border configuration (oversized ledger knobs) -- the
# storage/limits arithmetic must hold at the edges, not just the default.
gcc @warn '-DGPIB_CFG_MAX_LISTENERS=11' '-DGPIB_CFG_MAX_CMDS=13' @src tests\test_bigcfg_main.c tests\vbus.c tests\mock_lines.c -Itests -o build\test_bigcfg.exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& .\build\test_bigcfg.exe
$rc7 = $LASTEXITCODE

if ($rc1 -ne 0 -or $rc2 -ne 0 -or $rc3 -ne 0 -or $rc4 -ne 0 -or $rc5 -ne 0 -or $rc6 -ne 0 -or $rc7 -ne 0) { exit 1 }
exit 0
