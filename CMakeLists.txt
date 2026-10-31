cmake_minimum_required(VERSION 3.16)
project(gpibcore C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(GPIB_ROLE "CONTROLLER" CACHE STRING "Compile-time role: CONTROLLER or DEVICE")
set_property(CACHE GPIB_ROLE PROPERTY STRINGS CONTROLLER DEVICE)

set(GPIB_CORE_SRCS
    src/core.c
    src/submit.c
    src/results.c
    src/ledger.c
    src/executor.c
    src/trace.c
    src/proto.c
    src/lineexec.c
    src/mod4882.c
    src/epoch.c
    src/port_null.c)

add_library(gpibcore STATIC ${GPIB_CORE_SRCS})
target_include_directories(gpibcore PUBLIC include)
target_compile_definitions(gpibcore PUBLIC GPIB_CFG_ROLE=GPIB_ROLE_${GPIB_ROLE})

if(MSVC)
    target_compile_options(gpibcore PRIVATE /W4 /WX)
else()
    target_compile_options(gpibcore PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wundef)
endif()

add_executable(test_s1
    tests/test_main.c
    tests/test_addr.c
    tests/test_lifecycle.c
    tests/test_ledger.c
    tests/test_txn.c
    tests/test_mbx.c
    tests/test_emergency.c
    tests/test_stream.c
    tests/test_vbus.c
    tests/test_trace.c
    tests/test_deadline.c
    tests/test_interleave.c
    tests/test_litmus.c
    tests/test_proto.c
    tests/test_linesim.c
    tests/test_lineexec.c
    tests/test_tlc.c
    tests/test_spoll.c
    tests/test_devmode.c
    tests/test_dualcore.c
    tests/test_rldcdt.c
    tests/test_m4882.c
    tests/test_s2oracle.c
    tests/test_optfeatures.c
    tests/oracle_peer.c
    tests/test_oracle.c
    tests/mock_port.c
    tests/vbus.c
    tests/line_sim.c
    tests/mock_lines.c
    tests/mock_bus.c)
target_include_directories(test_s1 PRIVATE tests)
target_link_libraries(test_s1 PRIVATE gpibcore)
if(NOT MSVC)
    target_compile_options(test_s1 PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wundef)
endif()

# Device-role build: API smoke + full device-line E2E against a mock
# controller (test_devmode selects its surface via DEVLINE_DEVICE_BUILD).
add_executable(test_device ${GPIB_CORE_SRCS}
    tests/test_device_main.c
    tests/test_devmode.c
    tests/mock_ctrl.c
    tests/vbus.c)
target_include_directories(test_device PRIVATE include tests)
target_compile_definitions(test_device PRIVATE
    GPIB_CFG_ROLE=GPIB_ROLE_DEVICE DEVLINE_DEVICE_BUILD)
if(MSVC)
    target_compile_options(test_device PRIVATE /W4 /WX)
else()
    target_compile_options(test_device PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wundef)
endif()

enable_testing()
add_test(NAME s1 COMMAND test_s1)
add_test(NAME device_smoke COMMAND test_device)

# 4-configuration CI matrix: role x profile, mandatory-subset parity + RAM.
set(MIN_KNOBS GPIB_CFG_MAX_OPERATIONS=2 GPIB_CFG_MAX_BLOCKS=2
    GPIB_CFG_TRACE_EVENTS=8 GPIB_CFG_DEVICE_EVENTS=2 GPIB_CFG_PROCESS_BUDGET=4)

add_executable(matrix_c_full ${GPIB_CORE_SRCS} tests/test_matrix_main.c)
target_compile_definitions(matrix_c_full PRIVATE GPIB_MATRIX_PROFILE=full)
add_executable(matrix_c_min ${GPIB_CORE_SRCS} tests/test_matrix_main.c)
target_compile_definitions(matrix_c_min PRIVATE GPIB_MATRIX_PROFILE=min ${MIN_KNOBS})
add_executable(matrix_d_full ${GPIB_CORE_SRCS} tests/test_matrix_main.c)
target_compile_definitions(matrix_d_full PRIVATE
    GPIB_CFG_ROLE=GPIB_ROLE_DEVICE GPIB_MATRIX_PROFILE=full)
add_executable(matrix_d_min ${GPIB_CORE_SRCS} tests/test_matrix_main.c)
target_compile_definitions(matrix_d_min PRIVATE
    GPIB_CFG_ROLE=GPIB_ROLE_DEVICE GPIB_MATRIX_PROFILE=min ${MIN_KNOBS})
foreach(mtx matrix_c_full matrix_c_min matrix_d_full matrix_d_min)
    target_include_directories(${mtx} PRIVATE include tests)
    if(MSVC)
        target_compile_options(${mtx} PRIVATE /W4 /WX)
    else()
        target_compile_options(${mtx} PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wshadow -Wundef)
    endif()
    add_test(NAME ${mtx} COMMAND ${mtx})
endforeach()

# T14: compile-time border configuration -- the limits arithmetic must
# hold at the edges, and CI must RUN it, not just build.ps1.
add_executable(test_bigcfg ${GPIB_CORE_SRCS}
    tests/test_bigcfg_main.c tests/vbus.c tests/mock_lines.c)
target_include_directories(test_bigcfg PRIVATE include tests)
target_compile_definitions(test_bigcfg PRIVATE
    GPIB_CFG_MAX_LISTENERS=11 GPIB_CFG_MAX_CMDS=13)
if(MSVC)
    target_compile_options(test_bigcfg PRIVATE /W4 /WX)
else()
    target_compile_options(test_bigcfg PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wundef)
endif()
add_test(NAME bigcfg COMMAND test_bigcfg)

# R9 §4: the oracle suite also runs as its OWN CTest target (suite #24)
# -- a mutant killed by an earlier suite must not mask the oracle's
# sensitivity in the aggregate run.
add_test(NAME oracle COMMAND test_s1 24)
