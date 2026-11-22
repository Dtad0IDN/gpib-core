# gpib-core

[English](README.md) | [Русский](README_RU.md)

Portable IEEE 488.1/488.2 (GPIB) protocol core for MCU targets.

**[Quickstart (EN)](QUICKSTART.md) | [Quickstart (RU)](QUICKSTART_RU.md)**

- **Language:** C11, zero heap, single static context (2408/1216 bytes full/min)
- **Roles:** Controller and Device from one codebase (`GPIB_CFG_ROLE`)
- **Standards:** IEEE 488.1-1987 (all interface functions) + IEEE 488.2-1992 (controller protocols, module E helper)
- **Verification:** 177 199 checks, 0 failures; independent wire oracle + chaos stress + mutation-tested regressions

## Quick start

```
powershell -ExecutionPolicy Bypass -File build.ps1
```

Seven binaries, all must exit 0:

| Binary | Checks | Purpose |
|---|---|---|
| `test_s1` | 177 199 | Controller-role: 25 suites (lifecycle, ledger, FSMs, E2E, oracle, chaos stress) |
| `test_device` | 666 | Device-role E2E against a simulated controller |
| `matrix_c/d_full/min` | 4 x 35 | Role x profile CI matrix with RAM report |
| `test_bigcfg` | 14 | Compile-time border configuration |

Single suite: `build\test_s1.exe --suite oracle` (or `stress`, `lineexec`, etc.)

Linux/CMake: `cmake -S . -B build && cmake --build build && ctest`

## Architecture

```
Application
    ?
    ?
???????????????????????????????????????????
?  gpib.h � public API (submit/process/   ?
?  ack/peek/cancel/lifecycle)             ?
???????????????????????????????????????????
?  submit.c    ?  executor.c  ?  ledger.c ?
?  (validation,?  (action     ?  (txn/block?
?   staging)   ?   lifecycle) ?   records) ?
???????????????????????????????????????????
?  lineexec.c � line-level sequencer      ?
?  (phases, drive composer, detach)       ?
???????????????????????????????????????????
?  proto.c � IEEE 488.1 interface FSMs    ?
?  (SH AH T/TE L/LE SR RL DC DT C PP)     ?
???????????????????????????????????????????
?  PORT (gpib_port.h) � the ONLY hardware ?
?  contact: read_lines/drive/drive_data/  ?
?  poll_event/now_us + G1-G5 guarantees   ?
???????????????????????????????????????????
```

The core never touches GPIO directly. Everything hardware-specific goes through
the port callback interface. A real adapter (STM32, ESP32, bit-bang) implements
the five callbacks; the core is platform-independent C11.

## File layout

| Path | Purpose |
|---|---|
| `include/gpib/gpib.h` | Public API (Doxygen) |
| `include/gpib/gpib_port.h` | Port contract + G1-G5 guarantees |
| `include/gpib/gpib_types.h` | Status codes, enums, handles, results |
| `include/gpib/gpib_cfg.h` | Compile-time knobs (role, sizes, timings) |
| `include/gpib/gpib_proto.h` | FSM types + encode helpers |
| `include/gpib/gpib_storage.h` | Context layout + fingerprint |
| `include/gpib/gpib_4882.h` | Module E helper API |
| `src/` | 11 implementation files (see @file docs) |
| `tests/` | 25 suites, 45+ files |
| `docs/norms/` | Verified normative extracts |
| `doc/html/` | Generated Doxygen documentation |

## API overview

```c
#include "gpib/gpib.h"

GPIB_CONTEXT_STORAGE_DEFINE(ctx);     /* static placement */
gpib_init(&ctx, &(gpib_init_params_t){
    .system_controller = true,
    .own_address = { .primary = 5 },
});
gpib_start(&ctx);

/* Write */
gpib_handle_t t, b;
gpib_request_t r = { .kind = GPIB_REQ_WRITE,
                     .address = { .primary = 6 },
                     .tx_data = "*IDN?\n", .tx_len = 6 };
gpib_submit(&ctx, &r, &t, &b);

/* Pump */
gpib_result_t res;
while (gpib_result_peek(&ctx, t, &res) != GPIB_OK)
    gpib_process(&ctx);

/* Result */
if (res.reason == GPIB_TERM_COMPLETED)
    /* res.bytes_confirmed == 6 */;

/* Ack (ownership returns) */
gpib_result_ack(&ctx, b);
gpib_result_ack(&ctx, t);

gpib_request_shutdown(&ctx);
gpib_process(&ctx);
gpib_deinit(&ctx);
```

## Verification layers

1. **FSM vectors** � directed tests transcribed from normative extracts (`docs/norms/`)
2. **S2 oracle** � exhaustive-input cross-check of SH/AH transition AND output tables
3. **E2E mock instruments** � FSM-shared integration tests
4. **Core-to-core** � two real instances exchanging TCT on one shared bus
5. **Independent wire oracle** (`tests/oracle_peer.c`) � zero shared protocol code; found 3 real bugs the FSM-shared tests couldn't see
6. **Chaos stress** � adversarial bus (random IFC/ATN-steal/SRQ) + seeded random workload + two-sided byte accounting

## Configuration for weak MCUs

```c
/* In gpib_cfg.h or via -D flags: */
#define GPIB_CFG_PROCESS_BUDGET 1     /* one exchange per process() call */
#define GPIB_CFG_TRACE_EVENTS 0      /* disable trace: saves code + RAM */
#define GPIB_CFG_MAX_OPERATIONS 2    /* 2 txn slots for simple device */
#define GPIB_CFG_MAX_BLOCKS 2       /* 1 block per operation */
```

This yields ~800 bytes context, ~6-8 KB flash (device-role min profile).

## Documentation

```
tools\doxygen\doxygen.exe Doxyfile
```

Generates `doc/html/index.html` (173 pages) covering all public headers,
API functions, and internal implementation.

## Status

Host-level implementation is complete and audit-hardened. Both roles pass
all tests. Remaining work is hardware-specific:

- Level-2 port adapters (STM32H743, ESP32, bit-bang)
- VT bench validation (real transceiver timings T1-T10)
- Independent multi-controller takeover testing

## Normative basis

`docs/norms/` contains verified engineering extracts from IEEE 488.1-1987
and IEEE 488.2-1992, used as the ground truth for all FSM implementations.
See the table in the norms directory for coverage details.

## License

TBD
