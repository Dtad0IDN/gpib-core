# Quickstart

From zero to talking on the GPIB bus in 15 minutes.

## 1. Prerequisites

- **Windows:** MinGW GCC (`gcc --version` works)
- **Linux/macOS:** GCC or Clang, CMake 3.16+
- No external dependencies. No packages to install.

## 2. Build and verify

```bash
git clone <repo> && cd gpib-core

# Windows
powershell -ExecutionPolicy Bypass -File build.ps1

# Linux
cmake -S . -B build && cmake --build build && cd build && ctest
```

All tests must pass (exit code 0). You should see:
```
S1/S2: checks=177199 failures=0
DEVICE: checks=666 failures=0
```

## 3. Link into your project

Copy `include/gpib/` and `src/*.c` into your build. That's it —
no library to build, just compile the sources alongside your code.

```cmake
# CMakeLists.txt example
add_library(gpib-core STATIC
    src/core.c src/submit.c src/results.c src/ledger.c
    src/executor.c src/trace.c src/proto.c src/lineexec.c
    src/mod4882.c src/epoch.c src/port_null.c
)
target_include_directories(gpib-core PUBLIC include)
```

## 4. Implement a port (the ONLY hardware code)

The core talks to the bus through 5 callbacks. Here's a minimal
byte-pump port (for testing; a real port uses `read_lines`/`drive`):

```c
#include "gpib/gpib_port.h"

static bool my_arm(void *user) { return true; }
static bool my_detached(void *user) { return true; }
static bool my_quiesced(void *user) { return true; }
static void my_inhibit(void *user) { /* release all GPIO */ }
static uint64_t my_now_us(void *user) { return get_timer_us(); }
static size_t my_transfer(void *u, gpib_dir_t d, uint8_t *buf, size_t cap) {
    /* Your GPIO handshake here. Return bytes moved. */
    return 0;
}
static bool my_poll_event(void *u, gpib_port_event_t *out) { return false; }

static const gpib_port_t my_port = {
    .name = "my-gpib",
    .arm = my_arm,
    .safe_detached = my_detached,
    .executor_quiesced = my_quiesced,
    .emergency_inhibit = my_inhibit,
    .now_us = my_now_us,
    .transfer = my_transfer,
    .poll_event = my_poll_event,
};
```

For a real bus you also implement the line-level callbacks:

```c
static bool my_read_lines(void *u, gpib_line_snap_t *out) {
    /* Read all 16 lines (8 DIO + 8 control) in ONE atomic snapshot. */
    out->dav  = !gpio_get(DAV_PIN);   /* active-low */
    out->nrfd = !gpio_get(NRFD_PIN);
    out->ndac = !gpio_get(NDAC_PIN);
    out->atn  = !gpio_get(ATN_PIN);
    out->eoi  = !gpio_get(EOI_PIN);
    out->srq  = !gpio_get(SRQ_PIN);
    out->ifc  = !gpio_get(IFC_PIN);
    out->ren  = !gpio_get(REN_PIN);
    out->dio  = gpio_read_bus(DIO_PORT);
    out->now_us = get_timer_us();
    return true;
}

static void my_drive(void *u, const gpib_line_drive_t *req) {
    gpio_set(DAV_PIN, !req->dav);    /* inverted: asserted = LOW */
    gpio_set(NRFD_PIN, !req->nrfd);
    gpio_set(NDAC_PIN, !req->ndac);
    gpio_set(ATN_PIN, !req->atn);
    gpio_set(EOI_PIN, !req->eoi);
    gpio_set(REN_PIN, !req->ren);
    /* Never drive IFC/SRQ from here — the core handles those. */
}

static void my_drive_data(void *u, const uint8_t *dio) {
    if (dio) {
        gpio_bus_output(DIO_PORT);
        gpio_write_bus(DIO_PORT, *dio);
    } else {
        gpio_bus_input(DIO_PORT);    /* release: open-collector */
    }
}
```

## 5. Controller: talk to an instrument

```c
#include "gpib/gpib.h"

int main(void) {
    /* 1. Create and start the instance */
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init(&ctx, &(gpib_init_params_t){
        .system_controller = true,
        .own_address = { .primary = 0 },   /* our GPIB address */
        .port = &my_port,
    });
    gpib_start(&ctx);

    /* 2. Send *IDN? to the instrument at address 5 */
    gpib_handle_t t, b;
    gpib_request_t w = {
        .kind = GPIB_REQ_WRITE,
        .address = { .primary = 5 },
        .tx_data = "*IDN?\n",
        .tx_len = 6,
        .deadline_us = 0,   /* no timeout */
    };
    gpib_submit(&ctx, &w, &t, &b);

    gpib_result_t res;
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);          /* call from main loop or timer IRQ */
    }
    /* res.reason == GPIB_TERM_COMPLETED, res.bytes_confirmed == 6 */
    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    /* 3. Read the response */
    char buf[128] = {0};
    gpib_request_t r = {
        .kind = GPIB_REQ_READ,
        .address = { .primary = 5 },
        .rx_data = buf,
        .rx_cap = sizeof(buf) - 1,
    };
    gpib_submit(&ctx, &r, &t, &b);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);
    }

    if (res.reason == GPIB_TERM_COMPLETED) {
        /* buf contains res.bytes_confirmed bytes of the IDN string */
        printf("IDN: %.*s\n", (int)res.bytes_confirmed, buf);
    }

    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    /* 4. Shut down cleanly */
    gpib_request_shutdown(&ctx);
    gpib_process(&ctx);
    gpib_deinit(&ctx);
    return 0;
}
```

## 6. Device: respond to a controller

```c
int main(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init(&ctx, &(gpib_init_params_t){
        .system_controller = false,          /* we're a device */
        .own_address = { .primary = 5 },
        .port = &my_port,
    });
    gpib_start(&ctx);

    /* Wait for the controller to address us and send data */
    char rxbuf[256];
    gpib_handle_t t, b;
    gpib_device_sink(&ctx, rxbuf, sizeof(rxbuf), &t, &b);

    gpib_result_t res;
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);
        /* Check for device commands (GET, DCL, etc.) */
        uint8_t cmd;
        while (gpib_poll_device_event(&ctx, &cmd) == GPIB_OK) {
            /* Handle trigger, clear, remote/local... */
        }
    }

    if (res.reason == GPIB_TERM_COMPLETED) {
        /* rxbuf[0..res.bytes_confirmed-1] contains the received data */
    }
    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    /* Supply a response when talk-addressed */
    gpib_device_supply(&ctx, "HELLO\n", 6, &t, &b);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);
    }
    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    gpib_request_shutdown(&ctx);
    gpib_process(&ctx);
    gpib_deinit(&ctx);
    return 0;
}
```

## 7. What's next

- Read the [API documentation](doc/html/index.html) (generated by Doxygen)
- Look at `tests/test_lineexec.c` for complete E2E examples
- Look at `tests/oracle_peer.c` for an independent port implementation
- Check `include/gpib/gpib_cfg.h` for size/speed tuning knobs
- Read the [port contract](include/gpib/gpib_port.h) for G1-G5 guarantees
