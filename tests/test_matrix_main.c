#include <stdio.h>
#include <string.h>

/* Role- and profile-agnostic mandatory-subset test (REQ-006/R4: identical
 * mandatory functionality of minimal/full profiles for the role). Compiled
 * into FOUR CI binaries: {Controller, Device} x {default, minimal} knobs.
 * Uses only the null port: no line dependency. Also prints the G-MEM RAM
 * evidence: sizeof(struct gpib_context) for this configuration. */

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "test_util.h"

#ifndef GPIB_MATRIX_PROFILE
#define GPIB_MATRIX_PROFILE full
#endif

#define MMP_STR2(x) #x
#define MMP_STR(x) MMP_STR2(x)

#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
#define MANDATORY_OTHER_DEFINED
#else
#define MANDATORY_DEVICE_DEFINED
#endif

unsigned g_checks;
unsigned g_failures;

static uint8_t s_buf[8];

static void mandatory_common(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t ip;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_status_info_t info;
    gpib_trace_info_t ti;
    gpib_fault_snapshot_t snap;
    gpib_address_t good = { 7, 0, false };
    gpib_address_t bad = { 7, 1, true };
    uint8_t cmd = 0;

    memset(&ip, 0, sizeof(ip));
    ip.system_controller = false;
    ip.own_address.primary = 5;
    (void)t; (void)b;
    CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);

    /* Addressing surface: primary-only and secondary (TE/LE now supported
     * for 0..31); secondary > 31 is still rejected. */
    CHECK(gpib_set_address(&ctx, good) == GPIB_OK);
    bad.secondary = 32u; /* out of SAD encoding space */
    CHECK(gpib_set_address(&ctx, bad) == GPIB_UNSUPPORTED_ADDRESSING);

    /* Lifecycle + diagnostics surface. */
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_RUNNING);
    CHECK(gpib_trace_query(&ctx, &ti) == GPIB_OK);
    CHECK(ti.count > 0);
    CHECK(gpib_poll_device_event(&ctx, &cmd) == GPIB_NO_RESULT);
    CHECK(gpib_emergency_stop(&ctx) == GPIB_OK);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.fault_code == GPIB_FAULT_EMERGENCY_STOP);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

#ifndef MANDATORY_DEVICE_DEFINED
static void mandatory_controller(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t ip;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = true;
    ip.own_address.primary = 5;
    CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);

    /* Remote-addressed operations exist in every controller profile; the
     * queued-cancel path needs no port bytes. */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_buf;
    r.tx_len = sizeof(s_buf);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED || res.reason == GPIB_TERM_CANCELED);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);

    /* Serial poll and control transfer submit cleanly and cancel. */
    CHECK(gpib_serial_poll(&ctx, 6, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_pass_control(&ctx, 6, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);

    /* Device-role API is refused in the controller build. */
    CHECK(gpib_device_supply(&ctx, s_buf, sizeof(s_buf), &t, &b) ==
          GPIB_UNSUPPORTED);
    CHECK(gpib_device_sink(&ctx, s_buf, sizeof(s_buf), &t, &b) ==
          GPIB_UNSUPPORTED);

    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}
#endif /* controller only */

#ifndef MANDATORY_OTHER_DEFINED
static void mandatory_device(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t ip;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;

    memset(&ip, 0, sizeof(ip));
    ip.system_controller = false;
    ip.own_address.primary = 5;
    CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_INVALID_STATE); /* repeated start */

    /* Device exchanges submit and cancel cleanly in every profile. */
    CHECK(gpib_device_supply(&ctx, s_buf, sizeof(s_buf), &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED || res.reason == GPIB_TERM_CANCELED);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_device_sink(&ctx, s_buf, sizeof(s_buf), &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);

    /* Controller-initiated operations are refused in the device build. */
    {
        gpib_request_t r;
        memset(&r, 0, sizeof(r));
        r.kind = GPIB_REQ_WRITE;
        r.address.primary = 6;
        r.tx_data = s_buf;
        r.tx_len = sizeof(s_buf);
        CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_UNSUPPORTED);
    }
    CHECK(gpib_serial_poll(&ctx, 6, &t, &b) == GPIB_UNSUPPORTED);
    CHECK(gpib_pass_control(&ctx, 6, &t, &b) == GPIB_UNSUPPORTED);

    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}
#endif /* device only */

int main(void) {
    mandatory_common();
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    mandatory_controller();
    printf("gpib-core MATRIX Controller/%s: checks=%u failures=%u "
        "ctx=%zu bytes\n", MMP_STR(GPIB_MATRIX_PROFILE), g_checks,
        g_failures, sizeof(struct gpib_context));
#else
    mandatory_device();
    printf("gpib-core MATRIX Device/%s: checks=%u failures=%u "
        "ctx=%zu bytes\n", MMP_STR(GPIB_MATRIX_PROFILE), g_checks,
        g_failures, sizeof(struct gpib_context));
#endif
    return (g_failures == 0u) ? 0 : 1;
}
