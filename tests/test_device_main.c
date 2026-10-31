#include <string.h>

/* Device-role build tests: the API smoke surface plus the full device-line
 * E2E suite (suite_devmode) against a simulated external controller.
 * Compiled with GPIB_CFG_ROLE=DEVICE and DEVLINE_DEVICE_BUILD. */

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "test_util.h"

unsigned g_checks;
unsigned g_failures;

void suite_devmode(void);

static uint8_t s_tx[8];

int main(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_status_info_t info;
    gpib_trace_info_t tinfo;

    memset(&p, 0, sizeof(p));
    p.system_controller = false; /* a Device is never the system controller */
    p.own_address.primary = 5;
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_RUNNING);

    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = sizeof(s_tx);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_UNSUPPORTED);
    r.kind = GPIB_REQ_WRITE_MANY;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_UNSUPPORTED);

    /* Own-address configuration: secondary 1 is now valid (TE/LE).
     * Only values > 31 are out of the SAD encoding space. */
    {
        gpib_address_t good = { 9, 0, false };
        gpib_address_t sec_ok = { 9, 1, true };
        gpib_address_t bad = { 9, 32, true };
        CHECK(gpib_set_address(&ctx, good) == GPIB_OK);
        CHECK(gpib_set_address(&ctx, sec_ok) == GPIB_OK);
        CHECK(gpib_set_address(&ctx, bad) == GPIB_UNSUPPORTED_ADDRESSING);
    }
    CHECK(gpib_recover(&ctx) == GPIB_OK); /* idle RUNNING: allowed */
    CHECK(gpib_trace_query(&ctx, &tinfo) == GPIB_OK);
    CHECK(tinfo.count > 0);
    {
        uint8_t cmd = 0;
        CHECK(gpib_poll_device_event(&ctx, &cmd) == GPIB_NO_RESULT);
    }
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);

    /* Full device-line E2E: supply/sink/service channel. */
    suite_devmode();

    printf("gpib-core DEVICE: checks=%u failures=%u\n", g_checks,
        g_failures);
    return (g_failures == 0u) ? 0 : 1;
}
