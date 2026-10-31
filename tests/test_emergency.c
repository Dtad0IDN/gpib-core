#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_port.h"
#include "test_util.h"

static uint8_t s_tx[100];

/* LIF-09: emergency stop from INITIALIZED and RUNNING is idempotent, never
 * performs IFC/DCL, latches a fault and reports it in the snapshot. */
static void emg_01_basic(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_init_params_t p;
    gpib_fault_snapshot_t snap;
    gpib_status_info_t info;
    mock_init(&m);
    memset(&p, 0, sizeof(p));
    p.system_controller = true;
    p.own_address.primary = 5;
    p.port = &m.base;
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_emergency_stop(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_STOPPING);
    CHECK(info.fault_latched);
    CHECK(m.inhibit_called);
    CHECK(m.tx_accepted == 0); /* no bus traffic after the inhibit */
    CHECK(gpib_emergency_stop(&ctx) == GPIB_ALREADY_STOPPING);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.fault_code == GPIB_FAULT_EMERGENCY_STOP);
    CHECK(snap.snapshot_valid);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* Emergency stop mid-TX: every block gets one terminal record with the fault
 * cause and an uncertain last byte; the instance still drains and deinits. */
static void emg_02_mid_tx(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    mock_init(&m);
    memset(&r, 0, sizeof(r));
    memset(s_tx, 0x5A, sizeof(s_tx));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 100;
    {
        gpib_init_params_t p;
        memset(&p, 0, sizeof(p));
        p.system_controller = true;
        p.own_address.primary = 5;
        p.port = &m.base;
        CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    }
    CHECK(gpib_start(&ctx) == GPIB_OK);
    r.tx_data = s_tx;
    r.tx_len = 100;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* partial */
    CHECK(gpib_emergency_stop(&ctx) == GPIB_OK);
    CHECK(m.inhibit_called);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_FAULT);
    CHECK(res.bytes_confirmed > 0);
    CHECK(res.bytes_confirmed < 100);
    CHECK(res.last_byte_uncertain);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_FAULT);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* RCV-06 analog: the diagnostic snapshot is available while the instance is
 * in STOPPING, without allocation, IO or waiting. */
static void emg_03_snapshot_in_stopping(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_fault_snapshot_t snap;
    mock_init(&m);
    memset(&r, 0, sizeof(r));
    memset(s_tx, 0x33, sizeof(s_tx));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 100;
    {
        gpib_init_params_t p;
        memset(&p, 0, sizeof(p));
        p.system_controller = true;
        p.own_address.primary = 5;
        p.port = &m.base;
        CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    }
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_emergency_stop(&ctx) == GPIB_OK);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.lifecycle == GPIB_LC_STOPPING);
    CHECK(snap.instance_epoch == t.epoch);
    CHECK(snap.active_action_id != 0);
    CHECK(snap.unacked_blocks <= 2);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

void suite_emergency(void) {
    emg_01_basic();
    emg_02_mid_tx();
    emg_03_snapshot_in_stopping();
}
