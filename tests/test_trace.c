#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_port.h"
#include "test_util.h"

static uint8_t s_tx[64];

static void start_mock(gpib_context_t *ctx, mock_port_t *m) {
    gpib_init_params_t p;
    mock_init(m);
    memset(&p, 0, sizeof(p));
    p.system_controller = true;
    p.own_address.primary = 5;
    p.port = &m->base;
    CHECK(gpib_init(ctx, &p) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
    memset(s_tx, 0x77, sizeof(s_tx));
}

static gpib_request_t make_write(size_t len) {
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = len;
    return r;
}

/* TRC-01: lifecycle and operation events land in the ring in order; dump is
 * non-consuming. */
static void trc_01_ordered_events(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(8);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_trace_event_t ev[16];
    gpib_trace_event_t ev2[16];
    size_t n = 0;
    gpib_trace_info_t info;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_trace_query(&ctx, &info) == GPIB_OK);
    CHECK(info.count >= 8);
    CHECK(info.dropped == 0);
    CHECK(gpib_trace_dump(&ctx, ev, 16, &n) == GPIB_OK);
    CHECK(n == info.count);
    CHECK(n >= 8);
    CHECK(ev[0].kind == (uint16_t)GPIB_TRC_INIT);
    CHECK(ev[1].kind == (uint16_t)GPIB_TRC_START);
    CHECK(ev[2].kind == (uint16_t)GPIB_TRC_SUBMIT);
    CHECK(ev[n - 3].kind == (uint16_t)GPIB_TRC_SHUTDOWN ||
          ev[n - 2].kind == (uint16_t)GPIB_TRC_SHUTDOWN);
    /* Non-consuming: a second dump yields identical records. */
    CHECK(gpib_trace_dump(&ctx, ev2, 16, &n) == GPIB_OK);
    CHECK(memcmp(ev, ev2, n * sizeof(ev[0])) == 0);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* TRC-02: ring overflow drops the OLDEST records, counts them and keeps the
 * newest ones. WRITE transactions auto-close at their last block ([P]
 * OPEN-23), so the driver is a sequence of short transactions. */
static void trc_02_wrap_policy(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_trace_event_t ev[GPIB_CFG_TRACE_EVENTS];
    gpib_trace_info_t info;
    size_t n = 0;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 8;
    for (int i = 0; i < 9; i++) { /* ~7 events per cycle > ring capacity */
        gpib_request_t r = make_write(8);
        gpib_handle_t t = { 0, 0, 0, 0, 0 };
        gpib_handle_t b = { 0, 0, 0, 0, 0 };
        CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
        CHECK(gpib_process(&ctx) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    }
    CHECK(gpib_trace_query(&ctx, &info) == GPIB_OK);
    CHECK(info.count == GPIB_CFG_TRACE_EVENTS); /* ring is full */
    CHECK(info.dropped > 0);
    CHECK(gpib_trace_dump(&ctx, ev, GPIB_CFG_TRACE_EVENTS, &n) == GPIB_OK);
    CHECK(n == GPIB_CFG_TRACE_EVENTS);
    /* The very first events are gone; the newest record is the last txn ACK. */
    CHECK(ev[0].kind != (uint16_t)GPIB_TRC_INIT);
    CHECK(ev[n - 1].kind == (uint16_t)GPIB_TRC_ACK_TXN);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* TRC-03: faults, quarantine and deadline events are recorded alongside the
 * zombie diagnostics. */
static void trc_03_fault_events(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(64);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_port_event_t pe;
    gpib_trace_event_t ev[GPIB_CFG_TRACE_EVENTS];
    gpib_fault_snapshot_t snap;
    size_t n = 0;
    bool saw_emergency = false;
    bool saw_fault = false;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* partial */
    pe.type = (uint8_t)GPIB_PEVT_HISTORY_LOST;
    pe.tag = 0;
    pe.data = 0;
    pe.line = 0;
    mock_push_event(&m, pe);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* fault + quarantine */
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.fault_code == GPIB_FAULT_HISTORY_LOST);
    CHECK(gpib_trace_dump(&ctx, ev, GPIB_CFG_TRACE_EVENTS, &n) == GPIB_OK);
    for (size_t i = 0; i < n; i++) {
        if (ev[i].kind == (uint16_t)GPIB_TRC_QUARANTINE) {
            saw_emergency = true; /* quarantine recorded */
        }
        if (ev[i].kind == (uint16_t)GPIB_TRC_FAULT) {
            saw_fault = true;
        }
    }
    CHECK(saw_emergency);
    CHECK(saw_fault);
    CHECK(gpib_deinit(&ctx) == GPIB_BUSY_RESOURCES); /* pinned forever */
}

void suite_trace(void) {
    trc_01_ordered_events();
    trc_02_wrap_policy();
    trc_03_fault_events();
}
