#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_port.h"
#include "test_util.h"

static uint8_t s_tx[100];

static void start_mock(gpib_context_t *ctx, mock_port_t *m) {
    gpib_init_params_t p;
    mock_init(m);
    memset(&p, 0, sizeof(p));
    p.system_controller = true;
    p.own_address.primary = 5;
    p.port = &m->base;
    CHECK(gpib_init(ctx, &p) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
    memset(s_tx, 0x42, sizeof(s_tx));
}

static gpib_request_t make_write(size_t len, uint64_t deadline_us) {
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = len;
    r.deadline_us = deadline_us; /* absolute logical us ([P]) */
    return r;
}

/* DL-01: an expired deadline terminalizes the action and the operation with
 * zero confirmed bytes and no fabricated success. */
static void dl_01_expired(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(8, 100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 0; /* no bytes move at all */
    mock_advance_us(&m, 200); /* deadline at t=100 already passed */
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(res.bytes_confirmed == 0);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
}

/* DL-02: a late owner does not resurrect success: even a huge service delay
 * resolves to timeout, never to a completed result (MBX-04/PRG-05). */
static void dl_02_late_owner(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(8, 100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 0;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    mock_advance_us(&m, 1000000); /* owner stalled far past the deadline */
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(res.bytes_confirmed == 0);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.unacked_results == 2); /* exactly one terminal per record */
}

/* DL-03: completion before the deadline wins; a later expiry check may not
 * re-terminate the already-terminal records. */
static void dl_03_done_beats_deadline(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(8, 1000);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* completes at t=0 < 1000 */
    mock_advance_us(&m, 5000);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 8);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.unacked_results == 2);
}

/* DL-04: a deadline hit mid-transfer keeps the confirmed prefix; the result
 * is a partial timeout, not an error. */
static void dl_04_partial(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100, 100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 3;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* partial: 21 of 100 (7 steps) */
    mock_advance_us(&m, 2000); /* past the deadline */
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(res.bytes_confirmed == 21);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(res.bytes_confirmed == 21);
}

void suite_deadline(void) {
    dl_01_expired();
    dl_02_late_owner();
    dl_03_done_beats_deadline();
    dl_04_partial();
}
