#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_port.h"
#include "test_util.h"

static const uint8_t s_payload[4] = { 1, 2, 3, 4 };

static void init_params_fill(gpib_init_params_t *p, uint8_t primary) {
    memset(p, 0, sizeof(*p));
    p->system_controller = true;
    p->own_address.primary = primary;
}

static gpib_request_t make_write(uint8_t primary) {
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = primary;
    r.tx_data = s_payload;
    r.tx_len = sizeof(s_payload);
    return r;
}

/* LIF-01: init performs no protocol work and leaves INITIALIZED; only an
 * explicit start enters RUNNING; repeated start is denied. */
static void lif_01(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_status_info_t info;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_INITIALIZED);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_RUNNING);
    CHECK(gpib_start(&ctx) == GPIB_INVALID_STATE);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_RUNNING);
}

/* LIF-02: a failed bounded port arm leaves the instance INITIALIZED with no
 * persistent changes; a retry after the port condition clears succeeds. */
static void lif_02_arm_rollback(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_init_params_t p;
    gpib_status_info_t info;
    mock_init(&m);
    memset(&p, 0, sizeof(p));
    p.system_controller = true;
    p.own_address.primary = 5;
    p.port = &m.base;
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    m.arm_fail = true;
    CHECK(gpib_start(&ctx) == GPIB_PORT_ERROR);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_INITIALIZED);
    CHECK(info.accepted_blocks == 0);
    m.arm_fail = false;
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_RUNNING);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* LIF-03: INITIALIZED allows status/diagnostics/process and rejects protocol
 * work by default-deny; peek/ack over an empty ledger has no side effects. */
static void lif_03(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_status_info_t info;
    gpib_request_t r = make_write(6);
    gpib_handle_t t = { 0, 0, 0, (uint8_t)GPIB_HANDLE_TXN, 0 };
    gpib_handle_t b = { 0, 0, 0, (uint8_t)GPIB_HANDLE_BLOCK, 0 };
    gpib_result_t res;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_INVALID_STATE);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_INVALID_STATE);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_NO_RESULT);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_NO_RESULT);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_INITIALIZED);
    CHECK(info.accepted_blocks == 0);
}

/* LIF-04: deinit from INITIALIZED succeeds after a provably safe port
 * detach; the wiped instance reports INVALID_HANDLE. */
static void lif_04(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_status_info_t info;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_INVALID_HANDLE);
}

/* LIF-05: a regular cancel stays local (instance remains RUNNING); repeated
 * init and deinit from RUNNING are denied. */
static void lif_05(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_status_info_t info;
    gpib_request_t r = make_write(6);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_RUNNING);
    CHECK(gpib_init(&ctx, &p) == GPIB_INVALID_STATE);
    CHECK(gpib_deinit(&ctx) == GPIB_INVALID_STATE);
}

/* LIF-06/07/08: shutdown closes intake; STOPPING services only stop-work and
 * refuses deinit; process publishes DRAINING after full terminal accounting;
 * DRAINING allows peek/ACK but no new work; deinit succeeds once drained. */
static void lif_06_07_08(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_status_info_t info;
    gpib_request_t r = make_write(6);
    gpib_request_t r2 = make_write(7);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    gpib_handle_t t2 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b3 = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b1) == GPIB_OK);
    CHECK(gpib_submit_block(&ctx, t, s_payload, sizeof(s_payload), &b2) == GPIB_OK);

    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_STOPPING);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_ALREADY_STOPPING);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_SHUTDOWN_IN_PROGRESS);
    CHECK(gpib_submit(&ctx, &r2, &t2, &b3) == GPIB_INVALID_STATE);
    CHECK(gpib_deinit(&ctx) == GPIB_BUSY_RESOURCES);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.last_busy_reason == GPIB_BUSY_STOP_TRANSITION_PENDING);

    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_DRAINING);
    CHECK(info.unacked_results == 3); /* two blocks + one transaction */

    CHECK(gpib_result_peek(&ctx, b1, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_SHUTDOWN);
    CHECK(gpib_start(&ctx) == GPIB_INVALID_STATE);
    CHECK(gpib_set_address(&ctx, p.own_address) == GPIB_INVALID_STATE);
    CHECK(gpib_deinit(&ctx) == GPIB_BUSY_RESOURCES);

    CHECK(gpib_result_ack(&ctx, b1) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_INVALID_HANDLE);
}

/* LIF shutdown from INITIALIZED is permitted without a prior start. */
static void lif_shutdown_initialized(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

void suite_lifecycle(void) {
    lif_01();
    lif_02_arm_rollback();
    lif_03();
    lif_04();
    lif_05();
    lif_06_07_08();
    lif_shutdown_initialized();
}
