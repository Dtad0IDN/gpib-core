#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_port.h"
#include "test_util.h"

static uint8_t s_tx[100];
static const uint8_t s_rx_src[32] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};
static uint8_t s_rx_buf[32];

static void start_mock(gpib_context_t *ctx, mock_port_t *m) {
    gpib_init_params_t p;
    mock_init(m);
    memset(&p, 0, sizeof(p));
    p.system_controller = true;
    p.own_address.primary = 5;
    p.port = &m->base;
    CHECK(gpib_init(ctx, &p) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
    memset(s_tx, 0xA5, sizeof(s_tx));
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

static gpib_request_t make_read(size_t cap, size_t min_bytes) {
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = cap;
    r.rx_min_bytes = min_bytes;
    return r;
}

/* Happy path: a bounded TX stream completes and both terminal records are
 * published; the instance drains and deinits cleanly. */
static void mbx_tx_happy(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(8);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 8);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(m.tx_accepted == 8);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* MBX-01: a stop that wins before acceptance leaves the action never-ACTIVE
 * with exactly one completion and zero bytes on the wire. */
static void mbx_01_publish_vs_stop(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(8);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_port_event_t ev;
    start_mock(&ctx, &m);
    ev.type = (uint8_t)GPIB_PEVT_DEADLINE;
    ev.tag = 1; /* first action generation */
    ev.data = 0;
    ev.line = 0;
    mock_push_event(&m, ev);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(res.bytes_confirmed == 0);
    CHECK(m.tx_accepted == 0);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
}

/* MBX-02: cancel of a dispatched action yields one partial terminal record
 * and one ownership return; a repeated cancel reports ALREADY_TERMINAL. */
static void mbx_02_cancel_vs_dispatch(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* partial: 8 steps x 3 bytes */
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_CANCELED);
    CHECK(res.bytes_confirmed > 0);
    CHECK(res.bytes_confirmed < 100);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_CANCELED);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_ALREADY_TERMINAL);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK); /* single ownership return */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_INVALID_HANDLE);
}

/* MBX-04: done beats a late deadline: a completion already accounted is not
 * re-terminated; the stale deadline only counts as a dropped event. */
static void mbx_04_done_vs_timeout(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(8);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_fault_snapshot_t snap;
    gpib_port_event_t ev;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* completed */
    ev.type = (uint8_t)GPIB_PEVT_DEADLINE;
    ev.tag = 1;
    ev.data = 0;
    ev.line = 0;
    mock_push_event(&m, ev);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.unacked_results == 2); /* unchanged: single terminal result */
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.dropped_stale_events == 1);
}

/* MBX-08 (lite): an SRQ sideband during a long block is latched and serviced
 * without terminating the data block. */
static void mbx_08_srq_sideband(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_fault_snapshot_t snap;
    gpib_port_event_t ev;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    ev.type = (uint8_t)GPIB_PEVT_SRQ;
    ev.tag = 1;
    mock_push_event(&m, ev);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.unacked_results == 0); /* block still running */
    CHECK(m.tx_accepted > 0);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.srq_events == 1);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
}

/* MBX-07: a stale tagged event (old action generation) cannot touch the live
 * action; it is counted and dropped. */
static void mbx_07_stale_generation(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_fault_snapshot_t snap;
    gpib_port_event_t ev;
    start_mock(&ctx, &m);
    ev.type = (uint8_t)GPIB_PEVT_ATN_COMMAND; /* would P1-yield if applied */
    ev.tag = 99; /* stale */
    ev.data = 0x3f;
    mock_push_event(&m, ev);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(m.tx_accepted > 0); /* action unaffected, stream continued */
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.dropped_stale_events == 1);
    CHECK(!snap.command_valid);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
}

/* MBX-09 / LED-05: ATN during a data block triggers P1: one partial terminal
 * result for the WHOLE block, the command byte is captured for the owner and
 * never counted as payload, the remainder is not re-dispatched. */
static void mbx_09_atn_p1_yield(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_read(32, 32);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_fault_snapshot_t snap;
    gpib_port_event_t ev;
    start_mock(&ctx, &m);
    m.rx_src = s_rx_src;
    m.rx_len = sizeof(s_rx_src);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* partial RX: 21 of 32 (7 steps) */
    ev.type = (uint8_t)GPIB_PEVT_ATN_COMMAND;
    ev.tag = 1;
    ev.data = 0x3f; /* unlisten, example command byte */
    ev.line = 0;
    mock_push_event(&m, ev);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_PHASE_INTERRUPTED);
    CHECK(res.bytes_confirmed == 21); /* command byte not in the payload */
    CHECK(memcmp(s_rx_buf, s_rx_src, 21) == 0);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_PHASE_INTERRUPTED);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.command_valid);
    CHECK(snap.command_byte == 0x3f);
}

/* MBX-10: physical shutdown does not wait for user ACKs: results are
 * published, the ledger survives, DRAINING is reachable before ACKs. */
static void mbx_10_shutdown_vs_ack(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_DRAINING); /* no ACKs given yet */
    CHECK(info.unacked_results == 2);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_SHUTDOWN);
    CHECK(res.bytes_confirmed > 0);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* MBX-11 / QSC-01: a port that cannot prove executor quiescence blocks the
 * DRAINING transition and deinit until the fence is available. */
static void mbx_11_stuck_port(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    m.stuck_quiesce = true;
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_STOPPING);
    CHECK(gpib_deinit(&ctx) == GPIB_BUSY_RESOURCES);
    m.stuck_quiesce = false;
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_DRAINING);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* MBX-12 / LED-04 / QSC-02: lost mandatory edge history fails closed: fault,
 * quarantine of the pinned block, STOPPING forever, deinit refuses, snapshot
 * stays available. */
static void mbx_12_history_lost(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_fault_snapshot_t snap;
    gpib_status_info_t info;
    gpib_port_event_t ev;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* ACTIVE, partial */
    ev.type = (uint8_t)GPIB_PEVT_HISTORY_LOST;
    ev.tag = 0;
    mock_push_event(&m, ev);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_STOPPING);
    CHECK(info.fault_latched);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.fault_code == GPIB_FAULT_HISTORY_LOST);
    CHECK(snap.quarantined_blocks == 1);
    CHECK(gpib_result_ack(&ctx, b) != GPIB_OK); /* no safe ACK exists */
    CHECK(gpib_deinit(&ctx) == GPIB_BUSY_RESOURCES);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_STOPPING); /* forever, by design */
    CHECK(gpib_deinit(&ctx) == GPIB_BUSY_RESOURCES);
}

/* MBX-14: a late completion after a P1 yield cannot re-terminate the block:
 * stale generation is dropped, the single terminal record stays stable. */
static void mbx_14_yield_late_completion(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_read(32, 32);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_port_event_t ev;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    m.rx_src = s_rx_src;
    m.rx_len = sizeof(s_rx_src);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    ev.type = (uint8_t)GPIB_PEVT_ATN_COMMAND;
    ev.tag = 1;
    ev.data = 0x5f;
    mock_push_event(&m, ev);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* P1 yield happened */
    ev.type = (uint8_t)GPIB_PEVT_DEADLINE; /* late completion attempt */
    ev.tag = 1; /* stale: the slot generation moved on */
    mock_push_event(&m, ev);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.unacked_results == 2); /* exactly one terminal per record */
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_PHASE_INTERRUPTED);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_ALREADY_TERMINAL);
}

/* WM-01 / R5: a WRITE_MANY run is ONE action over the payload with one
 * aggregated terminal result; the common handshake moves each byte once,
 * not once per listener. */
static void wm_01_single_action(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_address_t listeners[3] = { { 5, 0, false }, { 6, 0, false },
                                    { 7, 0, false } };
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 8;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE_MANY;
    r.listeners = listeners;
    r.listener_count = 3;
    r.tx_data = s_tx;
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 8);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(m.tx_accepted == 8); /* bytes moved once: aggregated handshake */
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.unacked_results == 2);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DEV-01/02: raw device command bytes are latched FIFO for the application
 * without terminating the running block; ring overflow drops the oldest and
 * counts it; the channel drains oldest-first. */
static void dev_events_channel(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r = make_write(100);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_fault_snapshot_t snap;
    gpib_port_event_t ev;
    uint8_t cmd = 0;
    start_mock(&ctx, &m);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* partial, action ACTIVE */
    memset(&ev, 0, sizeof(ev));
    ev.type = (uint8_t)GPIB_PEVT_DEVICE_CMD;
    ev.data = 0x14; /* DCL candidate */
    mock_push_event(&m, ev);
    ev.data = 0x08; /* GET candidate */
    mock_push_event(&m, ev);
    ev.data = 0x11; /* LLO candidate */
    mock_push_event(&m, ev);
    ev.data = 0x13;
    mock_push_event(&m, ev);
    ev.data = 0x01; /* overflows the 4-slot ring: drops 0x14 */
    mock_push_event(&m, ev);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_NO_RESULT); /* still running */
    CHECK(gpib_poll_device_event(&ctx, &cmd) == GPIB_OK);
    CHECK(cmd == 0x08);
    CHECK(gpib_poll_device_event(&ctx, &cmd) == GPIB_OK);
    CHECK(cmd == 0x11);
    CHECK(gpib_poll_device_event(&ctx, &cmd) == GPIB_OK);
    CHECK(cmd == 0x13);
    CHECK(gpib_poll_device_event(&ctx, &cmd) == GPIB_OK);
    CHECK(cmd == 0x01);
    CHECK(gpib_poll_device_event(&ctx, &cmd) == GPIB_NO_RESULT);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.device_events_dropped == 1);
    /* The data block was never affected by the service channel. */
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_CANCELED);
}

void suite_mbx(void) {
    mbx_tx_happy();
    mbx_01_publish_vs_stop();
    mbx_02_cancel_vs_dispatch();
    mbx_04_done_vs_timeout();
    mbx_08_srq_sideband();
    mbx_07_stale_generation();
    mbx_09_atn_p1_yield();
    mbx_10_shutdown_vs_ack();
    mbx_11_stuck_port();
    mbx_12_history_lost();
    mbx_14_yield_late_completion();
    wm_01_single_action();
    dev_events_channel();
}
