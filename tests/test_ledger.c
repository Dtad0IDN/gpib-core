#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
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

/* LED-02: a queued, never-dispatched block is terminalized by shutdown
 * WITHOUT a fake ACTION_QUIESCED; reason NOT_STARTED, one record per block,
 * transaction result published after its blocks. */
static void led_02(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_request_t r = make_write(6);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b1) == GPIB_OK);
    CHECK(gpib_submit_block(&ctx, t, s_payload, sizeof(s_payload), &b2) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b1, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED);
    CHECK(res.status == GPIB_OK);
    CHECK(res.bytes_confirmed == 0);
    CHECK(gpib_result_peek(&ctx, b2, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_SHUTDOWN);
}

/* LED-06: stale/double/wrong-kind ACKs are refused without touching the
 * affected or newly occupied slots; the genuine block ACK still works. */
static void led_06(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_request_t r = make_write(6);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_handle_t wrong_kind;
    gpib_result_t res;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);

    wrong_kind = b;
    wrong_kind.kind = (uint8_t)GPIB_HANDLE_TXN;
    /* Required property: a wrong-kind ACK is refused without side effects;
     * the exact refusal code stays [O] because slot/generation numbers of
     * different axes may coincide numerically. */
    {
        gpib_status_t st = gpib_result_ack(&ctx, wrong_kind);
        CHECK(st == GPIB_INVALID_HANDLE || st == GPIB_NO_RESULT);
        CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_NO_RESULT);
    }
    CHECK(gpib_result_ack(&ctx, t) == GPIB_NO_RESULT); /* not terminal yet */

    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_INVALID_HANDLE); /* double ACK */
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_INVALID_HANDLE);
}

void suite_ledger(void) {
    led_02();
    led_06();
}
