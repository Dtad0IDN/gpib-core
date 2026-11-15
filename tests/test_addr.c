#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "test_util.h"

/* Semantic address API tests (REQ-078/079). TE/LE is now IMPLEMENTED:
 * secondary addresses 0..31 are accepted; the SAD encoding space is
 * 0x60|n, n=0..31. Values > 31 are rejected. Absence is still expressed
 * ONLY via secondary_present == false. */

static const uint8_t s_payload[4] = { 1, 2, 3, 4 };

static void init_params_fill(gpib_init_params_t *p, uint8_t primary) {
    memset(p, 0, sizeof(*p));
    p->system_controller = true;
    p->own_address.primary = primary;
}

static gpib_request_t make_write(const gpib_address_t *a) {
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address = *a;
    r.tx_data = s_payload;
    r.tx_len = sizeof(s_payload);
    return r;
}

/* ADDR-01: both address forms are representable; primary-only AND
 * secondary (0..31) are both ACCEPTED now (TE/LE implemented). */
static void addr_01(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    gpib_address_t prim_only = { 5, 0, false };
    gpib_request_t r = make_write(&prim_only);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    gpib_address_t with_sec = { 6, 0x1f, true }; /* SAD(31) valid */
    r = make_write(&with_sec);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
}

/* ADDR-02: with secondary_present == false the numeric secondary value is
 * irrelevant to addressing. */
static void addr_02(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    gpib_address_t a_zero = { 5, 0, false };
    gpib_address_t a_seven = { 5, 7, false };
    gpib_request_t r = make_write(&a_zero);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    r = make_write(&a_seven);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
}

/* ADDR-03: secondary_present with secondary == 0 is a PRESENT secondary
 * (SAD(0) = 0x60), not "absent"; it is now ACCEPTED (valid SAD range). */
static void addr_03(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    gpib_address_t sec_zero = { 5, 0, true };
    gpib_request_t r = make_write(&sec_zero);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
}

/* ADDR-04: secondary > 31 is out of the SAD encoding space and is
 * still rejected BEFORE any side effect. */
static void addr_04(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_status_info_t before, after;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_query_status(&ctx, &before) == GPIB_OK);
    gpib_address_t bad = { 5, 32, true };
    gpib_request_t r = make_write(&bad);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_UNSUPPORTED_ADDRESSING);
    CHECK(gpib_query_status(&ctx, &after) == GPIB_OK);
    CHECK(after.accepted_blocks == before.accepted_blocks);
    CHECK(after.open_transactions == before.open_transactions);
    CHECK(after.lifecycle == GPIB_LC_RUNNING);
}

/* ADDR-05: write_many with secondary in 0..31 is accepted; with
 * secondary > 31 the whole list is rejected before any addressing. */
static void addr_05(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_status_info_t before, after;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    /* Valid: secondary listener accepted */
    gpib_address_t good[2] = { { 5, 0, false }, { 6, 3, true } };
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE_MANY;
    r.listeners = good;
    r.listener_count = 2;
    r.tx_data = s_payload;
    r.tx_len = sizeof(s_payload);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* Invalid: secondary > 32 */
    CHECK(gpib_query_status(&ctx, &before) == GPIB_OK);
    gpib_address_t bad[2] = { { 5, 0, false }, { 6, 32, true } };
    r.listeners = bad;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_UNSUPPORTED_ADDRESSING);
    CHECK(gpib_query_status(&ctx, &after) == GPIB_OK);
    CHECK(after.accepted_blocks == before.accepted_blocks);
    CHECK(after.open_transactions == before.open_transactions);
}

/* ADDR-06: own-address configuration accepts secondary 0..31, rejects
 * > 31 without changing the active config. */
static void addr_06(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);

    /* Valid secondary accepted */
    CHECK(gpib_set_address(&ctx, (gpib_address_t){ 9, 3, true }) == GPIB_OK);
    /* Invalid secondary rejected, active config unchanged */
    CHECK(gpib_set_address(&ctx, (gpib_address_t){ 9, 32, true }) ==
          GPIB_UNSUPPORTED_ADDRESSING);
    CHECK(gpib_set_address(&ctx, (gpib_address_t){ 10, 0, false }) == GPIB_OK);
    /* init with secondary accepted */
    {
        GPIB_CONTEXT_STORAGE_DEFINE(ctx2);
        gpib_init_params_t p2;
        init_params_fill(&p2, 5);
        p2.own_address.secondary = 2;
        p2.own_address.secondary_present = true;
        CHECK(gpib_init(&ctx2, &p2) == GPIB_OK);
        CHECK(gpib_start(&ctx2) == GPIB_OK);
        CHECK(gpib_request_shutdown(&ctx2) == GPIB_OK);
        CHECK(gpib_process(&ctx2) == GPIB_OK);
        CHECK(gpib_deinit(&ctx2) == GPIB_OK);
    }
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* ADDR-07: primary-address range gates and raw-submit payload contracts
 * (audit Этап A: F07/F08/F12). The low level owns the validation -- a
 * wrapper must not be the only defense line. */
static void addr_07(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);

    /* F12: primary 0..30 valid; 31 is the reserved UNL/UNT encoding,
     * larger values alias through the 5-bit field. */
    CHECK(gpib_address_supported(&(gpib_address_t){ 30, 0, false }));
    CHECK(!gpib_address_supported(&(gpib_address_t){ 31, 0, false }));
    CHECK(!gpib_address_supported(&(gpib_address_t){ 32, 0, false }));
    CHECK(!gpib_address_supported(&(gpib_address_t){ 255, 0, false }));
    gpib_request_t r = make_write(&(gpib_address_t){ 31, 0, false });
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_UNSUPPORTED_ADDRESSING);

    /* F07: raw SEND COMMAND validates its list at the low level. */
    {
        gpib_request_t c;
        uint8_t buf[4] = { 0x3Fu, 0x05u, 0x68u, 0x15u };
        memset(&c, 0, sizeof(c));
        c.kind = GPIB_REQ_CMDS;
        c.cmd_list = NULL;
        c.cmd_len = 4;
        CHECK(gpib_submit(&ctx, &c, &t, &b) == GPIB_INVALID_ARGUMENT);
        c.cmd_list = buf;
        c.cmd_len = 0;
        CHECK(gpib_submit(&ctx, &c, &t, &b) == GPIB_INVALID_ARGUMENT);
        c.cmd_len = (size_t)GPIB_CFG_MAX_CMDS + 1u;
        CHECK(gpib_submit(&ctx, &c, &t, &b) == GPIB_INVALID_ARGUMENT);
        c.cmd_len = 4;
        CHECK(gpib_submit(&ctx, &c, &t, &b) == GPIB_OK);
        CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    }

    /* F08/F12 + AUD-02: address-list operations validate the list, the
     * results sink AND its capacity at the low level. */
    {
        gpib_request_t q;
        uint8_t good[2] = { 6, 7 };
        uint8_t with31[2] = { 6, 31 };
        uint8_t stb[2] = { 0, 0 };
        memset(&q, 0, sizeof(q));
        q.kind = GPIB_REQ_ALLSPOLL;
        q.tx_data = with31;
        q.tx_len = 2;
        q.rx_data = stb;
        q.rx_cap = sizeof(stb);
        CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_INVALID_ARGUMENT);
        q.tx_data = NULL;
        CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_INVALID_ARGUMENT);
        q.tx_data = good;
        q.rx_data = NULL;
        q.rx_cap = 0;
        CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_INVALID_ARGUMENT);
        q.rx_data = stb;
        /* AUD-02: a too-small sink is rejected before any side effect. */
        q.rx_cap = 1;
        CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_INVALID_ARGUMENT);
        q.rx_cap = sizeof(stb);
        CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_OK);
        CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
        /* FINDRQS also requires the found-index sink (F06-adjacent). */
        {
            size_t found = 0;
            memset(&q, 0, sizeof(q));
            q.kind = GPIB_REQ_FINDRQS;
            q.tx_data = good;
            q.tx_len = 2;
            q.rx_data = stb;
            q.rx_cap = 1; /* AUD-02: capacity enforced here too */
            CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_INVALID_ARGUMENT);
            q.rx_cap = sizeof(stb);
            CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_INVALID_ARGUMENT);
            q.find_out = &found;
            CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_OK);
            CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
            CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
            CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
        }
        /* AUD-02 (FINDLSTN): one bool per address -- n*sizeof(bool). */
        {
            bool results[2] = { false, false };
            memset(&q, 0, sizeof(q));
            q.kind = GPIB_REQ_FINDLSTN;
            q.tx_data = good;
            q.tx_len = 2;
            q.rx_data = results;
            q.rx_cap = 1; /* too small for 2 bools */
            CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_INVALID_ARGUMENT);
            q.rx_cap = sizeof(results);
            CHECK(gpib_submit(&ctx, &q, &t, &b) == GPIB_OK);
            CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
            CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
            CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
        }
    }

    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

void suite_addr(void) {
    addr_01();
    addr_02();
    addr_03();
    addr_04();
    addr_05();
    addr_06();
    addr_07();
}
