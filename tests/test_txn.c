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

/* TXN-01: an accepted transaction with N blocks occupies N block slots plus
 * one transaction record; shortage rejects the new work without transferring
 * ownership (counters unchanged). */
static void txn_01(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_request_t r = make_write(6);
    gpib_handle_t t[4];
    gpib_handle_t b;
    gpib_handle_t extra = { 0, 0, 0, 0, 0 };
    gpib_handle_t t5 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b5 = { 0, 0, 0, 0, 0 };
    gpib_status_info_t before, after;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    memset(t, 0, sizeof(t));
    memset(&b, 0, sizeof(b));
    for (int i = 0; i < 4; i++) { /* GPIB_CFG_MAX_OPERATIONS == 4 */
        CHECK(gpib_submit(&ctx, &r, &t[i], &b) == GPIB_OK);
    }
    for (int i = 0; i < 4; i++) { /* blocks 4..7 of 8 */
        CHECK(gpib_submit_block(&ctx, t[0], s_payload, sizeof(s_payload),
            &extra) == GPIB_OK);
    }
    CHECK(gpib_query_status(&ctx, &before) == GPIB_OK);
    CHECK(before.open_transactions == 4);
    CHECK(before.accepted_blocks == 8);
    CHECK(gpib_submit_block(&ctx, t[0], s_payload, sizeof(s_payload),
        &extra) == GPIB_BUSY_RESOURCES);
    CHECK(gpib_submit(&ctx, &r, &t5, &b5) == GPIB_BUSY_RESOURCES);
    CHECK(gpib_query_status(&ctx, &after) == GPIB_OK);
    CHECK(after.open_transactions == before.open_transactions);
    CHECK(after.accepted_blocks == before.accepted_blocks);
}

/* TXN-02: the transaction terminal record is published only after all of its
 * accepted blocks are terminal; block ACKs are NOT required for that. */
static void txn_02(void) {
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
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    /* No block ACKed so far, yet both blocks and the txn are terminal. */
    CHECK(gpib_result_peek(&ctx, b1, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED);
    CHECK(gpib_result_peek(&ctx, b2, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_NOT_STARTED);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_CANCELED);
}

/* TXN-03: after txn ACK and slot reuse, an un-ACKed block handle stays valid
 * until its own ACK (FIX-01); the stale transaction handle is dead. */
static void txn_03(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_request_t r = make_write(6);
    gpib_handle_t t1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    gpib_handle_t t2 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b3 = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t1, &b1) == GPIB_OK);
    CHECK(gpib_submit_block(&ctx, t1, s_payload, sizeof(s_payload), &b2) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t1) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t1) == GPIB_OK);

    CHECK(gpib_submit(&ctx, &r, &t2, &b3) == GPIB_OK);
    CHECK(t2.slot == t1.slot);
    CHECK(t2.generation != t1.generation);
    CHECK(gpib_request_cancel(&ctx, t1) == GPIB_NOT_FOUND); /* stale */

    /* Late block ACKs of the reused transaction's blocks still work. */
    CHECK(gpib_result_peek(&ctx, b1, &res) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b1) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK);
}

/* TXN-04: cancel racing dispatch resolves to exactly one terminal result:
 * first cancel wins, the second reports ALREADY_TERMINAL. */
static void txn_04(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_request_t r = make_write(6);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_ALREADY_TERMINAL);
    CHECK(gpib_request_cancel(&ctx, b) == GPIB_NOT_FOUND); /* wrong kind */
}

/* TXN-05: after deinit and re-init in the same storage the epoch differs;
 * handles of the previous instance never come back to life. */
static void txn_05(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t p;
    gpib_request_t r = make_write(6);
    gpib_handle_t t_old = { 0, 0, 0, 0, 0 };
    gpib_handle_t b_old = { 0, 0, 0, 0, 0 };
    gpib_handle_t t_new = { 0, 0, 0, 0, 0 };
    gpib_handle_t b_new = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t_old, &b_old) == GPIB_OK);
    CHECK(gpib_request_cancel(&ctx, t_old) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b_old) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t_old) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);

    init_params_fill(&p, 5);
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b_old, &res) == GPIB_INVALID_HANDLE);
    CHECK(gpib_result_ack(&ctx, t_old) == GPIB_INVALID_HANDLE);
    CHECK(gpib_request_cancel(&ctx, t_old) == GPIB_NOT_FOUND);
    CHECK(gpib_submit(&ctx, &r, &t_new, &b_new) == GPIB_OK);
    CHECK(t_new.epoch != t_old.epoch);
}

/* TXN-06 (R17-01): block dispatch order is the SUBMIT order, not the
 * physical slot. The auditor's sequence: submit A, block B, block C;
 * ACK A while B executes (slot 0 freed); submit D (takes slot 0) --
 * the wire order must stay A B C D, the slot-based picker produced
 * A B D C. */
static void txn_06(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_init_params_t p;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t ba = { 0, 0, 0, 0, 0 };
    gpib_handle_t bb = { 0, 0, 0, 0, 0 };
    gpib_handle_t bc = { 0, 0, 0, 0, 0 };
    gpib_handle_t bd = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    static uint8_t blk[4][8];
    int guard = 0;
    init_params_fill(&p, 5);
    mock_init(&m);
    p.port = &m.base;
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    for (int i = 0; i < 4; i++) {
        memset(blk[i], (uint8_t)('A' + i), sizeof(blk[i]));
    }
    m.bytes_per_transfer = 1; /* one byte per round: slack to ACK and
                                * re-submit while B is still executing */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = blk[0];
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &ba) == GPIB_OK);
    CHECK(gpib_submit_block(&ctx, t, blk[1], 8, &bb) == GPIB_OK);
    CHECK(gpib_submit_block(&ctx, t, blk[2], 8, &bc) == GPIB_OK);
    /* Let A complete and B start, then free A's slot mid-flight. */
    while (gpib_result_peek(&ctx, ba, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000);
    CHECK(gpib_result_ack(&ctx, ba) == GPIB_OK); /* frees slot 0 */
    /* D reuses the freed LOW slot; its SUBMIT order is last. */
    CHECK(gpib_submit_block(&ctx, t, blk[3], 8, &bd) == GPIB_OK);
    CHECK(ctx.block[bd.slot].txn_slot == ba.slot); /* D indeed took the
                                                    * recycled low slot */
    guard = 0;
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 32u);
    /* The mock byte-pump journal must be A B C D in order. */
    CHECK(m.journal_len == 32u);
    for (size_t i = 0; i < m.journal_len; i++) {
        CHECK(m.journal[i] == (uint8_t)('A' + (i / 8u)));
    }
    CHECK(gpib_result_ack(&ctx, bd) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, bc) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, bb) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* TXN-07 (R17-02): at generation exhaustion the slot RETIRES -- a
 * wrapped generation can never re-validate a stale handle. */
static void txn_07(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_init_params_t p;
    gpib_request_t r = make_write(6);
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    init_params_fill(&p, 5);
    mock_init(&m);
    m.bytes_per_transfer = 4;
    p.port = &m.base;
    CHECK(gpib_init(&ctx, &p) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    /* White-box acceleration (the auditor's probe): place the block
     * generation AT the wrap boundary -- BOTH the record and the local
     * handle copy stay in sync (handles are values). The next ACK hits
     * the retirement branch. */
    ctx.block[b.slot].generation = UINT32_MAX;
    b.generation = UINT32_MAX;
    {
        int guard = 0;
        gpib_result_t res;
        while (gpib_result_peek(&ctx, b, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    /* The boundary ACK: generation would wrap -- the slot retires. */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(ctx.block[b.slot].retired);
    /* The single-block WRITE is already terminal: peek + ack the txn. */
    {
        gpib_result_t res;
        CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    }
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* A fresh submit NEVER lands in the retired slot. */
    {
        gpib_handle_t t2 = { 0, 0, 0, 0, 0 };
        gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
        CHECK(gpib_submit(&ctx, &r, &t2, &b2) == GPIB_OK);
        CHECK(b2.slot != b.slot);
        CHECK(gpib_request_cancel(&ctx, t2) == GPIB_OK);
        {
            gpib_result_t res;
            int guard = 0;
            while (gpib_result_peek(&ctx, t2, &res) != GPIB_OK &&
                   guard++ < 20000) {
                CHECK(gpib_process(&ctx) == GPIB_OK);
            }
            REQUIRE(guard < 20000);
        }
        CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, t2) == GPIB_OK);
    }
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    {
        int guard = 0;
        while (ctx.lifecycle != (uint8_t)GPIB_LC_DRAINING &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

void suite_txn(void) {
    txn_01();
    txn_02();
    txn_03();
    txn_04();
    txn_05();
    txn_06();
    txn_07();
}
