#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_port.h"
#include "test_util.h"

#define STREAM_TX_TOTAL (10u * 1024u * 1024u) /* REQ-031/§5.2: 10 MB */
#define STREAM_BLK_TX 65536u
#define STREAM_BLK_RX 16384u

static uint8_t s_blk_tx[STREAM_BLK_TX];
static uint8_t s_rx_a[STREAM_BLK_RX];
static uint8_t s_rx_b[STREAM_BLK_RX];
static const uint8_t s_src8[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
static uint8_t s_buf8[8];

static uint32_t xs(uint32_t *s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

static void start_mock(gpib_context_t *ctx, mock_port_t *m) {
    gpib_init_params_t p;
    mock_init(m);
    memset(&p, 0, sizeof(p));
    p.system_controller = true;
    p.own_address.primary = 5;
    p.port = &m->base;
    CHECK(gpib_init(ctx, &p) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
}

/* FIX-03 / REQ-065: a read-only view of the confirmed RX prefix exists
 * before ACK, expires after ACK, and TX blocks expose none. */
static void stream_view_semantics(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    const uint8_t *view = NULL;
    size_t view_len = 0;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 8;
    m.rx_src = s_src8;
    m.rx_len = sizeof(s_src8);
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_buf8;
    r.rx_cap = 8;
    r.rx_min_bytes = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_view(&ctx, b, &view, &view_len) == GPIB_OK);
    CHECK(view_len == 8);
    CHECK(memcmp(view, s_src8, 8) == 0);
    CHECK(gpib_result_view(&ctx, t, &view, &view_len) == GPIB_INVALID_HANDLE);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    /* After ACK the borrowed view is expired; the application's own pointer
     * stays valid (semantic check: the refusal, not a crash). */
    CHECK(gpib_result_view(&ctx, b, &view, &view_len) == GPIB_INVALID_HANDLE);
    CHECK(s_buf8[7] == 8);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
}

/* REQ-035: read completion by EOI is reported as an end cause separate from
 * the byte count. */
static void stream_eoi_end_cause(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_port_event_t ev;
    start_mock(&ctx, &m);
    m.rx_generate = true; /* default 3 bytes/step, budget 8 -> 21 of 32 */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_a;
    r.rx_cap = 32;
    r.rx_min_bytes = 0; /* EOI-terminated stream */
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* partial: 21 of 32 */
    ev.type = (uint8_t)GPIB_PEVT_EOI;
    ev.tag = 1;
    ev.data = 0;
    ev.line = 0;
    mock_push_event(&m, ev);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI);
    CHECK(res.bytes_confirmed == 21);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI);
    CHECK(res.bytes_confirmed == 21);
}

/* REQ-026: a drained RX block queue is a controlled pause, not completion;
 * the stream continues when the application supplies the next block. */
static void stream_backpressure(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_status_info_t info;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = STREAM_BLK_RX; /* one step fills a block */
    m.rx_generate = true;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_a;
    r.rx_cap = STREAM_BLK_RX;
    r.rx_min_bytes = 2 * STREAM_BLK_RX; /* two blocks needed */
    CHECK(gpib_submit(&ctx, &r, &t, &b1) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    /* First block filled; queue drained: backpressure pause. */
    CHECK(gpib_result_peek(&ctx, b1, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == STREAM_BLK_RX);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_NO_RESULT);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_RUNNING);
    CHECK(info.open_transactions == 1);
    CHECK(gpib_recover(&ctx) == GPIB_OK); /* idle instance: allowed */
    CHECK(gpib_submit_block(&ctx, t, s_rx_b, STREAM_BLK_RX, &b2) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_result_peek(&ctx, b2, &res) == GPIB_OK);
    CHECK(res.bytes_confirmed == STREAM_BLK_RX);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_LENGTH);
    CHECK(res.bytes_confirmed == 2 * STREAM_BLK_RX);
    CHECK(gpib_result_ack(&ctx, b1) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* REQ-031 / §5.2: a 10 MB TX stream in 64 KB blocks with at most 8 borrowed
 * blocks per transaction; WRITE auto-closes at the last accepted block
 * ([P], OPEN-23), so the stream spans 20 transactions; no loss, no
 * duplication (byte-sum oracle), bounded core RAM. */
static void stream_10mb_tx(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_result_t res;
    const size_t blocks_per_txn = GPIB_CFG_MAX_BLOCKS;
    const size_t total_blocks = STREAM_TX_TOTAL / STREAM_BLK_TX;
    const size_t total_txns = total_blocks / blocks_per_txn;
    uint32_t gs = 0x12345678u;
    uint64_t blk_sum = 0;
    uint64_t expect = 0;
    start_mock(&ctx, &m);
    m.bytes_per_transfer = STREAM_BLK_TX;
    for (size_t i = 0; i < STREAM_BLK_TX; i++) {
        s_blk_tx[i] = (uint8_t)(xs(&gs) & 0xFFu);
    }
    for (size_t i = 0; i < STREAM_BLK_TX; i++) {
        blk_sum += s_blk_tx[i];
    }
    expect = blk_sum * (uint64_t)total_blocks;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_blk_tx;
    r.tx_len = STREAM_BLK_TX;
    for (size_t ti = 0; ti < total_txns; ti++) {
        gpib_handle_t th = { 0, 0, 0, 0, 0 };
        gpib_handle_t bh[GPIB_CFG_MAX_BLOCKS];
        int guard = 0;
        CHECK(gpib_submit(&ctx, &r, &th, &bh[0]) == GPIB_OK);
        for (size_t k = 1; k < blocks_per_txn; k++) {
            CHECK(gpib_submit_block(&ctx, th, s_blk_tx, STREAM_BLK_TX,
                &bh[k]) == GPIB_OK);
        }
        while (gpib_result_peek(&ctx, th, &res) != GPIB_OK && guard++ < 100) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        CHECK(guard < 100);
        CHECK(res.reason == GPIB_TERM_COMPLETED);
        CHECK(res.bytes_confirmed == blocks_per_txn * STREAM_BLK_TX);
        for (size_t k = 0; k < blocks_per_txn; k++) {
            CHECK(gpib_result_peek(&ctx, bh[k], &res) == GPIB_OK);
            CHECK(res.bytes_confirmed == STREAM_BLK_TX);
            CHECK(gpib_result_ack(&ctx, bh[k]) == GPIB_OK);
        }
        CHECK(gpib_result_ack(&ctx, th) == GPIB_OK);
    }
    CHECK(m.tx_accepted == STREAM_TX_TOTAL);
    CHECK(m.tx_sum == expect); /* no loss, no duplication */
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* REQ-031: a 10 MB RX stream in ONE transaction (explicit length target)
 * through two ping-pong buffers: at most 2 blocks borrowed at a time; every
 * confirmed byte passes the view oracle before ACK. */
static void stream_10mb_rx(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t h[2];
    gpib_result_t res;
    const size_t total_blocks = STREAM_TX_TOTAL / STREAM_BLK_RX;
    uint32_t gs = 0x12345678u;
    uint64_t expect = 0;
    uint64_t got = 0;
    size_t submitted = 0;
    size_t acked = 0;
    int guard = 0;
    for (size_t i = 0; i < STREAM_TX_TOTAL; i++) {
        expect += (uint8_t)(xs(&gs) & 0xFFu);
    }
    start_mock(&ctx, &m);
    m.bytes_per_transfer = STREAM_BLK_RX;
    m.rx_generate = true;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_a;
    r.rx_cap = STREAM_BLK_RX;
    r.rx_min_bytes = STREAM_TX_TOTAL;
    CHECK(gpib_submit(&ctx, &r, &t, &h[0]) == GPIB_OK);
    submitted = 1;
    while (acked < total_blocks && guard++ < 20000) {
        while (submitted < total_blocks && (submitted - acked) < 2) {
            gpib_handle_t nb = { 0, 0, 0, 0, 0 };
            /* Ping-pong: even submissions borrow s_rx_a, odd s_rx_b; a
             * buffer may never be borrowed twice before its ACK. */
            const void *buf = (submitted % 2u) ? (const void *)s_rx_b
                                               : (const void *)s_rx_a;
            CHECK(gpib_submit_block(&ctx, t, buf, STREAM_BLK_RX, &nb) == GPIB_OK);
            h[submitted % 2u] = nb;
            submitted++;
        }
        CHECK(gpib_process(&ctx) == GPIB_OK);
        while (acked < submitted) {
            gpib_handle_t cur = h[acked % 2u];
            const uint8_t *view = NULL;
            size_t vlen = 0;
            if (gpib_result_peek(&ctx, cur, &res) != GPIB_OK) {
                break;
            }
            CHECK(gpib_result_view(&ctx, cur, &view, &vlen) == GPIB_OK);
            CHECK(vlen == STREAM_BLK_RX);
            for (size_t i = 0; i < vlen; i++) {
                got += view[i];
            }
            CHECK(gpib_result_ack(&ctx, cur) == GPIB_OK);
            acked++;
        }
    }
    CHECK(acked == total_blocks);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_LENGTH);
    CHECK(res.bytes_confirmed == STREAM_TX_TOTAL);
    CHECK(m.rx_generated == STREAM_TX_TOTAL);
    CHECK(got == expect); /* byte-exact stream through the view oracle */
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* RCV-01 analog: recover refuses where the recovery guard is unproven. */
static void stream_recover_guards(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_port_t m;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    start_mock(&ctx, &m);
    m.bytes_per_transfer = 64; /* completes the small write in one step */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_blk_tx;
    r.tx_len = 64;
    CHECK(gpib_recover(&ctx) == GPIB_OK); /* RUNNING, idle */
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK); /* completes the small write */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_recover(&ctx) == GPIB_RECOVERY_NOT_ALLOWED); /* STOPPING */
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);

    start_mock(&ctx, &m);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK); /* back to STOPPING */
    CHECK(gpib_recover(&ctx) == GPIB_RECOVERY_NOT_ALLOWED);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

void suite_stream(void) {
    stream_view_semantics();
    stream_eoi_end_cause();
    stream_backpressure();
    stream_10mb_tx();
    stream_10mb_rx();
    stream_recover_guards();
}
