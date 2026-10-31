#include <stdio.h>
#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_port.h"
#include "test_util.h"

/* Systematic interleaving enumeration for the mailbox host model (G-MBX,
 * MAILBOX_IRQ_CONTRACT §8): every short sequence of injected port facts is
 * applied at every action boundary (pre-dispatch / live / retired) for a TX
 * and an RX stream, and the same invariant checker runs after each sequence.
 * A seeded fuzz pass covers longer storms (deterministic seed, PRG-06/MBX
 * "failed-liveness ... с детерминированным seed"). */

enum { K_TX = 0, K_RX = 1 };
enum { PH_PRE = 0, PH_MID, PH_POST };

enum {
    E_ATN_OK = 0,   /* ATN command tagged with the live generation */
    E_ATN_STALE,    /* ATN command with a stale generation tag */
    E_DEAD_OK,      /* deadline expiry for the live generation */
    E_DEAD_STALE,   /* deadline expiry with a stale tag */
    E_SRQ,          /* sideband request */
    E_EOI,          /* end marker (RX-relevant only) */
    E_IFC,          /* untagged external IFC fact */
    E_HIST,         /* unrecoverable edge-history loss */
    E_COUNT
};

static const char *ev_name(int id) {
    static const char *names[E_COUNT] = {
        "ATN_OK", "ATN_STALE", "DEAD_OK", "DEAD_STALE",
        "SRQ", "EOI", "IFC", "HIST"
    };
    return names[id];
}

#define TX_LEN 100u
#define RX_LEN 32u

GPIB_CONTEXT_STORAGE_DEFINE(g_ctx);
static mock_port_t g_m;
static char g_run_desc[96];

static void hard_reset(void) {
    /* Test-side power cycle; the epoch manager keeps counting, so handles
     * of previous runs stay dead (TXN-05). */
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.storage_fingerprint = GPIB_STORAGE_FINGERPRINT;
}

static void begin_instance(void) {
    gpib_init_params_t p;
    mock_init(&g_m);
    memset(&p, 0, sizeof(p));
    p.system_controller = true;
    p.own_address.primary = 5;
    p.port = &g_m.base;
    if (gpib_init(&g_ctx, &p) != GPIB_OK || gpib_start(&g_ctx) != GPIB_OK) {
        CHECK(0 && "begin_instance failed");
    }
}

static void submit_stream(int kind, gpib_handle_t *t, gpib_handle_t *b) {
    static uint8_t tx_buf[TX_LEN];
    static uint8_t rx_buf[RX_LEN];
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    memset(tx_buf, 0x5C, sizeof(tx_buf));
    g_m.bytes_per_transfer = 3;
    if (kind == K_TX) {
        r.kind = GPIB_REQ_WRITE;
        r.address.primary = 6;
        r.tx_data = tx_buf;
        r.tx_len = TX_LEN;
    } else {
        r.kind = GPIB_REQ_READ;
        r.address.primary = 6;
        r.rx_data = rx_buf;
        r.rx_cap = RX_LEN;
        r.rx_min_bytes = RX_LEN;
        g_m.rx_generate = true;
    }
    CHECK(gpib_submit(&g_ctx, &r, t, b) == GPIB_OK);
}

static gpib_port_event_t make_event(int id) {
    gpib_port_event_t ev;
    memset(&ev, 0, sizeof(ev));
    switch (id) {
    case E_ATN_OK:
        ev.type = (uint8_t)GPIB_PEVT_ATN_COMMAND;
        ev.tag = 1; /* first action generation of the run */
        ev.data = 0x3f;
        break;
    case E_ATN_STALE:
        ev.type = (uint8_t)GPIB_PEVT_ATN_COMMAND;
        ev.tag = 99;
        ev.data = 0x3f;
        break;
    case E_DEAD_OK:
        ev.type = (uint8_t)GPIB_PEVT_DEADLINE;
        ev.tag = 1;
        break;
    case E_DEAD_STALE:
        ev.type = (uint8_t)GPIB_PEVT_DEADLINE;
        ev.tag = 99;
        break;
    case E_SRQ:
        ev.type = (uint8_t)GPIB_PEVT_SRQ;
        ev.tag = 1;
        break;
    case E_EOI:
        ev.type = (uint8_t)GPIB_PEVT_EOI;
        ev.tag = 1;
        break;
    case E_IFC:
        ev.type = (uint8_t)GPIB_PEVT_IFC;
        break;
    default:
        ev.type = (uint8_t)GPIB_PEVT_HISTORY_LOST;
        break;
    }
    return ev;
}

/* Single-evaluation check: prints the scenario descriptor on failure.
 * A naive `if (!(cond)) ... CHECK(cond)` would evaluate side-effecting
 * conditions twice and corrupt the run. */
#define ICHECK(cond)                                                     \
    do {                                                                 \
        int ok_ = (cond) ? 1 : 0;                                        \
        if (!ok_) {                                                      \
            fprintf(stderr, "RUN %s\n", g_run_desc);                     \
        }                                                                \
        CHECK(ok_);                                                      \
    } while (0)

static bool reason_is_expected(gpib_term_reason_t reason) {
    switch (reason) {
    case GPIB_TERM_COMPLETED:
    case GPIB_TERM_NOT_STARTED:
    case GPIB_TERM_CANCELED:
    case GPIB_TERM_SHUTDOWN:
    case GPIB_TERM_TIMEOUT:
    case GPIB_TERM_PHASE_INTERRUPTED:
    case GPIB_TERM_IFC:
        return true;
    default:
        return false;
    }
}

static void verify_block(int kind, gpib_handle_t b) {
    gpib_result_t rb;
    size_t len = (kind == K_TX) ? TX_LEN : RX_LEN;
    memset(&rb, 0, sizeof(rb));
    gpib_status_t st = gpib_result_peek(&g_ctx, b, &rb);
    if (st == GPIB_OK) {
        ICHECK(rb.bytes_confirmed <= len);
        ICHECK(reason_is_expected(rb.reason));
        /* No fabricated success: a LENGTH completion is always full. */
        if (rb.reason == GPIB_TERM_COMPLETED &&
            rb.end_cause == GPIB_END_LENGTH) {
            ICHECK(rb.bytes_confirmed == len);
        }
        /* EOI completions exist only on RX streams. */
        if (rb.end_cause == GPIB_END_EOI) {
            ICHECK(kind == K_RX);
            ICHECK(rb.bytes_confirmed < len);
        }
    } else {
        ICHECK(st == GPIB_NO_RESULT);
    }
}

static void verify_and_cleanup(int kind, int phase, const int *seq, int len,
    gpib_handle_t t, gpib_handle_t b) {
    gpib_status_info_t info;
    gpib_fault_snapshot_t snap;
    bool has_hist = false;
    for (int i = 0; i < len; i++) {
        has_hist = has_hist || seq[i] == E_HIST;
    }
    ICHECK(gpib_query_status(&g_ctx, &info) == GPIB_OK);
    ICHECK(info.lifecycle == GPIB_LC_RUNNING || info.lifecycle == GPIB_LC_STOPPING ||
           info.lifecycle == GPIB_LC_DRAINING);
    ICHECK(info.unacked_results <= 2);
    /* Drain the whole injected sequence: an idle executor consumes exactly
     * one queued fact per gpib_process call, so iterate generously. */
    for (int i = 0; i < len + 4; i++) {
        ICHECK(gpib_process(&g_ctx) == GPIB_OK);
    }
    ICHECK(gpib_get_fault_snapshot(&g_ctx, &snap) == GPIB_OK);
    if (has_hist) {
        ICHECK(snap.fault_latched);
        ICHECK(snap.fault_code == GPIB_FAULT_HISTORY_LOST);
    }
    verify_block(kind, b);
    (void)phase;

    ICHECK(gpib_query_status(&g_ctx, &info) == GPIB_OK);

    if (snap.quarantined_blocks != 0) {
        /* History lost while a block was pinned: permanent quarantine, the
         * instance can never drain and deinit must keep refusing. */
        ICHECK(info.lifecycle == GPIB_LC_STOPPING);
        ICHECK(gpib_get_fault_snapshot(&g_ctx, &snap) == GPIB_OK);
        ICHECK(snap.quarantined_blocks >= 1);
        ICHECK(gpib_deinit(&g_ctx) == GPIB_BUSY_RESOURCES);
        return;
    }

    /* Every other outcome drains: leftovers cancelled, then shutdown. */
    if (!has_hist && info.lifecycle == GPIB_LC_RUNNING) {
        gpib_status_t cst = gpib_request_cancel(&g_ctx, t);
        ICHECK(cst == GPIB_OK || cst == GPIB_ALREADY_TERMINAL);
    }
    {
        gpib_status_t sst = gpib_request_shutdown(&g_ctx);
        ICHECK(sst == GPIB_OK || sst == GPIB_ALREADY_STOPPING ||
               sst == GPIB_ALREADY_STOPPED);
    }
    for (int i = 0; i < 3; i++) {
        ICHECK(gpib_process(&g_ctx) == GPIB_OK);
    }
    ICHECK(gpib_query_status(&g_ctx, &info) == GPIB_OK);
    ICHECK(info.lifecycle == GPIB_LC_DRAINING);
    /* Exactly one ownership return per record. */
    gpib_status_t ab = gpib_result_ack(&g_ctx, b);
    ICHECK(ab == GPIB_OK || ab == GPIB_NO_RESULT || ab == GPIB_INVALID_HANDLE);
    gpib_status_t at = gpib_result_ack(&g_ctx, t);
    ICHECK(at == GPIB_OK || at == GPIB_NO_RESULT || at == GPIB_INVALID_HANDLE);
    ICHECK(gpib_query_status(&g_ctx, &info) == GPIB_OK);
    ICHECK(info.unacked_results == 0);
    ICHECK(gpib_deinit(&g_ctx) == GPIB_OK);
}

static void run_scenario(int kind, int phase, const int *seq, int len) {
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    char seqbuf[64];
    seqbuf[0] = '\0';
    for (int i = 0; i < len; i++) {
        char one[12];
        sprintf(one, "%s%s", (i == 0) ? "" : ",", ev_name(seq[i]));
        strcat(seqbuf, one);
    }
    sprintf(g_run_desc, "%s PH%d [%s]",
        (kind == K_TX) ? "TX" : "RX", phase, seqbuf);

    hard_reset();
    begin_instance();
    submit_stream(kind, &t, &b);
    if (phase == PH_MID) {
        ICHECK(gpib_process(&g_ctx) == GPIB_OK); /* live action, partial */
    } else if (phase == PH_POST) {
        for (int i = 0; i < 12; i++) {
            ICHECK(gpib_process(&g_ctx) == GPIB_OK);
        }
    }
    for (int i = 0; i < len; i++) {
        mock_push_event(&g_m, make_event(seq[i]));
    }
    for (int i = 0; i < 3; i++) {
        ICHECK(gpib_process(&g_ctx) == GPIB_OK);
    }
    verify_and_cleanup(kind, phase, seq, len, t, b);}

static uint32_t s_rng_state = 0xDEADBEEFu;

static uint32_t rng(uint32_t *s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

void suite_interleave(void) {
    int seq[5];
    /* Exhaustive short-sequence enumeration at every boundary. */
    for (int kind = 0; kind < 2; kind++) {
        for (int phase = 0; phase < 3; phase++) {
            for (int len = 2; len <= 3; len++) {
                int total = 1;
                for (int i = 0; i < len; i++) {
                    total *= E_COUNT;
                    seq[i] = 0;
                }
                for (int n = 0; n < total; n++) {
                    run_scenario(kind, phase, seq, len);
                    for (int i = len - 1; i >= 0; i--) {
                        seq[i] = (seq[i] + 1) % E_COUNT;
                        if (seq[i] != 0) {
                            break;
                        }
                    }
                }
            }
        }
    }
    /* Seeded random storms (deterministic seed). */
    for (int n = 0; n < 500; n++) {
        int kind = (int)(rng(&s_rng_state) & 1u);
        int phase = (int)(rng(&s_rng_state) % 3u);
        int len = 3 + (int)(rng(&s_rng_state) % 3u);
        for (int i = 0; i < len; i++) {
            seq[i] = (int)(rng(&s_rng_state) % E_COUNT);
        }
        run_scenario(kind, phase, seq, len);
    }
}
