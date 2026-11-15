#include <stdio.h>
#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_storage.h"
#include "../src/gpib_internal.h"
#include "oracle_peer.h"
#include "test_util.h"
#include "vbus.h"

/* CHAOS STRESS: the adversarial bus. One real oracle instrument, plus
 * a THIEF participant that injects random IFC pulses, ATN steals and
 * wiring faults (stuck acceptor, wedged talker, random SRQ) while the
 * APPLICATION runs a randomized workload with random servicing gaps.
 *
 * Properties checked CONTINUOUSLY:
 *  - the oracle's own invariants (per-drive journal, wire conflicts,
 *    DIO stability, trace overflow) -- any breach fails the run;
 *  - two-sided BYTE ACCOUNTING: every completed WRITE's bytes match the
 *    oracle's receive log exactly; every completed READ's bytes match
 *    the armed reply model;
 *  - TERMINAL REASONS are whitelisted per operation and per the chaos
 *    that fired during it (PHASE_INTERRUPTED/IFC only allowed when the
 *    thief actually stole/pulsed in that window);
 *  - no hang: every operation is deadline-bounded and completes within
 *    the guard;
 *  - ledger drains to all-terminal before every ack cycle ends.
 *
 * Deterministic: seeded LCG; a failing seed reproduces by number. */

#define CH_THIEF 2
#define CH_MAX_ROUNDS 600000u

typedef struct chaos_port {
    oracle_peer_t peer;  /* the real instrument (address 6) */
    gpib_port_t base;    /* wrapper handed to the core */
    uint32_t rng;
    unsigned ifc_left;   /* rounds of an injected IFC pulse */
    unsigned steal_left; /* rounds of an injected ATN steal */
    unsigned op_ifc;     /* chaos that fired during the CURRENT op */
    unsigned op_steal;
    unsigned op_stall;
    unsigned rounds;
} chaos_port_t;

static uint32_t ch_rnd(chaos_port_t *c) {
    /* xorshift32: deterministic, seedable */
    uint32_t x = c->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    c->rng = x;
    return x;
}

static bool ch_read_lines(void *user, gpib_line_snap_t *snap) {
    chaos_port_t *c = (chaos_port_t *)user;
    vbus_t *bus = c->peer.bus;
    c->rounds++;
    /* Adversarial rolls (independent per round). */
    if (c->ifc_left == 0 && c->steal_left == 0) {
        uint32_t r = ch_rnd(c);
        if ((r % 997u) == 0u) {
            c->ifc_left = 2u + (r % 3u); /* a foreign IFC pulse */
            c->op_ifc++;
        } else if ((r % 1231u) == 0u) {
            c->steal_left = 3u + (r % 8u); /* a foreign controller */
            c->op_steal++;
        }
    }
    if (c->ifc_left > 0) {
        vbus_assert(bus, CH_THIEF, VBL_IFC);
        c->ifc_left--;
        if (c->ifc_left == 0) {
            vbus_release(bus, CH_THIEF, VBL_IFC);
        }
    }
    if (c->steal_left > 0) {
        vbus_assert(bus, CH_THIEF, VBL_ATN);
        c->steal_left--;
        if (c->steal_left == 0) {
            vbus_release(bus, CH_THIEF, VBL_ATN);
        }
    }
    /* Occasional instrument-side chaos: random service requests. */
    if ((ch_rnd(c) % 613u) == 0u) {
        oracle_set_stb(&c->peer, (uint8_t)(0x40u | (ch_rnd(c) & 0x3Fu)),
                       (ch_rnd(c) & 1u) != 0u);
    }
    /* The instrument's own round, then the snapshot. */
    return oracle_port_read_lines(&c->peer, snap);
}

static void chaos_init(chaos_port_t *c, vbus_t *bus, uint32_t seed) {
    memset(c, 0, sizeof(*c));
    oracle_init(&c->peer, bus, 0, 6);
    c->rng = seed ? seed : 0x9E3779B9u;
    vbus_attach_as(bus, CH_THIEF);
    /* Copy the oracle's port contract, override the read path. */
    c->base = c->peer.base;
    c->base.user = c;
    c->base.read_lines = ch_read_lines;
}

/* ---- application-side accounting ---- */

static uint8_t ch_tx_log[8192];
static size_t ch_tx_len;
static uint8_t ch_rxbuf[64];
static uint8_t ch_rx2[64];
static uint8_t ch_wrbuf[64];
static uint8_t ch_reply[24];

static void ch_fill(void) {
    for (size_t i = 0; i < sizeof(ch_reply); i++) {
        ch_reply[i] = (uint8_t)(0x30u + (i % 10u));
    }
}

static gpib_result_t ch_wait(gpib_context_t *ctx, chaos_port_t *c,
    gpib_handle_t t) {
    gpib_result_t res;
    int guard = 0;
    memset(&res, 0, sizeof(res));
    while (gpib_result_peek(ctx, t, &res) != GPIB_OK &&
           guard++ < 400000) {
        CHECK(gpib_process(ctx) == GPIB_OK);
        REQUIRE(c->rounds < CH_MAX_ROUNDS);
        if (c->peer.violations != 0) {
            fprintf(stderr, "[violate] at guard=%d seed-round=%u "
                "atn=%u confl=%u ifc_left=%u steal_left=%u\n",
                guard, c->rounds, c->peer.viol_atn,
                c->peer.viol_conflict, c->ifc_left, c->steal_left);
        }
        REQUIRE(c->peer.violations == 0);
        if (guard == 20000) {
            fprintf(stderr, "[hang] act=%d seq=%d ctrl=%d dl=%llu now=%llu"
                " sess=%d ss=%u sg=%u stop=%d lc=%d ifc0=%d\n",
                (int)ctx->action.state, (int)ctx->line.seq,
                (int)ctx->line.ctrl.main,
                (unsigned long long)ctx->action.deadline_us,
                (unsigned long long)c->peer.now_us,
                (int)ctx->line.rx_session_open,
                (unsigned)ctx->line.rx_session_slot,
                (unsigned)ctx->line.rx_session_gen,
                (int)ctx->stop_request.active,
                (int)ctx->lifecycle, (int)ctx->ifc_request);
        }
    }
    REQUIRE(guard < 400000);
    return res;
}

static void ch_check_writedata(const chaos_port_t *c) {
    /* Two-sided accounting: the oracle received EXACTLY the bytes the
     * application confirmed, in order, nothing extra. */
    if (c->peer.rx_len != ch_tx_len) {
        size_t i = 0;
        while (i < c->peer.rx_len && i < ch_tx_len &&
               c->peer.rx[i] == ch_tx_log[i]) {
            i++;
        }
        fprintf(stderr, "[acct] diverge at %zu: oracle_len=%zu app_len=%zu "
            "oracle[%zu]=0x%02X app[%zu]=0x%02X\n",
            i, (size_t)c->peer.rx_len, ch_tx_len,
            i, i < c->peer.rx_len ? c->peer.rx[i] : 0,
            i, i < ch_tx_len ? ch_tx_log[i] : 0);
    }
    CHECK(c->peer.rx_len == ch_tx_len);
    if (c->peer.rx_len == ch_tx_len) {
        CHECK(memcmp(c->peer.rx, ch_tx_log, ch_tx_len) == 0);
    }
}

static void chaos_seed_run(uint32_t seed) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    chaos_port_t chaos;
    gpib_result_t res;
    gpib_request_t r;
    ch_tx_len = 0;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    chaos_init(&chaos, &bus, seed);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &chaos.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* Settle the power-on IFC/REN state machine. */
    {
        int guard = 0;
        while (ctx.line.ctrl.main != GPIB_C_CACS && guard++ < 400000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
            REQUIRE(chaos.rounds < CH_MAX_ROUNDS);
        }
        REQUIRE(guard < 400000);
    }
    for (int op = 0; op < 24; op++) {
        uint32_t dice0 = ch_rnd(&chaos); /* roll first for the banner */
        fprintf(stderr, "[seed=%u op=%d kind=%d]\n", (unsigned)seed, op,
            (int)(dice0 % 6u));
        unsigned ifc0 = chaos.op_ifc;
        unsigned steal0 = chaos.op_steal;
        gpib_handle_t t = { 0, 0, 0, 0, 0 };
        gpib_handle_t b = { 0, 0, 0, 0, 0 };
        uint32_t dice = dice0;
        int kind = (int)(dice % 6u);
        gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
        bool have_b2 = false;
        /* Random servicing gap: the owner may be late to the pump. */
        int gap = (int)(ch_rnd(&chaos) % 7u);
        for (int i = 0; i < gap; i++) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        if (kind == 0 || kind == 1) {
            /* WRITE: 1..32 bytes, counter-derived pattern. */
            size_t n = 1u + (ch_rnd(&chaos) % 32u);
            for (size_t i = 0; i < n; i++) {
                ch_wrbuf[i] = (uint8_t)(0xC0u + ((ch_tx_len + i) & 0x3Fu));
            }
            memset(&r, 0, sizeof(r));
            r.kind = GPIB_REQ_WRITE;
            r.address.primary = 6;
            r.tx_data = ch_wrbuf;
            r.tx_len = n;
            r.deadline_us = 40000; /* bounded: chaos may stall us */
            CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
            res = ch_wait(&ctx, &chaos, t);
            bool interrupted = chaos.op_ifc != ifc0 ||
                               chaos.op_steal != steal0;
            CHECK(res.reason == GPIB_TERM_COMPLETED ||
                  res.reason == GPIB_TERM_TIMEOUT ||
                  res.reason == GPIB_TERM_IFC ||
                  (interrupted &&
                   res.reason == GPIB_TERM_PHASE_INTERRUPTED));
            /* Two-sided accounting, partial case included: every
             * CONFIRMED byte reached the wire (the handshake completed
             * for exactly those), whatever the terminal reason was. */
            {
                size_t k = res.bytes_confirmed;
                CHECK(k <= n);
                if (ch_tx_len + k <= sizeof(ch_tx_log)) {
                    memcpy(&ch_tx_log[ch_tx_len], ch_wrbuf, k);
                    ch_tx_len += k;
                }
            }
            ch_check_writedata(&chaos);
        } else if (kind == 2) {
            /* READ: armed reply, block 1 (maybe smaller than the
             * response), then either the stream ends with EOI or the
             * remainder block is submitted (the F05 contract: an open
             * stream yields its TXN result only at stream end). */
            size_t cap = 4u + (ch_rnd(&chaos) % 20u);
            bool ch_have_b2 = false;
            oracle_arm_reply(&chaos.peer, ch_reply, sizeof(ch_reply),
                             true);
            memset(&r, 0, sizeof(r));
            r.kind = GPIB_REQ_READ;
            r.address.primary = 6;
            r.rx_data = ch_rxbuf;
            r.rx_cap = cap;
            r.deadline_us = 40000;
            CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
            {
                int guard = 0;
                while (gpib_result_peek(&ctx, b, &res) != GPIB_OK &&
                       guard++ < 400000) {
                    CHECK(gpib_process(&ctx) == GPIB_OK);
                    REQUIRE(chaos.rounds < CH_MAX_ROUNDS);
                    REQUIRE(chaos.peer.violations == 0);
                }
                REQUIRE(guard < 400000);
            }
            if (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
                /* Stream still open: feed the remainder into its own
                 * buffer (block buffers never overlap). b2 is the
                 * OP-LEVEL handle -- no shadowing. */
                CHECK(gpib_submit_block(&ctx, t, ch_rx2,
                                        sizeof(ch_reply), &b2) == GPIB_OK);
                have_b2 = true;
                ch_have_b2 = true;
            }
            res = ch_wait(&ctx, &chaos, t);
            bool interrupted = chaos.op_ifc != ifc0 ||
                               chaos.op_steal != steal0;
            CHECK(res.reason == GPIB_TERM_COMPLETED ||
                  res.reason == GPIB_TERM_TIMEOUT ||
                  res.reason == GPIB_TERM_IFC ||
                  (interrupted &&
                   res.reason == GPIB_TERM_PHASE_INTERRUPTED));
            if (res.reason == GPIB_TERM_COMPLETED &&
                res.bytes_confirmed == sizeof(ch_reply) && ch_have_b2) {
                CHECK(memcmp(ch_rxbuf, ch_reply, cap) == 0);
                CHECK(memcmp(ch_rx2, &ch_reply[cap],
                             sizeof(ch_reply) - cap) == 0);
            } else if (res.reason == GPIB_TERM_COMPLETED &&
                       res.bytes_confirmed == sizeof(ch_reply)) {
                CHECK(memcmp(ch_rxbuf, ch_reply, sizeof(ch_reply)) == 0);
            }
        } else if (kind == 3) {
            /* SPOLL: whatever STB/rsv chaos left us with. */
            CHECK(gpib_serial_poll(&ctx, 6, &t, &b) == GPIB_OK);
            res = ch_wait(&ctx, &chaos, t);
            CHECK(res.reason == GPIB_TERM_COMPLETED ||
                  res.reason == GPIB_TERM_TIMEOUT ||
                  res.reason == GPIB_TERM_IFC ||
                  res.reason == GPIB_TERM_PHASE_INTERRUPTED);
            /* NOTE: the RQS bit reflects rsv at STB-send time; the
             * oracle's rsv/stb_sent legitimately mutate afterwards
             * (SPD re-arm, random chaos re-set), so no post-hoc RQS
             * cross-check is sound here -- O3 covers the semantics. */
        } else if (kind == 4) {
            /* FINDLSTN over [absent, present]. Since the normative
             * 17.6.5 rework (no DAB, NDAC sampling after a 1.5 ms
             * ATN-false settle) the sweep is SAFE with a stale talker
             * on the bus: the controller sources nothing on DIO, so
             * the old two-talker conflict window is gone. This branch
             * deliberately does NOT drain the stale reply first -- the
             * audit's unmask scenario must stay covered. */
            uint8_t addrs[2] = { 7, 6 };
            bool results[2] = { false, false };
            CHECK(gpib_findlstn(&ctx, addrs, 2, results, &t, &b) ==
                  GPIB_OK);
            res = ch_wait(&ctx, &chaos, t);
            CHECK(res.reason == GPIB_TERM_COMPLETED ||
                  res.reason == GPIB_TERM_TIMEOUT ||
                  res.reason == GPIB_TERM_IFC ||
                  res.reason == GPIB_TERM_PHASE_INTERRUPTED);
            if (res.reason == GPIB_TERM_COMPLETED) {
                CHECK(results[1] == true); /* the oracle is always there */
                /* Normative 17.6.5: NO probe byte reaches the listener
                 * anymore -- nothing to account. */
            }
        } else {
            /* Cancel a live WRITE mid-flight (the C1/C3 paths under
             * random chaos timing). */
            size_t n = 24u;
            for (size_t i = 0; i < n; i++) {
                ch_wrbuf[i] = (uint8_t)(0x80u + i);
            }
            memset(&r, 0, sizeof(r));
            r.kind = GPIB_REQ_WRITE;
            r.address.primary = 6;
            r.tx_data = ch_wrbuf;
            r.tx_len = n;
            r.deadline_us = 40000;
            CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
            for (int i = 0; i < (int)(ch_rnd(&chaos) % 12u); i++) {
                CHECK(gpib_process(&ctx) == GPIB_OK);
            }
            (void)gpib_request_cancel(&ctx, t); /* may be terminal already */
            res = ch_wait(&ctx, &chaos, t);
            CHECK(res.reason == GPIB_TERM_COMPLETED ||
                  res.reason == GPIB_TERM_CANCELED ||
                  res.reason == GPIB_TERM_TIMEOUT ||
                  res.reason == GPIB_TERM_IFC ||
                  res.reason == GPIB_TERM_PHASE_INTERRUPTED);
            {
                size_t k = res.bytes_confirmed; /* partial delivery
                                                 * counts on ANY reason */
                CHECK(k <= n);
                if (ch_tx_len + k <= sizeof(ch_tx_log)) {
                    memcpy(&ch_tx_log[ch_tx_len], ch_wrbuf, k);
                    ch_tx_len += k;
                }
            }
            ch_check_writedata(&chaos);
        }
        if (have_b2) {
            gpib_status_t ab = gpib_result_ack(&ctx, b2);
            if (ab != GPIB_OK) {
                fprintf(stderr, "[b2ack] st=%d slot=%u gen=%u\n", (int)ab,
                    (unsigned)b2.slot, (unsigned)b2.generation);
            }
            CHECK(ab == GPIB_OK);
        }
        CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
        /* The wire partner stayed clean through the whole operation. */
        CHECK(chaos.peer.violations == 0);
        CHECK(!chaos.peer.trace_overflow);
        CHECK(!chaos.peer.dr_overflow);
    }
    /* Lifecycle close: everything drains, the detach is earned. */
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    {
        int guard = 0;
        while (ctx.lifecycle != (uint8_t)GPIB_LC_DRAINING &&
               guard++ < 400000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
            REQUIRE(chaos.rounds < CH_MAX_ROUNDS);
        }
        REQUIRE(guard < 400000);
    }
    {
        gpib_status_t d = gpib_deinit(&ctx);
        if (d != GPIB_OK) {
            fprintf(stderr, "[deinit] st=%d busy=%d sess=%d\n", (int)d,
                (int)ctx.last_busy_reason,
                (int)ctx.line.rx_session_open);
            for (int i = 0; i < 4; i++) {
                fprintf(stderr, "[deinit] txn[%d]=%d ", i,
                    (int)ctx.txn[i].state);
            }
            fprintf(stderr, "\n");
            for (int i = 0; i < 8; i++) {
                fprintf(stderr, "blk[%d]=%d ", i,
                    (int)ctx.block[i].state);
            }
            fprintf(stderr, "\n");
        }
        CHECK(d == GPIB_OK);
    }
}

void suite_stress(void) {
    ch_fill();
    /* Fixed seed set: deterministic, reproducible by number. */
    chaos_seed_run(1u);
    chaos_seed_run(2u);
    chaos_seed_run(0xACE1u);
    chaos_seed_run(0xBADC0DEu);
    chaos_seed_run(0x5EED5u);
    chaos_seed_run(0xFACE8u);
}
