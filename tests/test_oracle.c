#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_storage.h"
#include "../src/gpib_internal.h"
#include "oracle_peer.h"
#include "test_util.h"
#include "vbus.h"

/* T04 oracle suite: the core under test talks to a partner that shares
 * ZERO protocol code with it. All assertions are on the oracle's own
 * observations (command log, data log, waveform trace) -- if the core
 * and the oracle disagree, the TEST says who is right. */

static uint8_t s_tx[64];
static uint8_t s_reply[24];
static uint8_t s_rx_buf[64];
static uint8_t rx_follow[24]; /* O22: the second RX block gets its OWN
                               * buffer -- zero-copy ownership returns
                               * only at ACK (rev13 §6) */

/* Rev7 §5.4 / Rev10 §6.5: one clean-exit gate for every oracle
 * scenario. The near-limit print is diagnostics ONLY (the overflow
 * flag itself is the test condition). */
static void oracle_assert_clean(const oracle_peer_t *p) {
    if (p->violations != 0u || p->trace_overflow || p->dr_overflow) {
        fprintf(stderr, "[clean] viol=%u atn=%u confl=%u len=%zu ovf=%d "
            "dr_ovf=%d\n", p->violations, p->viol_atn, p->viol_conflict,
            (size_t)p->trace_len, (int)p->trace_overflow,
            (int)p->dr_overflow);
    }
    CHECK(p->violations == 0);
    CHECK(!p->trace_overflow);
    CHECK(!p->dr_overflow);
}

static void o_fill(void) {
    for (size_t i = 0; i < sizeof(s_tx); i++) {
        s_tx[i] = (uint8_t)(0x41u + (i % 26u));
    }
    for (size_t i = 0; i < sizeof(s_reply); i++) {
        s_reply[i] = (uint8_t)(0x61u + (i % 26u));
    }
}

static gpib_result_t o_wait(gpib_context_t *ctx, gpib_handle_t t) {
    gpib_result_t res;
    int guard = 0;
    memset(&res, 0, sizeof(res));
    while (gpib_result_peek(ctx, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000);
    return res;
}

/* O1: WRITE. Independent decode of the SEND SETUP prologue, exact data
 * bytes, END (EOI) on the last DAB, ATN phase discipline on the wire. */
static void oracle_o1_write(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 12;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 12);
    /* Independent prologue decode: MTA(controller), UNL, MLA(oracle). */
    CHECK(peer.n_cmds == 3);
    CHECK(peer.cmds[0].cmd == OC_MTA && peer.cmds[0].arg == 5);
    CHECK(peer.cmds[1].cmd == OC_UNL);
    CHECK(peer.cmds[2].cmd == OC_MLA && peer.cmds[2].arg == 6);
    /* Exact data bytes received by the independent acceptor. */
    CHECK(peer.rx_len == 12);
    CHECK(memcmp(peer.rx, s_tx, 12) == 0);
    /* Waveform: every traced DAV byte ran under the right ATN phase and
     * EOI accompanied EXACTLY the LAST data byte (T01: the old "at
     * least one EOI" check let the every-byte-EOI mutation survive). */
    {
        /* T01 precise END check: anchor on the first round the LAST
         * byte's DIO value appears on the wire (the source drives DIO
         * during SGNS/SDYS settle, before DAV; EOI accompanies the
         * whole final-byte window, not just the DAV assertion). */
        uint8_t last_byte = s_tx[11];
        size_t last_byte_first_round = 0;
        bool found_last = false;
        for (size_t i = 0; i < peer.trace_len; i++) {
            if (!peer.trace[i].atn &&
                peer.trace[i].dio == last_byte && !found_last) {
                last_byte_first_round = i;
                found_last = true;
            }
        }
        CHECK(found_last);
        bool saw_end = false;
        for (size_t i = 0; i < peer.trace_len; i++) {
            if (peer.trace[i].eoi && !peer.trace[i].atn) {
                if (i < last_byte_first_round) {
                    fprintf(stderr, "[eoi] round=%zu dav=%u dio=0x%02X "
                        "last_first=%zu last_byte=0x%02X\n",
                        i, peer.trace[i].dav, peer.trace[i].dio,
                        last_byte_first_round, last_byte);
                }
                CHECK(i >= last_byte_first_round);
                saw_end = true;
            }
        }
        CHECK(saw_end); /* the END is present */
    }
    oracle_assert_clean(&peer); /* DIO stability etc. */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O2: READ. Independent talker sources a reply with EOI; the core's
 * RECEIVE SETUP prologue and the received stream are verified. */
static void oracle_o2_read(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    size_t vlen = 0;
    const uint8_t *view;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = sizeof(s_reply);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI);
    CHECK(res.bytes_confirmed == sizeof(s_reply));
    CHECK(gpib_result_view(&ctx, b, &view, &vlen) == GPIB_OK);
    CHECK(view != NULL && vlen == sizeof(s_reply));
    CHECK(memcmp(view, s_reply, sizeof(s_reply)) == 0);
    /* Independent prologue decode: UNL, MLA(controller), MTA(oracle). */
    CHECK(peer.n_cmds == 3);
    CHECK(peer.cmds[0].cmd == OC_UNL);
    CHECK(peer.cmds[1].cmd == OC_MLA && peer.cmds[1].arg == 5);
    CHECK(peer.cmds[2].cmd == OC_MTA && peer.cmds[2].arg == 6);
    /* R6-02 (receive_stop_state_drive_trace): after the LAST data byte
     * there must be no round with the handshake fully released -- the
     * stop state (NRFD/NDAC held) follows the byte cycle continuously.
     * Rounds with DAV still asserted belong to the byte cycle itself
     * (the accept pulse legitimately releases NDAC there); the FIRST
     * DAV-free round is the AWNS->ANRS transition (488.1 allows exactly
     * one such round: NDAC returns on the next round); every round
     * after that must hold both lines. NRFD must NEVER release. */
    {
        size_t last_dav = 0;
        for (size_t i = 0; i < peer.trace_len; i++) {
            if (peer.trace[i].dav && !peer.trace[i].atn) {
                last_dav = i;
            }
        }
        REQUIRE(last_dav > 0u);
        bool transition_allowed = true;
        for (size_t i = last_dav + 1u; i < peer.trace_len; i++) {
            if (peer.trace[i].dav) {
                transition_allowed = true; /* still inside a byte cycle */
                continue;
            }
            CHECK(peer.trace[i].nrfd == 1u);
            if (transition_allowed) {
                transition_allowed = false; /* the AWNS->ANRS round */
            } else {
                CHECK(peer.trace[i].ndac == 1u);
            }
        }
    }
    CHECK(peer.trace_overflow == false); /* Rev6 §5.4 */
    oracle_assert_clean(&peer);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O3: SERIAL POLL. The oracle requests service (SRQ on the wire), the
 * core runs READ STATUS BYTE: SPE/MTA/one STB/SPD/UNT, RQS in bit 6,
 * SRQ cleared after the answer. */
static void oracle_o3_spoll(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    bool saw_srq = false;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_set_stb(&peer, 0x82u, true); /* B7|B1 with RQS when polled */
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* Let the SRQ line rise while idle. */
    for (int i = 0; i < 8; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    for (size_t i = 0; i < peer.trace_len; i++) {
        if (peer.trace[i].srq) {
            saw_srq = true;
        }
    }
    CHECK(saw_srq);
    CHECK(gpib_serial_poll(&ctx, 6, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.stb == (0x82u | 0x40u)); /* STB with RQS in B6 */
    /* The poll serviced us: SRQ is down, the epilogue ran. */
    for (int i = 0; i < 8; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(!peer.rsv);
    CHECK(!vbus_level(&bus, VBL_SRQ));
    {
        bool saw_spe = false;
        bool saw_spd = false;
        for (size_t i = 0; i < peer.n_cmds; i++) {
            if (peer.cmds[i].cmd == OC_SPE) {
                saw_spe = true;
            }
            if (peer.cmds[i].cmd == OC_SPD) {
                saw_spd = true;
            }
        }
        /* The epilogue must reach the wire even though the oracle
         * asserted EOI together with its status byte (16.2.6: EOI on
         * the STB is ignored by the controller). */
        CHECK(saw_spe && saw_spd);
    }
    oracle_assert_clean(&peer);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O4: PARALLEL POLL. Remote configure through the oracle's own PPC/PPE
 * interpretation, IDY answer iff ist == S, PPU disarm. */
static void oracle_o4_ppoll(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_set_pp(&peer, true, true); /* capable, ist = true */
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* Configure: S=1, line=4 -- the oracle must decode PPC+PPE(0x6C). */
    CHECK(gpib_ppoll_configure(&ctx, 6, 1, 4, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(peer.pp_configured);
    CHECK(peer.pp_sense == true);
    CHECK(peer.pp_line == 4);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* Poll 1: ist == S -> PPR5 (bit 4). */
    CHECK(gpib_parallel_poll(&ctx, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.stb == (1u << 4));
    /* L-01 regression: scope the check to THIS poll's journal window --
     * the whole test continues with Poll 2/unconfigure whose ATN
     * sequences legitimately release the line. */
    {
        size_t dr_end = peer.dr_n; /* captured NOW, after poll 1 */
        size_t last_idy = 0;
        for (size_t i = 0; i < dr_end; i++) {
            if (peer.dr_journal[i].atn && peer.dr_journal[i].eoi) {
                last_idy = i;
            }
        }
        REQUIRE(last_idy > 0);
        for (size_t i = last_idy + 1u; i < dr_end; i++) {
            CHECK(peer.dr_journal[i].atn == 1u); /* no gap after IDY */
        }
    }
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* Poll 2: ist != S -> passive false: zero on the wire. */
    oracle_set_pp(&peer, true, false);
    CHECK(gpib_parallel_poll(&ctx, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.stb == 0);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* Unconfigure: the oracle's own PPU handling. */
    CHECK(gpib_ppoll_unconfigure(&ctx, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(!peer.pp_configured);
    oracle_assert_clean(&peer);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O5: FINDLSTN. The oracle at 6 is found; the absent address is not.
 * The probe DAB byte lands in the oracle's receive log. */
static void oracle_o5_findlstn(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    uint8_t addrs[2] = { 7, 6 };
    bool results[2] = { false, false };
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    CHECK(gpib_findlstn(&ctx, addrs, 2, results, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(results[0] == false); /* 7: nobody there */
    CHECK(results[1] == true);  /* 6: the oracle listened */
    oracle_assert_clean(&peer);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O6: fault injection -- a wedged talker never asserts DAV; the READ
 * must end through the structural deadline with TIMEOUT and a partial
 * (here: zero) confirmed count, no hangs, no wire violations. */
static void oracle_o6_stalled_talker(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    oracle_stall_talker(&peer, true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = sizeof(s_reply);
    r.deadline_us = 500; /* absolute simulated us */
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(res.bytes_confirmed == 0);
    oracle_assert_clean(&peer);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O7: the System Controller's SEND IFC resets the partner on the wire:
 * addressing, serial-poll arm and the parallel-poll configuration. */
static void oracle_o7_ifc_reset(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_set_pp(&peer, true, true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* Establish state on the partner: configure PP remotely. */
    CHECK(gpib_ppoll_configure(&ctx, 6, 1, 2, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    REQUIRE(peer.pp_configured);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* IFC: the pulse must reach the wire and reset the partner. (Clear
     * the flag first: the SC power-on IFC already set it at start.) */
    peer.saw_ifc = false;
    CHECK(gpib_interface_clear(&ctx) == GPIB_OK);
    {
        int guard = 0;
        while (!peer.saw_ifc && guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(!peer.pp_configured);
    CHECK(!peer.listen);
    CHECK(!peer.talk);
    CHECK(!peer.spe);
    oracle_assert_clean(&peer);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O8 (R7-01/T8-02): per-DRIVE SC-line persistence. The port wrapper
 * records EVERY drive() call and flags (a) any request whose REN/IFC
 * disagrees with the controller FSM's own SC state, (b) any REN drop
 * while the TEST-OWNED intent flag says REN is enabled (independent of
 * ctx internals), (c) any phantom IFC outside the test-owned intent
 * window. Exercises WRITE, READ (stop-state), SPOLL, PPOLL and a full
 * IFC pulse under an enabled REN. */
static gpib_context_t *g_o8_ctx;
static void (*g_o8_orig_drive)(void *, const gpib_line_drive_t *);
static unsigned g_o8_ren_drops;
static unsigned g_o8_ifc_drops;
static unsigned g_o8_total;
static unsigned g_o8_ifc_true_drives;
static unsigned g_o8_phantom_ifc;
static bool g_o8_ren_intent;  /* test-owned: API-level expectation */
static bool g_o8_ifc_intent;
static void o8_wrap_drive(void *u, const gpib_line_drive_t *req) {
    if (g_o8_ctx != NULL) {
        g_o8_total++;
        if (g_o8_ctx->line.sc_ren && !req->ren) {
            g_o8_ren_drops++;
        }
        if (!g_o8_ctx->line.sc_ren && req->ren) {
            g_o8_ren_drops++; /* phantom assert is also a violation */
        }
        if (g_o8_ctx->line.sc_ifc && !req->ifc) {
            g_o8_ifc_drops++;
        }
        /* Intent model (T8-02.4): independent of ctx fields. */
        if (g_o8_ren_intent && !req->ren) {
            g_o8_ren_drops++;
        }
        if (req->ifc) {
            g_o8_ifc_true_drives++;
            if (!g_o8_ifc_intent) {
                g_o8_phantom_ifc++;
            }
        }
    }
    g_o8_orig_drive(u, req);
}

static void oracle_o8_sc_persistence(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    g_o8_ctx = &ctx;
    g_o8_orig_drive = peer.base.drive;
    peer.base.drive = o8_wrap_drive;
    g_o8_ren_drops = 0;
    g_o8_ifc_drops = 0;
    g_o8_total = 0;
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* REN on, then exercise the exchange-heavy operations: WRITE (with
     * its CMD->DATA apply_release boundary), READ (with the R6-02
     * stop-state direct drive) and a serial poll (the STB-ack path). */
    CHECK(gpib_remote_enable(&ctx, true) == GPIB_OK);
    {
        int guard = 0;
        /* Settle order matters: the power-on IFC pulse (T8 ~101 us)
         * and the first REN assert (T8 from time zero) overlap -- wait
         * the pulse OUT first, then the REN level on the WIRE (the FSM
         * flag flips one round before the wire follows). */
        while ((ctx.line.sc_ifc || vbus_level(&bus, VBL_IFC) ||
                !ctx.line.sc_ren || !vbus_level(&bus, VBL_REN)) &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(vbus_level(&bus, VBL_REN) == true); /* confirmed ON THE WIRE */
    /* T8-02.3: counters measure the exercised window only. */
    g_o8_ren_drops = 0;
    g_o8_ifc_drops = 0;
    g_o8_total = 0;
    g_o8_ifc_true_drives = 0;
    g_o8_phantom_ifc = 0;
    g_o8_ren_intent = true;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = sizeof(s_reply);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    for (int i = 0; i < 8; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK); /* stop-state idle holds */
    }
    CHECK(vbus_level(&bus, VBL_REN) == true); /* still enabled on the wire */
    /* T8-02.2: SPOLL under REN (its STB-ack drives are a separate
     * composer path). */
    CHECK(peer.bus->dio_active[1] == false); /* our drivers are off */
    CHECK(peer.bus->dio_active[0] == false); /* and the core's too */
    oracle_set_stb(&peer, 0x41u, true);
    CHECK(gpib_serial_poll(&ctx, 6, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.stb == (0x41u | 0x40u));
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* PPOLL under REN (the IDY window + PPR read). */
    oracle_set_pp(&peer, true, true);
    CHECK(gpib_ppoll_configure(&ctx, 6, 1, 2, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_parallel_poll(&ctx, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.stb == (1u << 2));
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* T8-02.1: a full IFC pulse under REN -- the composer must carry
     * BOTH SC lines through every pulse drive (REN persistent, IFC
     * continuous for the whole T8 window, no phantom pulses outside). */
    g_o8_ifc_intent = true;
    peer.saw_ifc = false; /* power-on pulse already set it */
    CHECK(gpib_interface_clear(&ctx) == GPIB_OK);
    {
        int guard = 0;
        while ((ctx.line.sc_ifc || !peer.saw_ifc) && guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    g_o8_ifc_intent = false;
    /* The R7-01 regression: REN (and the IFC pulse window) must be
     * carried by EVERY drive while the FSM holds them -- zero drops,
     * a real pulse happened, no phantom IFC outside the intent. */
    CHECK(g_o8_ren_drops == 0);
    CHECK(g_o8_ifc_drops == 0);
    CHECK(g_o8_phantom_ifc == 0);
    REQUIRE(g_o8_ifc_true_drives > 50u); /* the pulse truly exercised */
    REQUIRE(g_o8_total > 100u); /* the scenario really exercised drives */
    /* Clean release through the API is the ONLY legal way down; the
     * intent drops with the CALL (the settle drives after it legally
     * carry ren=false while the FSM walks SRAS->SRNS). */
    CHECK(gpib_remote_enable(&ctx, false) == GPIB_OK);
    g_o8_ren_intent = false;
    {
        int guard = 0;
        while ((ctx.line.sc_ren || vbus_level(&bus, VBL_REN)) &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(vbus_level(&bus, VBL_REN) == false);
    peer.base.drive = g_o8_orig_drive;
    g_o8_ctx = NULL;
    oracle_assert_clean(&peer);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
    /* T8-01: the off-bus detach left NOTHING of ours on the wire. */
    CHECK(vbus_participant_clean(&bus, 0));
}

/* O12 (T8-01): electrical detach safety. The port's safe_detached is
 * PROOF now: after shutdown the core must have released every output at
 * the DRAINING boundary (even with REN live), and a port that refuses
 * to certify blocks deinit with BUSY_PORT_TEARDOWN_PENDING. */
static bool o12_refuse_detach(void *user) {
    (void)user;
    return false; /* the port never certifies */
}

static void oracle_o12_detach_safety(void) {
    {
        /* Scenario A: shutdown with REN ACTIVE -- deinit must succeed
         * through the electrical detach, leaving the wire clean. */
        GPIB_CONTEXT_STORAGE_DEFINE(ctx);
        vbus_t bus;
        oracle_peer_t peer;
        gpib_handle_t t = { 0, 0, 0, 0, 0 };
        gpib_handle_t b = { 0, 0, 0, 0, 0 };
        vbus_init(&bus);
        vbus_attach_as(&bus, 0);
        oracle_init(&peer, &bus, 0, 6);
        {
            gpib_init_params_t ip;
            memset(&ip, 0, sizeof(ip));
            ip.system_controller = true;
            ip.own_address.primary = 5;
            ip.port = &peer.base;
            CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
            CHECK(gpib_start(&ctx) == GPIB_OK);
        }
        CHECK(gpib_remote_enable(&ctx, true) == GPIB_OK);
        {
            int guard = 0;
            /* R9 §4.1: wait for the FSM flag AND the physical WIRE --
             * a port may lag the model by a round. */
            while ((!ctx.line.sc_ren || !vbus_level(&bus, VBL_REN)) &&
                   guard++ < 20000) {
                CHECK(gpib_process(&ctx) == GPIB_OK);
            }
            REQUIRE(guard < 20000);
        }
        REQUIRE(vbus_level(&bus, VBL_REN)); /* proven before shutdown */
        CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
        CHECK(gpib_process(&ctx) == GPIB_OK);
        CHECK(gpib_deinit(&ctx) == GPIB_OK); /* detach earned */
        CHECK(vbus_participant_clean(&bus, 0));
        CHECK(!vbus_level(&bus, VBL_REN)); /* physically released */
        (void)t;
        (void)b;
    }
    {
        /* Scenario B: a port that never certifies -- deinit is blocked
         * (BUSY_PORT_TEARDOWN_PENDING), never a silent wipe. */
        GPIB_CONTEXT_STORAGE_DEFINE(ctx);
        vbus_t bus;
        oracle_peer_t peer;
        vbus_init(&bus);
        vbus_attach_as(&bus, 0);
        oracle_init(&peer, &bus, 0, 6);
        peer.base.safe_detached = o12_refuse_detach;
        {
            gpib_init_params_t ip;
            memset(&ip, 0, sizeof(ip));
            ip.system_controller = true;
            ip.own_address.primary = 5;
            ip.port = &peer.base;
            CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
            CHECK(gpib_start(&ctx) == GPIB_OK);
        }
        CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
        CHECK(gpib_process(&ctx) == GPIB_OK);
        /* R9 §4.2: the exact BUSY status AND the machine-readable
         * reason -- not just "not OK". */
        CHECK(gpib_deinit(&ctx) == GPIB_BUSY_RESOURCES);
        CHECK(ctx.last_busy_reason == GPIB_BUSY_PORT_TEARDOWN_PENDING);
        CHECK(ctx.lifecycle == (uint8_t)GPIB_LC_DRAINING); /* still live */
        /* Recovery: once the port can certify again, deinit succeeds --
         * the refusal must be a busy state, never a dead end. */
        peer.base.safe_detached = oracle_port_safe_detached;
        CHECK(gpib_deinit(&ctx) == GPIB_OK);
        CHECK(vbus_participant_clean(&bus, 0));
        oracle_assert_clean(&peer);
    }
}

/* O13 (R9 §3 recommendation, receive_final_ndac_ack_on_wire): the LAST
 * byte's completion sequence, round by round on the wire: (1) the accept
 * pulse round -- NDAC released while DAV still held and the byte still
 * driven; (2) the source reacts -- DAV drops and the DIO drivers go off;
 * (3) from then on the stop state holds both handshake lines. No API
 * completion may precede the physical acknowledgement of the talker. */
static void oracle_o13_final_ack_sequence(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = sizeof(s_reply);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI);
    {
        /* Trace granularity note: the oracle reacts to the accept pulse
         * inside the same read_lines call that first observes it, so
         * the wire picture per round is: (1) the END byte held (DAV+EOI,
         * byte on the lines, NDAC asserted); (2) the acknowledgement
         * round -- NDAC released (the pulse) and the source ALREADY
         * reacted: DAV down, DIO drivers off; (3) from the next round
         * the stop state holds both lines. */
        size_t held = 0;
        bool found = false;
        for (size_t i = 0; i + 1u < peer.trace_len; i++) {
            const oracle_trace_t *tr = &peer.trace[i];
            const oracle_trace_t *nx = &peer.trace[i + 1u];
            if (!tr->atn && tr->dav && tr->eoi && tr->ndac &&
                tr->dio == s_reply[sizeof(s_reply) - 1u] &&
                !nx->atn && !nx->dav && !nx->ndac && nx->dio == 0u) {
                held = i;
                found = true;
                break;
            }
        }
        if (!found) {
            size_t from = peer.trace_len > 12 ? peer.trace_len - 12 : 0;
            for (size_t i = from; i < peer.trace_len; i++) {
                fprintf(stderr, "[o13] %zu: atn=%u eoi=%u dav=%u nrfd=%u "
                    "ndac=%u dio=0x%02X\n", i, peer.trace[i].atn,
                    peer.trace[i].eoi, peer.trace[i].dav,
                    peer.trace[i].nrfd, peer.trace[i].ndac,
                    peer.trace[i].dio);
            }
        }
        REQUIRE(found);
        /* (3) stop state from the round after the acknowledgement.
         * Rev10 §6.3: NO new DAV may appear after the terminal byte
         * (the source is done; the controller never sources here). */
        for (size_t i = held + 2u; i < peer.trace_len; i++) {
            CHECK(!peer.trace[i].dav);
            CHECK(peer.trace[i].nrfd == 1u);
            CHECK(peer.trace[i].ndac == 1u);
        }
        /* (4) Rev10 §4.3 / rev11 N11-05: the per-DRIVE journal,
         * time-stamped with the snapshot round. In the acknowledge
         * round itself the legal NDAC pulse appears (nrfd held, ndac
         * released); from the FIRST drive of the NEXT round on, the
         * stop state must hold BOTH lines (nrfd && ndac) until a
         * control sequence legitimately ends it. A single erroneous
         * release between neighbouring drives must fail. */
        {
            uint16_t from_round = (uint16_t)(held + 2u);
            for (size_t i = 0; i < peer.dr_n; i++) {
                const oracle_drive_rec_t *jr = &peer.dr_journal[i];
                if (jr->round < from_round) {
                    continue;
                }
                if (jr->atn) {
                    break; /* next control sequence: stop state ends */
                }
                CHECK(jr->nrfd == 1u && jr->ndac == 1u); /* BOTH held */
            }
            CHECK(peer.dr_overflow == false);
        }
    }
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O14 (R6-04 fault injection, requestctl_stuck_port): with a frozen
 * snapshot clock the internal FINDRQS can never progress; the composite
 * must hit its bound, cancel+drain the action and return
 * BUSY_RESOURCES -- leaving a usable instance behind (no hang, no
 * uncontrolled live action writing into instance memory). */
static void oracle_o14_requestctl_stuck(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    uint8_t addrs[1] = { 6 };
    size_t found = 0;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_set_stb(&peer, 0x40u, true); /* a real requester exists */
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    {
        int guard = 0;
        while ((ctx.line.ctrl.main != GPIB_C_CACS) && guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    /* Freeze the SNAPSHOT clock: every handshake timing window (T1, the
     * empty-bus probe, deadlines) stops elapsing -- the internal FINDRQS
     * cannot progress, exactly a wedged port. */
    peer.freeze_clock = true;
    peer.trace_enabled = false; /* wedged rounds exhaust any ring */
    CHECK(gpib_requestctl_pass(&ctx, addrs, 1, &found, &t, &b) ==
          GPIB_BUSY_RESOURCES); /* bounded failure, no hang */
    peer.freeze_clock = false;
    peer.trace_enabled = true;
    /* Rev10 §7 (the auditor's external probe, now in CI): a port whose
     * read_lines FAILS outright -- the composite must still bound. */
    peer.read_fail = true;
    {
        uint8_t addrs3[1] = { 6 };
        size_t found3 = 0;
        CHECK(gpib_requestctl_pass(&ctx, addrs3, 1, &found3, &t, &b) ==
              GPIB_BUSY_RESOURCES);
    }
    peer.read_fail = false;
    /* Rev13 §7 (REQUESTCTL fault lifecycle): after the bounded failure
     * and recovery, the ledger holds NO retained slots -- a fresh
     * REQUESTCTL runs to completion through the same composite (after
     * which the core is no longer CIC, so this must come LAST). */
    {
        uint8_t addrs2[1] = { 6 };
        size_t found2 = 1;
        CHECK(gpib_requestctl_pass(&ctx, addrs2, 1, &found2, &t, &b) ==
              GPIB_OK);
        CHECK(found2 == 0);
        /* AUD-15: no asynchronous TCT follows the poll -- nothing to
         * wait for; the handles are simply not populated. The instance
         * keeps CIC and the lifecycle closes cleanly below. */
        for (int i = 0; i < 8; i++) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
    }
    /* The instance is still usable: the drain recovered, shutdown and
     * deinit complete normally once the clock runs again. */
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    {
        int guard = 0;
        while (ctx.lifecycle != (uint8_t)GPIB_LC_DRAINING && guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
    oracle_assert_clean(&peer);
}

/* O15 (final audit C1, use-after-ACK): cancel mid-WRITE, ACK, then
 * OVERWRITE the application buffer -- the oracle must not receive a
 * single byte of the new contents; a following WRITE must complete with
 * its own exact count, never the cancelled block's counter. */
static void oracle_o15_cancel_use_after_ack(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    static uint8_t buf[40];
    size_t rx_before;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(buf, 'A', sizeof(buf));
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = buf;
    r.tx_len = sizeof(buf);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    /* Let a few bytes through, then cancel the ACTIVE write. */
    {
        int guard = 0;
        while (ctx.action.state != (uint8_t)GPIB_ACTION_ACTIVE &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
        for (int i = 0; i < 16; i++) {
            CHECK(gpib_process(&ctx) == GPIB_OK); /* mid-stream */
        }
    }
    /* Cancel EXACTLY mid-byte (deterministic): the oracle's acceptor
     * HOLDS the byte (NRFD released, NDAC asserted) -- the core's SH
     * waits with DAV up, so the stop path provably sees an in-flight
     * byte: uncertain flag + the ATN takeover (C3/N11-04). */
    peer.hold_byte = true;
    {
        int guard = 0;
        while (!(peer.trace_len > 0u &&
                 peer.trace[peer.trace_len - 1u].dav &&
                 !peer.trace[peer.trace_len - 1u].atn) &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
        for (int i = 0; i < 8; i++) {
            CHECK(gpib_process(&ctx) == GPIB_OK); /* soak mid-byte */
        }
    }
    /* N12-03: per-DRIVE ATN watch from this exact point -- the stop
     * path's takeover assert must be the FIRST post-cancel drive and
     * NO drive may release ATN until the result exists. Linear scan:
     * each round checks only the NEWLY added journal entries (the
     * index advances; no quadratic re-walk). */
    {
        size_t mark = peer.dr_n;
        CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
        {
            int guard = 0;
            gpib_result_t cres;
            size_t checked = mark;
            while (gpib_result_peek(&ctx, t, &cres) != GPIB_OK &&
                   guard++ < 20000) {
                CHECK(gpib_process(&ctx) == GPIB_OK);
                for (; checked < peer.dr_n; checked++) {
                    if (!peer.dr_journal[checked].atn) {
                        fprintf(stderr,
                            "ATN gap journal=%zu round=%u nrfd=%u "
                            "ndac=%u\n", checked,
                            peer.dr_journal[checked].round,
                            peer.dr_journal[checked].nrfd,
                            peer.dr_journal[checked].ndac);
                    }
                    CHECK(peer.dr_journal[checked].atn == 1u);
                }
            }
            REQUIRE(guard < 20000);
        }
    }
    peer.hold_byte = false;
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_CANCELED);
    CHECK(res.last_byte_uncertain == true); /* C3: aborted mid-byte */
    /* N11-04/C3 sensitivity: from the abort on, ATN must stay asserted
     * (the takeover assert + the CACS idle hold are one continuous
     * level; the mutation that drops the takeover assert opens a
     * one-round ATN=0 gap right after the mid-byte cancel). */
    {
        bool atn_seen_down = false;
        for (int i = 0; i < 40; i++) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
            if (!vbus_level(&bus, VBL_ATN)) {
                atn_seen_down = true;
            }
        }
        CHECK(atn_seen_down == false);
    }
    rx_before = peer.rx_len;
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* The application reuses its buffer -- the zombie must NOT source
     * it: not a single 'Z' may reach the wire. */
    memset(buf, 'Z', sizeof(buf));
    for (int i = 0; i < 200; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(peer.rx_len == rx_before); /* no byte after the ACK */
    for (size_t i = 0; i < peer.rx_len; i++) {
        CHECK(peer.rx[i] == (uint8_t)'A'); /* and nothing but the prefix */
    }
    /* A fresh WRITE completes with ITS OWN count (the old bug: the
     * zombie accounted the cancelled 40-byte block into this result). */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = buf;
    r.tx_len = 4;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 4);
    CHECK(peer.rx_len == rx_before + 4u);
    for (size_t i = rx_before; i < peer.rx_len; i++) {
        CHECK(peer.rx[i] == (uint8_t)'Z'); /* the new contents, delivered */
    }
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    {
        int guard = 0;
        while (ctx.lifecycle != (uint8_t)GPIB_LC_DRAINING &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    {
        gpib_status_t d = gpib_deinit(&ctx);
        CHECK(d == GPIB_OK);
    }
}

/* O16 (final audit C2, sticky stop latch): after a cancel the latch
 * must be CONSUMED -- a second cancel of another transaction works, new
 * work dispatches, shutdown reaches DRAINING (not an eternal stop). */
static void oracle_o16_stop_latch_consumed(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t t2 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    static uint8_t buf[16];
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(buf, 'Q', sizeof(buf));
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = buf;
    r.tx_len = 16;
    CHECK(gpib_submit(&ctx, &r, &t1, &b1) == GPIB_OK);
    {
        int guard = 0;
        while (ctx.action.state != (uint8_t)GPIB_ACTION_ACTIVE &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
        for (int i = 0; i < 8; i++) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
    }
    CHECK(gpib_request_cancel(&ctx, t1) == GPIB_OK);
    res = o_wait(&ctx, t1);
    CHECK(res.reason == GPIB_TERM_CANCELED);
    CHECK(gpib_result_ack(&ctx, b1) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t1) == GPIB_OK);
    CHECK(!ctx.stop_request.active); /* C2: consumed at accounting */
    /* A second WRITE must DISPATCH (the old bug: the latch kept the
     * slot -- and effectively all later work -- from ever starting). */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = buf;
    r.tx_len = 4;
    CHECK(gpib_submit(&ctx, &r, &t2, &b2) == GPIB_OK);
    res = o_wait(&ctx, t2);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 4);
    /* And a cancel of THIS one works too (the latch is reusable). */
    CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t2) == GPIB_OK);
    /* Shutdown completes: DRAINING reachable, deinit clean. */
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
    oracle_assert_clean(&peer);
}

/* O17 (final audit C5, session hijack): an OPEN READ stream at address
 * 6, cancelled between blocks; a new READ to a DIFFERENT address must
 * never inherit the stream -- no data substitution between devices. */
static void oracle_o17_session_hijack(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* READ #1 (address 6): first block of 4 fills, the stream stays
     * OPEN (backpressure between blocks). */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = 4;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    {
        int guard = 0;
        while (gpib_result_peek(&ctx, b, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    REQUIRE(ctx.line.rx_session_open); /* the stream is live */
    /* Cancel the OPEN transaction between blocks (shortcut path). */
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_CANCELED);
    CHECK(!ctx.line.rx_session_open); /* C5: died with its transaction */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* READ #2 to a DIFFERENT, absent address: must time out with ZERO
     * bytes -- the old bug completed it with device 6's data ("4567890")
     * without ever sending a RECEIVE SETUP. */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 7;
    r.rx_data = s_rx_buf;
    r.rx_cap = 8;
    r.deadline_us = 800;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(res.bytes_confirmed == 0);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* O18 (final audit M05/M7 leftovers): a TCT arriving while OUR talker
 * is NOT addressed must never hand this core the bus (F16 gate), and a
 * properly addressed TCT must (positive control). */
static void oracle_o18_tct_addressing_gate(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    uint8_t bare[1] = { 0x09u };              /* TCT alone */
    uint8_t addressed[2] = { 0x45u, 0x09u };  /* MTA(5) then TCT */
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* Bare TCT (no MTA(own) before it): the takeover pulse must NOT
     * fire -- a stray 0x09 must not hand us the bus. */
    CHECK(gpib_send_commands(&ctx, bare, 1, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* N11-03: the WIRE carries ATN after the sequence -- 16.2.1/16.2.9:
     * ATN stays asserted until another sequence changes it. Verified on
     * the physical bus level across idle rounds (the mutation that
     * restored the old `action.state == EMPTY` gate drives ATN=0 here
     * and MUST fail this check). */
    for (int i = 0; i < 8; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
        CHECK(vbus_level(&bus, VBL_ATN) == true);
    }
    CHECK(!ctx.line.ctrl_tct_pulse);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* Positive control: MTA(own) + TCT -> the pulse fires exactly once
     * per takeover. */
    CHECK(gpib_send_commands(&ctx, addressed, 2, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* The pulse fires in feed_self_addressing and is immediately
     * consumed by the C FSM (we are the SC -- already CIC), so the
     * transient flag is not observable here. The ADDRESSING is: our
     * talker must sit in TADS/TACS after MTA(own); the takeover pulse
     * path itself is proven externally by the dual-core TC-01. */
    CHECK(ctx.line.tk.state == GPIB_T_TADS ||
          ctx.line.tk.state == GPIB_T_TACS);
    CHECK(peer.n_cmds == 3); /* 0x09, then MTA(5), TCT */
    CHECK(peer.cmds[1].raw == 0x45u);
    CHECK(peer.cmds[2].raw == 0x09u);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
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

/* O19 (rev11 N11-01/N11-02): an OPEN READ stream OWNS the bus executor.
 * A foreign transaction submitted between blocks stays queued (no
 * RECEIVE SETUP on the wire, no session break); after the stream ends
 * with EOI the foreign work dispatches and completes on its own. ONE
 * RECEIVE SETUP per response, counted on the independent command log. */
static void oracle_o19_stream_ownership(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t t1b = { 0, 0, 0, 0, 0 };
    gpib_handle_t t2 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* READ #1 (device 6): 4-byte first block, stream OPEN between
     * blocks (backpressure). */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = 4;
    CHECK(gpib_submit(&ctx, &r, &t1, &b1) == GPIB_OK);
    {
        int guard = 0;
        while (gpib_result_peek(&ctx, b1, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    REQUIRE(ctx.line.rx_session_open);
    REQUIRE(peer.n_cmds == 3); /* UNL, MLA(5), MTA(6): ONE setup so far */
    /* Foreign transaction mid-stream: submitted and queued -- it must
     * NOT touch the wire (the old bug: a second RECEIVE SETUP for the
     * device 7 probe broke the stream and device 6's response was
     * later resumed through a THIRD setup). The foreign WRITE targets
     * the SAME present device: the arbitration is about TRANSACTION
     * ownership, not the address (an absent target would add an
     * unrelated deadline interaction). */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 4;
    CHECK(gpib_submit(&ctx, &r, &t2, &b2) == GPIB_OK);
    for (int i = 0; i < 60; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK); /* idle/backpressure rounds */
    }
    CHECK(peer.n_cmds == 3); /* STILL one setup: nothing dispatched */
    CHECK(ctx.line.rx_session_open); /* the stream is intact */
    /* Resume the stream: the next block completes the response WITH
     * EOI -- still exactly ONE RECEIVE SETUP on the wire. */
    CHECK(gpib_submit_block(&ctx, t1, s_rx_buf, 20, &t1b) == GPIB_OK);
    {
        int guard = 0;
        while (gpib_result_peek(&ctx, t1, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI);
    CHECK(res.bytes_confirmed == sizeof(s_reply));
    CHECK(peer.n_cmds == 3); /* ONE RECEIVE per response, proven */
    CHECK(gpib_result_ack(&ctx, t1b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b1) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t1) == GPIB_OK);
    /* NOW the foreign work dispatches and completes on its own. */
    {
        int guard = 0;
        while (gpib_result_peek(&ctx, t2, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 4);
    CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t2) == GPIB_OK);
    oracle_assert_clean(&peer);
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

/* O20 (rev11 N11-06): delayed executor quiescence. A port that cannot
 * certify quiescence blocks STOPPING->DRAINING (no silent lifecycle
 * advance); once the port recovers, the lifecycle completes and deinit
 * succeeds -- the ledger is intact after recovery. */
static void oracle_o20_delayed_quiescence(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* The port stops certifying quiescence: shutdown wedges in
     * STOPPING -- no silent DRAINING, deinit refuses. */
    peer.quiesce_fail = true;
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    for (int i = 0; i < 64; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(ctx.lifecycle == (uint8_t)GPIB_LC_STOPPING);
    CHECK(gpib_deinit(&ctx) != GPIB_OK);
    /* Recovery: the port certifies again -> DRAINING -> deinit OK. */
    peer.quiesce_fail = false;
    {
        int guard = 0;
        while (ctx.lifecycle != (uint8_t)GPIB_LC_DRAINING &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
    CHECK(vbus_participant_clean(&bus, 0));
    oracle_assert_clean(&peer);
}

/* O21 (N11-06 residual): the REQUESTCTL composite under a port that
 * cannot certify quiescence -- the composite itself completes (it
 * never needs lifecycle DRAINING), and the lifecycle tail then closes
 * once the port recovers. */
static void oracle_o21_requestctl_delayed_quiesce(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    uint8_t addrs[1] = { 6 };
    size_t found = 0;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_set_stb(&peer, 0x44u, true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    peer.quiesce_fail = true; /* the port certifies nothing meanwhile */
    CHECK(gpib_requestctl_pass(&ctx, addrs, 1, &found, &t, &b) ==
          GPIB_OK); /* the composite does not need lifecycle quiescence */
    CHECK(found == 0);
    /* AUD-15: no asynchronous TCT follows -- the composite stops at the
     * poll; nothing to wait for or ack. */
    for (int i = 0; i < 8; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    /* Lifecycle with the wedged port: shutdown parks in STOPPING. */
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    for (int i = 0; i < 32; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(ctx.lifecycle == (uint8_t)GPIB_LC_STOPPING);
    peer.quiesce_fail = false; /* recovery */
    {
        int guard = 0;
        while (ctx.lifecycle != (uint8_t)GPIB_LC_DRAINING &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
    oracle_assert_clean(&peer);
}

/* O22 (rev12 N12-01/N12-02): generation-boundary stream ownership and
 * zero-length block rejection.
 *  N12-01: a transaction whose generation exceeds 16 bits (reachable
 *  after 65 536 ACK cycles of a slot) must still own its RX session --
 *  the 16-bit rx_session_gen truncation used to disown it permanently.
 *  N12-02: submit_block(len==0) must be rejected in both directions
 *  and leave the stream usable. */
static void oracle_o22_boundaries(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_handle_t tb = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    uint8_t junk[4];
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* White-box: force a high generation BEFORE the submit books the
     * block (exactly like the rev12 probe: set right after init) --
     * 65537 = one past the 16-bit wrap. The session ownership must
     * carry the FULL width. */
    ctx.txn[0].generation = 65537u;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = 4;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    {
        int guard = 0;
        while (gpib_result_peek(&ctx, b, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    REQUIRE(ctx.line.rx_session_open);
    CHECK(ctx.line.rx_session_gen == 65537u); /* full width carried */
    /* N12-02 (RX): the zero-length block is REJECTED, the stream lives. */
    CHECK(gpib_submit_block(&ctx, t, junk, 0, &tb) ==
          GPIB_INVALID_ARGUMENT);
    /* N12-02 (TX direction): a fresh WRITE txn, then a zero-length TX
     * block -- rejected too, no ledger record, the WRITE completes. */
    {
        gpib_handle_t tw = { 0, 0, 0, 0, 0 };
        gpib_handle_t bw = { 0, 0, 0, 0, 0 };
        gpib_handle_t b0 = { 0, 0, 0, 0, 0 };
        gpib_request_t w;
        static uint8_t wbuf[4] = { 1, 2, 3, 4 };
        memset(&w, 0, sizeof(w));
        w.kind = GPIB_REQ_WRITE;
        w.address.primary = 6;
        w.tx_data = wbuf;
        w.tx_len = 4;
        CHECK(gpib_submit(&ctx, &w, &tw, &bw) == GPIB_OK);
        /* The open stream still owns the bus: the WRITE queues behind
         * it (N11-01) -- so first finish the stream. */
        CHECK(gpib_submit_block(&ctx, t, rx_follow, 20, &tb) == GPIB_OK);
        {
            int guard = 0;
            while (gpib_result_peek(&ctx, t, &res) != GPIB_OK &&
                   guard++ < 20000) {
                CHECK(gpib_process(&ctx) == GPIB_OK);
            }
            REQUIRE(guard < 20000);
        }
        CHECK(res.reason == GPIB_TERM_COMPLETED);
        CHECK(res.end_cause == GPIB_END_EOI);
        CHECK(res.bytes_confirmed == sizeof(s_reply));
        CHECK(peer.n_cmds == 3); /* still ONE RECEIVE SETUP */
        /* zero-length TX block: rejected, no phantom record */
        CHECK(gpib_submit_block(&ctx, tw, wbuf, 0, &b0) ==
              GPIB_INVALID_ARGUMENT);
        /* the queued WRITE completes normally afterwards */
        res = o_wait(&ctx, tw);
        CHECK(res.reason == GPIB_TERM_COMPLETED);
        CHECK(res.bytes_confirmed == 4);
        CHECK(gpib_result_ack(&ctx, bw) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, tw) == GPIB_OK);
        /* Both RX block buffers verified SEPARATELY (zero-copy: the
         * first block's memory is still owned until ITS ack). */
        CHECK(memcmp(s_rx_buf, s_reply, 4) == 0);
        CHECK(memcmp(rx_follow, &s_reply[4], 20) == 0);
    }
    CHECK(gpib_result_ack(&ctx, tb) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
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

/* O23 (rev14 T14-01, findlstn_timing): an INDEPENDENT timing oracle --
 * the per-round wire trace measures every ATN TRUE->FALSE release and
 * the following re-assertion: the NDAC sample happens in between, at
 * the earliest at release+1500 us. A build with a shortened window
 * fails HERE directly, without needing a stale talker to collide. */
static void oracle_o23_findlstn_timing(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    uint8_t addrs[2] = { 7, 6 }; /* absent, present: both settle */
    bool results[2] = { false, false };
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    CHECK(gpib_findlstn(&ctx, addrs, 2, results, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(results[0] == false);
    CHECK(results[1] == true);
    /* The settle proof: for every ATN release round (the trace shows
     * atn 1 -> 0) find the next re-assertion round (atn 0 -> 1): the
     * sample ran no earlier than release + 1500 us, and the next
     * command byte follows the sample -- so the gap between the
     * observations must cover the window. Scan only up to the round
     * the result published: the action's last wire event is the
     * epilogue UNL (atn=1); the post-action release belongs to the
     * idle handover and must not be measured. */
    {
        size_t len = peer.trace_len;
        unsigned windows = 0;
        for (size_t i = 1; i < len; i++) {
            if (peer.trace[i - 1u].atn == 1u && peer.trace[i].atn == 0u) {
                size_t j = i + 1u;
                while (j < len && peer.trace[j].atn == 0u) {
                    j++;
                }
                if (j >= len) {
                    /* The FINAL release (after the epilogue UNL, at the
                     * action's very end): no settle or sample follows
                     * it -- the idle handover owns that window. Skip. */
                    continue;
                }
                CHECK((unsigned)(j - i) >= 1500u); /* >= 1.5 ms */
                windows++;
            }
        }
        /* Two probes, each with one MLA release; the final epilogue UNL
         * re-asserts after the last settle -- exactly 2 measured. */
        CHECK(windows == 2u);
    }
    oracle_assert_clean(&peer);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
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

/* O24 (rev14 T14-03, findlstn_stale_talker): a local, easily
 * diagnosable version of the stress scenario -- the instrument still
 * holds an UNCONSUMED reply (a cancelled READ left its talker armed),
 * and FINDLSTN must (a) still detect correctly, (b) source ZERO DIO
 * bytes of its own after MLA, (c) raise no wire conflicts. */
static unsigned o24_dio_drives;
static bool o24_last_atn;
static void (*o24_orig_drive_data)(void *, const uint8_t *);
static void (*o24_orig_drive)(void *, const gpib_line_drive_t *);
static void o24_wrap_drive_data(void *u, const uint8_t *v) {
    /* T14-03 metric: the core's own DIO drive WITH ATN FALSE -- the
     * ATN command bytes (UNL/MLA) legitimately use DIO; the settle and
     * sample window must source NOTHING. */
    if (v != NULL && !o24_last_atn) {
        o24_dio_drives++;
    }
    o24_orig_drive_data(u, v);
}

static void o24_wrap_drive(void *u, const gpib_line_drive_t *req) {
    o24_last_atn = req->atn;
    o24_orig_drive(u, req);
}

static void oracle_o24_findlstn_stale_talker(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    uint8_t addrs[2] = { 7, 6 };
    bool results[2] = { false, false };
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    /* Interrupt a READ mid-reply: the instrument keeps the unconsumed
     * remainder (a stale talker on the wire). */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = 4;
    r.deadline_us = 40000;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    {
        int guard = 0;
        while (gpib_result_peek(&ctx, b, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_CANCELED);
    REQUIRE(peer.tx_pos < peer.tx_len); /* the stale reply IS there */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* FINDLSTN under the wrap: ZERO own DIO bytes, correct results,
     * zero conflicts -- the stale talker may keep its drivers, we must
     * never contend with them. */
    o24_orig_drive_data = peer.base.drive_data;
    o24_orig_drive = peer.base.drive;
    peer.base.drive_data = o24_wrap_drive_data;
    peer.base.drive = o24_wrap_drive;
    o24_dio_drives = 0;
    o24_last_atn = false;
    CHECK(gpib_findlstn(&ctx, addrs, 2, results, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    peer.base.drive_data = o24_orig_drive_data;
    peer.base.drive = o24_orig_drive;
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(results[0] == false);
    CHECK(results[1] == true);
    CHECK(o24_dio_drives == 0u); /* ZERO own DIO at ATN false */
    CHECK(peer.violations == 0u); /* and collided with nothing */
    oracle_assert_clean(&peer);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
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

/* O25 (AUD-03): a READ completing by rx_min_bytes (END_LENGTH) CLOSES
 * the stream session -- a following transaction dispatches immediately;
 * the old bug left rx_session_open forever and starved the bus. */
static void oracle_o25_length_close(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, 12, false); /* no EOI: the LENGTH target terminates */
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = 8;
    r.rx_min_bytes = 8; /* stream target: terminal at 8 bytes */
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_LENGTH);
    CHECK(res.bytes_confirmed == 8);
    CHECK(!ctx.line.rx_session_open); /* AUD-03: session released */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* A following WRITE dispatches and completes -- no starvation. */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 4;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 4);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
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

/* O26 (AUD-04): an OPEN stream between blocks honors the transaction
 * deadline even with NO active action: the owner sweep terminalizes it
 * with TIMEOUT (the old bug left it waiting for a buffer forever). */
static void oracle_o26_open_deadline(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    oracle_arm_reply(&peer, s_reply, sizeof(s_reply), true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx_buf;
    r.rx_cap = 4;
    r.deadline_us = 3000; /* absolute, reachable within the guard */
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    {
        int guard = 0;
        while (gpib_result_peek(&ctx, b, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    REQUIRE(ctx.line.rx_session_open); /* stream open, no buffer staged */
    /* No second block: the deadline must terminalize the whole txn. */
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_TIMEOUT);
    CHECK(!ctx.line.rx_session_open);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
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

/* O27 (AUD-07): SRQ rides EVERY exchange-scoped drive: raising rsv
 * mid-WRITE must put SRQ on the wire within the same operation -- the
 * old phase-local zero dropped it for the whole TX. */
static void oracle_o27_srq_during_tx(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    bool saw_srq_in_tx = false;
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 16;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    /* Raise the request while the write is in flight. */
    {
        int guard = 0;
        while (ctx.action.state != (uint8_t)GPIB_ACTION_ACTIVE &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
        oracle_set_stb(&peer, 0x40u, true);
        size_t mark = peer.trace_len;
        while (gpib_result_peek(&ctx, t, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
        for (size_t i = mark; i < peer.trace_len; i++) {
            if (peer.trace[i].srq) {
                saw_srq_in_tx = true;
            }
        }
    }
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(saw_srq_in_tx); /* SRQ visible on the wire DURING the TX */
    CHECK(vbus_level(&bus, VBL_SRQ) == true); /* and still after */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    oracle_assert_clean(&peer);
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

/* O28 (AUD-10): per-drive ATN continuity after SEND COMMAND -- the
 * journal must show NO atn=0 drive between the last command byte and
 * the idle re-assert (a one-call gap is invisible to snapshots). */
static void oracle_o28_cmd_atn_continuity(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    vbus_t bus;
    oracle_peer_t peer;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    uint8_t dcl[1] = { 0x14u };
    vbus_init(&bus);
    vbus_attach_as(&bus, 0);
    oracle_init(&peer, &bus, 0, 6);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &peer.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    CHECK(gpib_send_commands(&ctx, dcl, 1, &t, &b) == GPIB_OK);
    res = o_wait(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* Journal scan from the LAST atn=1 drive of the command phase: no
     * atn=0 entry may follow it before the observation ends (the idle
     * drive keeps the hold; the C model stays CACS until the next
     * sequence). */
    {
        size_t last_atn = 0;
        for (size_t i = 0; i < peer.dr_n; i++) {
            if (peer.dr_journal[i].atn) {
                last_atn = i;
            }
        }
        REQUIRE(last_atn > 0);
        for (size_t i = last_atn + 1u; i < peer.dr_n; i++) {
            CHECK(peer.dr_journal[i].atn == 1u); /* AUD-10: no gap */
        }
        CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    }
    oracle_assert_clean(&peer);
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

void suite_oracle(void) {
    o_fill();
    fprintf(stderr, "[o]1\n"); oracle_o1_write();
    fprintf(stderr, "[o]2\n"); oracle_o2_read();
    fprintf(stderr, "[o]3\n"); oracle_o3_spoll();
    fprintf(stderr, "[o]4\n"); oracle_o4_ppoll();
    fprintf(stderr, "[o]5\n"); oracle_o5_findlstn();
    fprintf(stderr, "[o]6\n"); oracle_o6_stalled_talker();
    fprintf(stderr, "[o]7\n"); oracle_o7_ifc_reset();
    fprintf(stderr, "[o]8\n"); oracle_o8_sc_persistence();
    fprintf(stderr, "[o]12\n"); oracle_o12_detach_safety();
    fprintf(stderr, "[o]13\n"); oracle_o13_final_ack_sequence();
    fprintf(stderr, "[o]14\n"); oracle_o14_requestctl_stuck();
    fprintf(stderr, "[o]15\n"); oracle_o15_cancel_use_after_ack();
    fprintf(stderr, "[o]16\n"); oracle_o16_stop_latch_consumed();
    fprintf(stderr, "[o]17\n"); oracle_o17_session_hijack();
    fprintf(stderr, "[o]18\n"); oracle_o18_tct_addressing_gate();
    fprintf(stderr, "[o]19\n"); oracle_o19_stream_ownership();
    fprintf(stderr, "[o]20\n"); oracle_o20_delayed_quiescence();
    fprintf(stderr, "[o]21\n"); oracle_o21_requestctl_delayed_quiesce();
    fprintf(stderr, "[o]22\n"); oracle_o22_boundaries();
    fprintf(stderr, "[o]23\n"); oracle_o23_findlstn_timing();
    fprintf(stderr, "[o]24\n"); oracle_o24_findlstn_stale_talker();
    fprintf(stderr, "[o]25\n"); oracle_o25_length_close();
    fprintf(stderr, "[o]26\n"); oracle_o26_open_deadline();
    fprintf(stderr, "[o]27\n"); oracle_o27_srq_during_tx();
    fprintf(stderr, "[o]28\n"); oracle_o28_cmd_atn_continuity();
}
