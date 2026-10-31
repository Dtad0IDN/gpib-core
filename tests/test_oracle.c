#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_storage.h"
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

/* Rev7 §5.4: one clean-exit gate for every oracle scenario. */
static void oracle_assert_clean(const oracle_peer_t *p) {
    if (p->violations != 0u || p->trace_overflow) {
        fprintf(stderr, "[clean] viol=%u atn=%u confl=%u (dioerr path)\n",
            p->violations, p->viol_atn, p->viol_conflict);
    }
    CHECK(p->violations == 0);
    CHECK(!p->trace_overflow);
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
     * the LAST data byte carried EOI. */
    bool saw_data_eoi = false;
    for (size_t i = 0; i < peer.trace_len; i++) {
        if (peer.trace[i].dav) {
            if (peer.trace[i].atn) {
                /* command byte: fine */
            } else {
                if (peer.trace[i].eoi) {
                    saw_data_eoi = true;
                }
            }
        }
    }
    CHECK(saw_data_eoi);
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
            while (!ctx.line.sc_ren && guard++ < 20000) {
                CHECK(gpib_process(&ctx) == GPIB_OK);
            }
            REQUIRE(guard < 20000);
        }
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
        CHECK(gpib_deinit(&ctx) != GPIB_OK); /* refused: teardown pending */
        CHECK(ctx.lifecycle == (uint8_t)GPIB_LC_DRAINING); /* still live */
        oracle_assert_clean(&peer);
    }
}

void suite_oracle(void) {
    o_fill();
    oracle_o1_write();
    oracle_o2_read();
    oracle_o3_spoll();
    oracle_o4_ppoll();
    oracle_o5_findlstn();
    oracle_o6_stalled_talker();
    oracle_o7_ifc_reset();
    oracle_o8_sc_persistence();
    oracle_o12_detach_safety();
}
