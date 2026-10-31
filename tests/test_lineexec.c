#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_lines.h"
#include "test_util.h"

/* End-to-end core tests over the line port: the full public API drives the
 * SH/AH protocol engine against a simulated instrument on the vbus. */

static uint8_t s_tx[64];
static uint8_t s_reply[64];
static uint8_t s_rx[64];

static void start_line(gpib_context_t *ctx, mock_line_port_t *p) {
    gpib_init_params_t ip;
    mock_lines_init(p);
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = true;
    ip.own_address.primary = 5; /* controller address */
    ip.port = &p->base;
    CHECK(gpib_init(ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
}

static void fill(void) {
    for (size_t i = 0; i < sizeof(s_tx); i++) {
        s_tx[i] = (uint8_t)(0xA0u + i);
        s_reply[i] = (uint8_t)(0x10u + i);
    }
}

/* Pump the instance until the transaction result is ready (T09: the
 * guard bound is a REQUIRE -- a wedged fixture must not "pass" a green
 * tail of meaningless checks -- and the result is always initialized,
 * so a caller never reads garbage). */
static gpib_result_t wait_txn(gpib_context_t *ctx, gpib_handle_t t) {
    gpib_result_t res;
    int guard = 0;
    memset(&res, 0, sizeof(res));
    while (gpib_result_peek(ctx, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000);
    return res;
}

/* Acknowledge a completed transaction: block record first, then the
 * transaction itself (LED ack order). */
static void drain_and_ack(gpib_context_t *ctx, gpib_handle_t b,
    gpib_handle_t t) {
    CHECK(gpib_result_ack(ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(ctx, t) == GPIB_OK);
}
/* E2E-01: write 64 bytes to the instrument at address 6. The instrument
 * must observe UNL, MLA(6), MTA(5) and then the exact data stream. */
static void e2e_01_write(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    fill();
    start_line(&ctx, &port);
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = sizeof(s_tx);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == sizeof(s_tx));
    /* F02 refinement: the data phase released ATN -> the C MODEL is in
     * standby (CSBS), tracking the wire the sequencer drove. */
    CHECK(ctx.line.ctrl.main == GPIB_C_CSBS);
    /* Instrument side: addressing prologue then the payload (16.2.2:
     * MTA(controller), UNL, MLA(device)). */
    CHECK(port.ins.n_cmdlog == 3);
    CHECK(port.ins.cmdlog[0].cls == GPIB_CMD_MTA && port.ins.cmdlog[0].arg == 5);
    CHECK(port.ins.cmdlog[1].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[2].cls == GPIB_CMD_MLA && port.ins.cmdlog[2].arg == 6);
    CHECK(port.ins.rxlen == sizeof(s_tx));
    CHECK(memcmp(port.ins.rx, s_tx, sizeof(s_tx)) == 0);
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-02: read 64 bytes from the instrument; the reply ends with EOI. */
static void e2e_02_read(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    const uint8_t *view = NULL;
    size_t vlen = 0;
    fill();
    memset(s_rx, 0, sizeof(s_rx));
    start_line(&ctx, &port);
    mock_lines_arm_reply(&port, s_reply, sizeof(s_reply));
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = s_rx;
    r.rx_cap = sizeof(s_rx);
    r.rx_min_bytes = 0; /* EOI-terminated */
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI);
    CHECK(res.bytes_confirmed == sizeof(s_reply));
    CHECK(gpib_result_view(&ctx, b, &view, &vlen) == GPIB_OK);
    CHECK(view != NULL); /* defensive before memcmp */
    CHECK(vlen == sizeof(s_reply));
    CHECK(memcmp(view, s_reply, sizeof(s_reply)) == 0);
    /* The instrument was addressed to talk, the controller to listen. */
    CHECK(port.ins.n_cmdlog == 3);
    CHECK(port.ins.cmdlog[1].cls == GPIB_CMD_MLA && port.ins.cmdlog[1].arg == 5);
    CHECK(port.ins.cmdlog[2].cls == GPIB_CMD_MTA && port.ins.cmdlog[2].arg == 6);
    drain_and_ack(&ctx, b, t);
    /* F13 (488.2 16.2.6): after a completed RECEIVE the acceptor stays
     * in the stop state -- NRFD and NDAC held TRUE on the wire until the
     * next control sequence (ATN) or a new action. */
    gpib_process(&ctx);
    CHECK(vbus_level(port.ins.bus, VBL_NRFD) == true);
    CHECK(vbus_level(port.ins.bus, VBL_NDAC) == true);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-03: an external controller steals ATN mid-transfer: P1 yield with the
 * confirmed prefix, clean line release, sequencer returns to idle after the
 * thief releases ATN. */
static void e2e_03_atn_steal(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    int guard = 0;
    fill();
    start_line(&ctx, &port);
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = sizeof(s_tx);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    /* Let a few bytes go out, then steal the bus. */
    while (guard++ < 200) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
        if (port.ins.rxlen >= 4) {
            break;
        }
    }
    CHECK(port.ins.rxlen >= 4);
    mock_lines_steal_atn(&port, true);
    for (int i = 0; i < 200; i++) {
        gpib_process(&ctx);
    }
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_PHASE_INTERRUPTED);
    CHECK(res.bytes_confirmed > 0);
    CHECK(res.bytes_confirmed < sizeof(s_tx));
    /* The instrument captured at most the one interrupted in-flight byte
     * beyond the confirmed prefix. */
    CHECK(port.ins.rxlen >= res.bytes_confirmed);
    CHECK(port.ins.rxlen <= res.bytes_confirmed + 1u);
    CHECK(memcmp(port.ins.rx, s_tx, res.bytes_confirmed) == 0);
    /* Sequencer parked in LISTEN_CMDS; releasing ATN returns it to idle. */
    CHECK(gpib_query_status(&ctx, NULL) != 0); /* context alive */
    mock_lines_steal_atn(&port, false);
    for (int i = 0; i < 100; i++) {
        gpib_process(&ctx);
    }
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-04: shutdown mid-write terminates the action at a byte boundary with
 * the confirmed prefix; no line is left asserted. */
static void e2e_04_shutdown_mid_write(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_status_info_t info;
    int guard = 0;
    fill();
    start_line(&ctx, &port);
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = sizeof(s_tx);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    while (guard++ < 200) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
        if (port.ins.rxlen >= 8) {
            break;
        }
    }
    CHECK(port.ins.rxlen >= 8);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    for (int i = 0; i < 200; i++) {
        gpib_process(&ctx);
    }
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.lifecycle == GPIB_LC_DRAINING);
    CHECK(gpib_result_peek(&ctx, b, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_SHUTDOWN);
    /* The instrument may have captured at most the one in-flight byte. */
    CHECK(res.bytes_confirmed >= port.ins.rxlen - 1u);
    CHECK(res.bytes_confirmed <= port.ins.rxlen);
    CHECK(port.ins.rxlen < sizeof(s_tx));
    CHECK(!vbus_level(port.ins.bus, VBL_DAV));
    CHECK(!vbus_level(port.ins.bus, VBL_ATN));
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-05: SEND COMMAND control sequence (488.2 16.2.1) — DCL universal
 * clear and addressed GET reach the instrument's command log. */
static void e2e_05_send_commands(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    fill();
    start_line(&ctx, &port);
    /* DEVICE CLEAR, all devices: ATN + DCL (16.2.9.2). */
    {
        uint8_t dcl[1] = { 0x14u };
        CHECK(gpib_send_commands(&ctx, dcl, 1, &t, &b) == GPIB_OK);
    }
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(port.ins.n_cmdlog >= 1);
    CHECK(port.ins.cmdlog[0].cls == GPIB_CMD_DCL);
    drain_and_ack(&ctx, b, t);
    /* TRIGGER, selected device (16.2.19.1): MTA(ctrl), UNL, MLA(6), GET. */
    {
        uint8_t trig[4] = { 0x45u, 0x3Fu, 0x26u, 0x08u };
        CHECK(gpib_send_commands(&ctx, trig, 4, &t, &b) == GPIB_OK);
    }
    res = wait_txn(&ctx, t);
    CHECK(port.ins.n_cmdlog == 5);
    CHECK(port.ins.cmdlog[1].cls == GPIB_CMD_MTA); /* MTA(5) first */
    CHECK(port.ins.cmdlog[2].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[3].cls == GPIB_CMD_MLA);
    CHECK(port.ins.cmdlog[3].arg == 6);
    CHECK(port.ins.cmdlog[4].cls == GPIB_CMD_GET);
    CHECK(port.ins.dt_triggers == 1); /* DT1: addressed listener triggered */
    CHECK(port.ins.dc_clears == 1);   /* DC1: DCL cleared */
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-06: SEND IFC (16.2.8) — the pulse clears the instrument's interface
 * functions (talker/listener idle, poll pair disarmed) within the T8 hold;
 * REN control (16.2.10.2/16.2.11.2) toggles the line persistently. */
static void e2e_06_ifc_ren(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    int guard = 0;
    fill();
    start_line(&ctx, &port);
    /* Address the instrument as listener (leaves lads set), arm a poll. */
    {
        uint8_t trig[4] = { 0x45u, 0x3Fu, 0x26u, 0x18u }; /* MTA5 UNL MLA6 SPE */
        CHECK(gpib_send_commands(&ctx, trig, 4, &t, &b) == GPIB_OK);
    }
    (void)wait_txn(&ctx, t);
    CHECK(port.ins.addr.lads);
    CHECK(port.ins.talker.sp == GPIB_T_SPMS);
    drain_and_ack(&ctx, b, t);

    /* REN control: enable then disable (persistent line state). */
    CHECK(gpib_remote_enable(&ctx, true) == GPIB_OK);
    guard = 0;
    while (!vbus_level(port.ins.bus, VBL_REN) && guard++ < 500) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(vbus_level(port.ins.bus, VBL_REN));
    CHECK(gpib_remote_enable(&ctx, false) == GPIB_OK);
    guard = 0;
    while (vbus_level(port.ins.bus, VBL_REN) && guard++ < 500) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(!vbus_level(port.ins.bus, VBL_REN));

    /* SEND IFC: pulse on the wire, instrument functions reset within t4. */
    CHECK(gpib_interface_clear(&ctx) == GPIB_OK);
    guard = 0;
    while (guard++ < 500) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
        if (!port.ins.addr.lads && port.ins.talker.sp == GPIB_T_SPIS) {
            break;
        }
    }
    CHECK(guard < 500);
    CHECK(!port.ins.addr.lads);      /* LIDS */
    CHECK(!port.ins.addr.tads);      /* TIDS */
    CHECK(port.ins.talker.sp == GPIB_T_SPIS); /* poll pair disarmed */
    CHECK(!ctx.ifc_request);         /* pulse consumed after the T8 hold */
    /* The controller re-seized the bus (IFC -> CADS -> CACS). */
    CHECK(ctx.line.ctrl.main != GPIB_C_CIDS);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-07: RECEIVE stop conditions (488.2 16.2.6.2-4) — NL, comma and
 * semicolon terminate the response message at the matching byte (the
 * stop byte IS part of the data); EOI still works as the default. */
static void e2e_07_stop_conditions(void) {
    static const uint8_t resp[] = "1.5,2.5;3.5\nTRAILER";
    for (int mode = 0; mode < 4; mode++) {
        GPIB_CONTEXT_STORAGE_DEFINE(ctx);
        mock_line_port_t port;
        gpib_request_t r;
        gpib_handle_t t = { 0, 0, 0, 0, 0 };
        gpib_handle_t b = { 0, 0, 0, 0, 0 };
        gpib_result_t res;
        uint8_t rxbuf[32];
        size_t expect_len = 0;
        gpib_end_cause_t expect_end = GPIB_END_CHAR;
        fill();
        memset(rxbuf, 0, sizeof(rxbuf));
        start_line(&ctx, &port);
        mock_lines_arm_reply(&port, resp, sizeof(resp) - 1u);
        memset(&r, 0, sizeof(r));
        r.kind = GPIB_REQ_READ;
        r.address.primary = 6;
        r.rx_data = rxbuf;
        r.rx_cap = sizeof(rxbuf);
        r.stop_cond = (gpib_stop_cond_t)mode;
        if (mode == 0) { /* EOI default: the armed reply ends with EOI on
                            its last byte (mock_lines_arm_reply) */
            expect_len = sizeof(resp) - 1u;
            expect_end = GPIB_END_EOI;
        } else if (mode == 1) { /* NL at index 11 */
            expect_len = 12;
        } else if (mode == 2) { /* comma at index 3 */
            expect_len = 4;
        } else { /* semicolon at index 7 */
            expect_len = 8;
        }
        CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
        res = wait_txn(&ctx, t);
        CHECK(res.reason == GPIB_TERM_COMPLETED);
        CHECK(res.end_cause == expect_end);
        CHECK(res.bytes_confirmed == expect_len);
        CHECK(memcmp(rxbuf, resp, expect_len) == 0);
        drain_and_ack(&ctx, b, t);
        CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
        CHECK(gpib_process(&ctx) == GPIB_OK);
        CHECK(gpib_deinit(&ctx) == GPIB_OK);
    }
}

/* E2E-08: TE/LE secondary addressing end to end - the controller
 * addresses the instrument with MLA(6)+SAD(2) and the data flows;
 * MLA(6) alone (without SAD) does NOT reach the device. */
static void e2e_08_secondary_addressing(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_request_t r;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    fill();
    start_line(&ctx, &port);
    /* Submit with a secondary address: should succeed. */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.address.secondary = 2;
    r.address.secondary_present = true;
    r.tx_data = s_tx;
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* The instrument was addressed with MLA(6)+SAD(2) and got the data. */
    CHECK(port.ins.rxlen == 8);
    CHECK(memcmp(port.ins.rx, s_tx, 8) == 0);
    /* Command log: MTA(5), UNL, MLA(6), SAD(2) - 4 prologue bytes. */
    CHECK(port.ins.n_cmdlog == 4);
    CHECK(port.ins.cmdlog[0].cls == GPIB_CMD_MTA);
    CHECK(port.ins.cmdlog[1].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[2].cls == GPIB_CMD_MLA && port.ins.cmdlog[2].arg == 6);
    CHECK(port.ins.cmdlog[3].cls == GPIB_CMD_SECONDARY && port.ins.cmdlog[3].arg == 2);
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-09: Parallel Poll configure + perform + unconfigure. The PPR byte
 * carries the wired-OR of all PPAS devices; with the mock instrument
 * configured (S=1, line=0) and ist=true, bit 0 must be set. */
static void e2e_09_parallel_poll(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    fill();
    start_line(&ctx, &port);
    /* Enable PP1 on the instrument (S=1, line=0, ist=true). */
    port.ins.pp.capable = true;
    port.ins.pp_ist = true;
    /* Configure: PPC + PPE(S=1, line=0) via the API. */
    CHECK(gpib_ppoll_configure(&ctx, 6, 1, 0, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* The instrument's PP is now in PPSS. */
    CHECK(port.ins.pp.state == GPIB_PP_PPSS);
    CHECK(port.ins.pp.sense == true);
    CHECK(port.ins.pp.dio_line == 0);
    drain_and_ack(&ctx, b, t);
    /* Perform the parallel poll: ATN+EOI (IDY). */
    CHECK(gpib_parallel_poll(&ctx, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* The instrument should have entered PPAS during the IDY window
     * (ist=true matches sense=true). The PPR result carries our bit. */
    CHECK(res.stb != 0); /* at least one device responded */
    CHECK((res.stb & 0x01u) != 0u); /* line 0, ist=true -> bit 0 set */
    drain_and_ack(&ctx, b, t);
    /* F02 refinement: after the IDY window the C model walked
     * CPPS -> CAWS -> CACS (rpp released, T9 elapsed). */
    for (int i = 0; i < 4; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(ctx.line.ctrl.main == GPIB_C_CACS);
    /* Table 25 (2.9.3.3): ist != S — the device still enters PPAS but
     * sends its PPR line PASSIVE FALSE; the wired-OR bus reads zero. */
    port.ins.pp_ist = false;
    CHECK(gpib_parallel_poll(&ctx, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.stb == 0); /* nobody drove: passive false on the wire */
    drain_and_ack(&ctx, b, t);
    port.ins.pp_ist = true;
    /* Unconfigure. */
    CHECK(gpib_ppoll_unconfigure(&ctx, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(port.ins.pp.state == GPIB_PP_PPIS); /* disarmed */
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-10: FINDLSTN (17.6). Each candidate is probed by UNL+MLA(addr) then
 * one DAB byte at ATN released: only a listener AT that address accepts it
 * (absent addressees fall back to AIDS and the SDYS stall times out).
 * Address 6 is the mock instrument; 7 and 30 have nobody. */
static void e2e_10_findlstn(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    uint8_t addrs[3] = { 7, 6, 30 };
    bool results[3] = { false, false, false };
    start_line(&ctx, &port);
    CHECK(gpib_findlstn(&ctx, addrs, 3, results, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(results[0] == false); /* 7: nobody there */
    CHECK(results[1] == true);  /* 6: the mock instrument listened */
    CHECK(results[2] == false); /* 30: nobody there */
    /* Command trail: UNL+MLA per probe plus the epilogue UNL. */
    CHECK(port.ins.n_cmdlog == 7);
    CHECK(port.ins.cmdlog[0].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[1].cls == GPIB_CMD_MLA && port.ins.cmdlog[1].arg == 7);
    CHECK(port.ins.cmdlog[2].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[3].cls == GPIB_CMD_MLA && port.ins.cmdlog[3].arg == 6);
    CHECK(port.ins.cmdlog[4].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[5].cls == GPIB_CMD_MLA && port.ins.cmdlog[5].arg == 30);
    CHECK(port.ins.cmdlog[6].cls == GPIB_CMD_UNL);
    /* The only present listener took the dummy probe DAB byte. */
    CHECK(port.ins.rxlen == 1);
    CHECK(port.ins.rx[0] == 0x00u);
    /* Epilogue UNL: the instrument is left unlistened. */
    CHECK(port.ins.addr.lads == false);
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-11: REQUESTCTL (17.5), CIC side. The instrument requests control
 * (STB with RQS); requestctl_pass FINDRQS-locates it and passes control
 * with TCT. Afterwards the core is no longer CIC: controller submits
 * fail NOT_CIC. */
static void e2e_11_requestctl(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    uint8_t addrs[1] = { 6 };
    size_t found = (size_t)-1;
    bool saw_tct = false;
    start_line(&ctx, &port);
    /* The instrument requests control: RQS asserted in its status byte. */
    mock_lines_set_stb(&port, 0x40u, true);
    CHECK(gpib_requestctl_pass(&ctx, addrs, 1, &found, &t, &b) == GPIB_OK);
    CHECK(found == 0); /* the requester is addrs[0] */
    /* The TCT runs asynchronously: wait for its completion. */
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* The instrument observed the poll pair and the TCT handoff: the
     * target is addressed MTA(6) first, then the fixed TCT byte (the
     * command itself carries no address). */
    for (size_t i = 0; i + 1 < port.ins.n_cmdlog; i++) {
        if (port.ins.cmdlog[i].cls == GPIB_CMD_MTA &&
            port.ins.cmdlog[i].arg == 6 &&
            port.ins.cmdlog[i + 1].cls == GPIB_CMD_TCT) {
            saw_tct = true;
        }
    }
    CHECK(saw_tct);
    drain_and_ack(&ctx, b, t);
    /* Control left with the requester: new controller submits -> NOT_CIC. */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_NOT_CIC);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-12: external IFC mid-exchange (audit F01). Another SC pulses IFC
 * while our WRITE is in flight: the transaction terminates with the IFC
 * terminal reason, the interface functions reset, and the instance
 * accepts new work immediately afterwards. */
static void e2e_12_external_ifc(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    int guard = 0;
    fill();
    start_line(&ctx, &port);
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    /* Run INTO the exchange, then pulse IFC from an external bus
     * participant (id 2, the same one the ATN-steal hook uses). */
    while (ctx.action.state != (uint8_t)GPIB_ACTION_ACTIVE && guard++ < 200) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    for (int i = 0; i < 2 && guard++ < 200; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK); /* prologue in flight */
    }
    /* Attach the external bus participant (lazy attach, same id as the
     * ATN-steal hook), then pulse IFC from it. */
    mock_lines_steal_atn(&port, false);
    vbus_assert(port.ins.bus, 2, VBL_IFC);
    for (int i = 0; i < 4; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    vbus_release(port.ins.bus, 2, VBL_IFC);
    for (int i = 0; i < 4; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_IFC);
    CHECK(res.bytes_confirmed < 8u); /* aborted mid-stream, not full */
    /* The instrument-side functions were reset by the pulse too; the
     * core's own SC authority survives (an SC is not unseated by a
     * foreign IFC), so a fresh WRITE completes normally. */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = s_tx;
    r.tx_len = 8;
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    res = wait_txn(&ctx, t);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 8);
    drain_and_ack(&ctx, b, t);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* E2E-13: multi-block READ is ONE RECEIVE (audit F05). Two blocks, one
 * response message: exactly ONE addressing prologue on the wire, the
 * handshake is held (NRFD) between blocks, the stream ends on EOI with
 * the F13 stop state. */
static void e2e_13_read_session(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_request_t r;
    static uint8_t reply[24];
    uint8_t buf1[8];
    uint8_t buf2[16];
    int guard = 0;
    fill();
    for (size_t i = 0; i < sizeof(reply); i++) {
        reply[i] = (uint8_t)(0x30u + i);
    }
    start_line(&ctx, &port);
    mock_lines_arm_reply(&port, reply, sizeof(reply));
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_READ;
    r.address.primary = 6;
    r.rx_data = buf1;
    r.rx_cap = sizeof(buf1);
    CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
    /* First block fills (8 bytes) but the stream is OPEN: backpressure. */
    while (gpib_result_peek(&ctx, b, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == sizeof(buf1));
    /* F05 between blocks: no re-prologue yet, NRFD held on the wire. */
    gpib_process(&ctx);
    CHECK(port.ins.n_cmdlog == 3); /* ONE prologue: UNL, MLA(5), MTA(6) */
    CHECK(vbus_level(port.ins.bus, VBL_NRFD) == true);
    CHECK(vbus_level(port.ins.bus, VBL_ATN) == false);
    CHECK(ctx.line.ctrl.main == GPIB_C_CSBS); /* F02: model in standby */
    /* R6-03: the open stream pins the addressing -- re-addressing this
     * core between blocks must be refused, not silently break the one
     * RECEIVE per response invariant. */
    {
        gpib_address_t probe;
        memset(&probe, 0, sizeof(probe));
        probe.primary = 9;
        CHECK(gpib_set_address(&ctx, probe) == GPIB_BUSY_RESOURCES);
        CHECK(ctx.own_address.primary == 5); /* untouched */
    }
    /* Stage the second block: the session resumes with NO new prologue. */
    CHECK(gpib_submit_block(&ctx, t, buf2, sizeof(buf2), &b2) == GPIB_OK);
    guard = 0;
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI); /* the reply's last byte */
    CHECK(res.bytes_confirmed == sizeof(reply));
    /* Still ONE prologue for the whole response message (F05 core
     * assertion) and the F13 stop state holds after the stream end. */
    CHECK(port.ins.n_cmdlog == 3);
    CHECK(memcmp(buf1, reply, sizeof(buf1)) == 0);
    CHECK(memcmp(buf2, &reply[8], sizeof(buf2)) == 0);
    gpib_process(&ctx);
    CHECK(vbus_level(port.ins.bus, VBL_NRFD) == true);
    CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

void suite_lineexec(void) {
    e2e_01_write();
    e2e_02_read();
    e2e_03_atn_steal();
    e2e_04_shutdown_mid_write();
    e2e_05_send_commands();
    e2e_06_ifc_ren();
    e2e_07_stop_conditions();
    e2e_08_secondary_addressing();
    e2e_09_parallel_poll();
    e2e_10_findlstn();
    e2e_11_requestctl();
    e2e_12_external_ifc();
    e2e_13_read_session();
}
