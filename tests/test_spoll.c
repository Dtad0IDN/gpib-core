#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_lines.h"
#include "test_util.h"

/* End-to-end serial poll: SRQ detection, SPE/MTA/STB/SPD/UNT sequence,
 * RQS bit semantics, request serviced on the instrument side. */

static void start_line(gpib_context_t *ctx, mock_line_port_t *p) {
    gpib_init_params_t ip;
    mock_lines_init(p);
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = true;
    ip.own_address.primary = 5;
    ip.port = &p->base;
    CHECK(gpib_init(ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
}

/* SP-01: instrument requests service; the core sees SRQ; serial poll
 * returns STB|RQS; the request clears and SRQ releases. */
static void sp_01_full_cycle(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_status_info_t info;
    int guard = 0;
    start_line(&ctx, &port);
    mock_lines_set_stb(&port, 0x02, true);
    /* Idle observation: wait for the power-on IFC pulse (T8 ~100 µs) to
     * complete, then SRQ must show up in the status. */
    guard = 0;
    while (!vbus_level(port.ins.bus, VBL_SRQ) && guard++ < 2000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 2000);
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(info.srq_active);
    CHECK(vbus_level(port.ins.bus, VBL_SRQ));
    CHECK(gpib_serial_poll(&ctx, 6, &t, &b) == GPIB_OK);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 1);
    CHECK(res.stb == (uint8_t)(0x02u | 0x40u)); /* STB with RQS */
    /* Command sequence observed by the instrument (16.2.18: UNL,
     * MLA(controller), SPE, MTA(device) ... SPD, UNT). */
    CHECK(port.ins.n_cmdlog == 6);
    CHECK(port.ins.cmdlog[0].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[1].cls == GPIB_CMD_MLA && port.ins.cmdlog[1].arg == 5);
    CHECK(port.ins.cmdlog[2].cls == GPIB_CMD_SPE);
    CHECK(port.ins.cmdlog[3].cls == GPIB_CMD_MTA && port.ins.cmdlog[3].arg == 6);
    CHECK(port.ins.cmdlog[4].cls == GPIB_CMD_SPD);
    CHECK(port.ins.cmdlog[5].cls == GPIB_CMD_UNT);
    /* Serviced: SR back to idle, SRQ released. */
    CHECK(port.ins.sr.state == GPIB_SR_NPRS);
    for (int i = 0; i < 10; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(gpib_query_status(&ctx, &info) == GPIB_OK);
    CHECK(!info.srq_active);
    CHECK(!vbus_level(port.ins.bus, VBL_SRQ));
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* SP-02: poll without a pending request: plain STB, no RQS bit. */
static void sp_02_no_request(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    int guard = 0;
    start_line(&ctx, &port);
    mock_lines_set_stb(&port, 0x21, false);
    CHECK(gpib_serial_poll(&ctx, 6, &t, &b) == GPIB_OK);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.stb == 0x21); /* no RQS: nobody was requesting */
    CHECK(port.ins.talker.sp == GPIB_T_SPIS); /* SPD restored */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* SP-03: ALLSPOLL (17.3, mandatory) — one armed SPE pass over an address
 * list; each device answers with its own STB; SPD+UNT close the pass. */
static void sp_03_allspoll(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port; /* two instruments on the SAME bus */
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    static const uint8_t addrs[2] = { 6u, 7u };
    uint8_t results[2] = { 0xEEu, 0xEEu };
    int guard = 0;
    mock_lines_init(&port);
    mock_lines_add_second(&port, 7);
    /* Both instruments request service with distinct STBs. */
    mock_lines_set_stb(&port, 0x02, true);
    mock_lines_set_stb_second(&port, 0x41, true);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &port.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    CHECK(gpib_allspoll(&ctx, addrs, 2, results, &t, &b) == GPIB_OK);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 40000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 40000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 2);
    CHECK(results[0] == (uint8_t)(0x02u | 0x40u)); /* STB|RQS of addr 6 */
    CHECK(results[1] == (uint8_t)((0x41u & 0xBFu) | 0x40u)); /* addr 7 */
    /* Both serviced: rsv cleared, SRQ released. */
    CHECK(!port.ins.rsv);
    CHECK(!port.ins2.rsv);
    CHECK(!vbus_level(port.ins.bus, VBL_SRQ));
    /* Command sequence: UNL, MLA(5), SPE, [MTA6, read, MTA7, read], SPD,
     * UNT — checked on the first instrument's log. */
    CHECK(port.ins.n_cmdlog == 7);
    CHECK(port.ins.cmdlog[0].cls == GPIB_CMD_UNL);
    CHECK(port.ins.cmdlog[1].cls == GPIB_CMD_MLA);
    CHECK(port.ins.cmdlog[2].cls == GPIB_CMD_SPE);
    CHECK(port.ins.cmdlog[3].cls == GPIB_CMD_MTA && port.ins.cmdlog[3].arg == 6);
    CHECK(port.ins.cmdlog[4].cls == GPIB_CMD_MTA && port.ins.cmdlog[4].arg == 7);
    CHECK(port.ins.cmdlog[5].cls == GPIB_CMD_SPD);
    CHECK(port.ins.cmdlog[6].cls == GPIB_CMD_UNT);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* SP-04: FINDRQS (17.2, optional) — poll in list order until RQS; the
 * second device requests, the first does not: the answer is index 1. */
static void sp_04_findrqs(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    static const uint8_t addrs[2] = { 6u, 7u };
    uint8_t results[2] = { 0xEEu, 0xEEu };
    size_t found = 12345;
    int guard = 0;
    mock_lines_init(&port);
    mock_lines_add_second(&port, 7);
    mock_lines_set_stb(&port, 0x00, false);      /* addr 6: no request */
    mock_lines_set_stb_second(&port, 0x55, true); /* addr 7: requesting */
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &port.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    CHECK(gpib_findrqs(&ctx, addrs, 2, results, &found, &t, &b) == GPIB_OK);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 40000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 40000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(found == 1); /* index of the requesting device */
    CHECK((results[1] & 0x40u) != 0u); /* RQS in its STB */
    CHECK(results[1] == (uint8_t)((0x55u & 0xBFu) | 0x40u));
    /* The requesting device was serviced; the non-requesting one polled
     * without RQS. */
    CHECK((results[0] & 0x40u) == 0u);
    CHECK(!port.ins2.rsv);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* SP-05: RESET protocol (17.1, mandatory) — REN+IFC+DCL+*RST to each
 * address; the instruments observe DCL and the *RST<NL> payload. */
static void sp_05_reset(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    static const uint8_t addrs[2] = { 6u, 7u };
    int guard = 0;
    mock_lines_init(&port);
    mock_lines_add_second(&port, 7);
    {
        gpib_init_params_t ip;
        memset(&ip, 0, sizeof(ip));
        ip.system_controller = true;
        ip.own_address.primary = 5;
        ip.port = &port.base;
        CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
        CHECK(gpib_start(&ctx) == GPIB_OK);
    }
    CHECK(gpib_system_reset(&ctx, addrs, 2, &t, &b) == GPIB_OK);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 60000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 60000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    /* Level 2: the universal DCL reached both devices. */
    CHECK(port.ins.dc_clears >= 1);
    CHECK(port.ins2.dc_clears >= 1);
    /* Level 3: each device received the exact *RST<NL> payload. */
    CHECK(port.ins.rxlen == 5);
    CHECK(memcmp(port.ins.rx, "*RST\n", 5) == 0);
    CHECK(port.ins2.rxlen == 5);
    CHECK(memcmp(port.ins2.rx, "*RST\n", 5) == 0);
    /* Level 1: REN asserted (persistent), IFC pulsed and consumed. */
    CHECK(vbus_level(port.ins.bus, VBL_REN));
    CHECK(!ctx.ifc_request);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

void suite_spoll(void) {
    sp_01_full_cycle();
    sp_02_no_request();
    sp_03_allspoll();
    sp_04_findrqs();
    sp_05_reset();
}
