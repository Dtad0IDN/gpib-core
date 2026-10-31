#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_bus.h"
#include "test_util.h"

/* Core-to-core: TWO real core instances (controller role, addresses 1 and
 * 2) on one shared wired-OR bus. TC-01 verifies the full TCT control
 * transfer. Per the multi-controller model (488.2 16.2.8) the INITIATOR
 * must hold CIC: A is the system controller (it took charge via IFC at
 * power-on), B is a plain controller that receives TCT. The negative
 * surface (a non-SC controller in CIDS cannot initiate) is TC-02. */
static void start_core(gpib_context_t *ctx, bus_port_t *port,
    shared_bus_t *bus, int id, uint8_t own_addr, bool sys_ctrl) {
    gpib_init_params_t ip;
    bus_port_init(port, bus, id);
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = sys_ctrl;
    ip.own_address.primary = own_addr;
    ip.port = &port->base;
    CHECK(gpib_init(ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
}

static void tc_01_pass_control(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx_a); /* two live core instances */
    GPIB_CONTEXT_STORAGE_DEFINE(ctx_b);
    shared_bus_t bus;
    bus_port_t port_a;
    bus_port_t port_b;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    int guard = 0;
    shared_bus_init(&bus);
    start_core(&ctx_a, &port_a, &bus, 0, 1, true);  /* SC: took CIC via IFC */
    start_core(&ctx_b, &port_b, &bus, 1, 2, false); /* plain controller */
    CHECK(gpib_pass_control(&ctx_a, 2, &t, &b) == GPIB_OK);
    while ((gpib_result_peek(&ctx_a, t, &res) != GPIB_OK ||
            ctx_b.line.ctrl.main == GPIB_C_CIDS ||
            ctx_b.line.ctrl.main == GPIB_C_CADS) && guard++ < 20000) {
        CHECK(gpib_process(&ctx_a) == GPIB_OK);
        CHECK(gpib_process(&ctx_b) == GPIB_OK);
    }
    CHECK(guard < 20000);
    /* Sender: action completed, controller groups released CIC. */
    CHECK(gpib_result_peek(&ctx_a, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(ctx_a.line.ctrl.main == GPIB_C_CIDS);
    /* Receiver: became CIC; one extra owner slice lets the idle drive
     * assert the ATN hold (CACS entry and the line drive are one slice
     * apart in the serialized model). */
    CHECK(ctx_b.line.ctrl.main == GPIB_C_CACS);
    for (int i = 0; i < 10; i++) {
        CHECK(gpib_process(&ctx_b) == GPIB_OK);
    }
    CHECK(vbus_level(&bus.bus, VBL_ATN));
    /* Receiver saw MTA(2) then TCT (service channel got TCT). */
    {
        uint8_t cmd = 0;
        CHECK(gpib_poll_device_event(&ctx_b, &cmd) == GPIB_OK);
        CHECK(cmd == 0x09u);
    }
    CHECK(gpib_result_ack(&ctx_a, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx_a, t) == GPIB_OK);

    /* Reviewer scenario: A (SC) held CIC and passed it away — a new
     * initiating submission MUST be refused (GPIB_NOT_CIC) even though
     * A is the System Controller: SC grants IFC/REN rights, not
     * permanent CIC. */
    {
        uint8_t dcl[1] = { 0x14u };
        gpib_handle_t t2 = { 0, 0, 0, 0, 0 };
        gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
        CHECK(ctx_a.line.ctrl.main == GPIB_C_CIDS);
        CHECK(gpib_send_commands(&ctx_a, dcl, 1, &t2, &b2) ==
              GPIB_NOT_CIC);
        /* The legitimate re-takeover: SEND IFC re-seizes the bus. */
        CHECK(gpib_interface_clear(&ctx_a) == GPIB_OK);
        guard = 0;
        while (ctx_a.line.ctrl.main != GPIB_C_CACS && guard++ < 2000) {
            CHECK(gpib_process(&ctx_a) == GPIB_OK);
            CHECK(gpib_process(&ctx_b) == GPIB_OK);
        }
        CHECK(guard < 2000);
        CHECK(ctx_a.line.ctrl.main == GPIB_C_CACS);
        /* After re-seizing, the DCL submission is accepted again. */
        CHECK(gpib_send_commands(&ctx_a, dcl, 1, &t2, &b2) == GPIB_OK);
        guard = 0;
        while (gpib_result_peek(&ctx_a, t2, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx_a) == GPIB_OK);
            CHECK(gpib_process(&ctx_b) == GPIB_OK);
        }
        CHECK(guard < 20000);
        CHECK(gpib_result_peek(&ctx_a, t2, &res) == GPIB_OK);
        CHECK(res.reason == GPIB_TERM_COMPLETED);
        CHECK(gpib_result_ack(&ctx_a, b2) == GPIB_OK);
        CHECK(gpib_result_ack(&ctx_a, t2) == GPIB_OK);
    }

    /* Clean teardown of both instances (B holds ATN: shutdown first). */
    CHECK(gpib_request_shutdown(&ctx_a) == GPIB_OK);
    CHECK(gpib_process(&ctx_a) == GPIB_OK);
    CHECK(gpib_deinit(&ctx_a) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx_b) == GPIB_OK);
    CHECK(gpib_process(&ctx_b) == GPIB_OK);
    CHECK(gpib_deinit(&ctx_b) == GPIB_OK);
}

/* TC-02: multi-controller arbitration — a non-SC controller in CIDS is
 * REFUSED when trying to initiate bus operations (GPIB_NOT_CIC); after
 * the legitimate TCT transfer it may initiate. */
static void tc_02_not_cic_refused(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx_c);
    shared_bus_t bus;
    bus_port_t port_c;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    uint8_t dcl[1] = { 0x14u };
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_WRITE;
    r.address.primary = 6;
    r.tx_data = dcl;
    r.tx_len = 1;
    shared_bus_init(&bus);
    start_core(&ctx_c, &port_c, &bus, 0, 3, false); /* NOT the SC */
    CHECK(ctx_c.line.ctrl.main == GPIB_C_CIDS);
    /* Every initiating surface refuses: submit, send_commands, TCT,
     * ALLSPOLL, FINDRQS, RESET. */
    CHECK(gpib_submit(&ctx_c, &r, &t, &b) == GPIB_NOT_CIC);
    CHECK(gpib_send_commands(&ctx_c, dcl, 1, &t, &b) == GPIB_NOT_CIC);
    CHECK(gpib_pass_control(&ctx_c, 2, &t, &b) == GPIB_NOT_CIC);
    CHECK(gpib_serial_poll(&ctx_c, 6, &t, &b) == GPIB_NOT_CIC);
    {
        static const uint8_t addrs[1] = { 6u };
        uint8_t results[1];
        size_t found;
        CHECK(gpib_allspoll(&ctx_c, addrs, 1, results, &t, &b) ==
              GPIB_NOT_CIC);
        CHECK(gpib_findrqs(&ctx_c, addrs, 1, results, &found, &t, &b) ==
              GPIB_NOT_CIC);
        /* RESET needs C2 (SC) on top of CIC: a non-SC instance gets a
         * distinct, documented rejection. */
        CHECK(gpib_system_reset(&ctx_c, addrs, 1, &t, &b) ==
              GPIB_UNSUPPORTED);
    }
    /* No side effects: nothing was armed, nothing on the bus. */
    CHECK(!ctx_c.ifc_request);
    CHECK(!ctx_c.ren_request);
    CHECK(ctx_c.action.state == (uint8_t)GPIB_ACTION_EMPTY);
    CHECK(gpib_request_shutdown(&ctx_c) == GPIB_OK);
    CHECK(gpib_process(&ctx_c) == GPIB_OK);
    CHECK(gpib_deinit(&ctx_c) == GPIB_OK);
}

/* TC-04 (audit A08 / F04): the PP1 responder is a REAL core instance,
 * not the test mock. B (address 2) arms its device PP1 capability; A
 * configures it via PPC+PPE, polls it, flips ist, and disarms via PPU. */
static void tc_04_parallel_poll_pp1(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx_a);
    GPIB_CONTEXT_STORAGE_DEFINE(ctx_b);
    shared_bus_t bus;
    bus_port_t port_a;
    bus_port_t port_b;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_init_params_t ip;
    int guard = 0;
    shared_bus_init(&bus);
    /* A: the SC controller at 1. */
    bus_port_init(&port_a, &bus, 0);
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = true;
    ip.own_address.primary = 1;
    ip.port = &port_a.base;
    CHECK(gpib_init(&ctx_a, &ip) == GPIB_OK);
    CHECK(gpib_start(&ctx_a) == GPIB_OK);
    /* B: a PP1-capable core at 2 (device functions run in any role). */
    bus_port_init(&port_b, &bus, 1);
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = false;
    ip.own_address.primary = 2;
    ip.port = &port_b.base;
    ip.pp_capable = true;
    CHECK(gpib_init(&ctx_b, &ip) == GPIB_OK);
    CHECK(gpib_start(&ctx_b) == GPIB_OK);
    CHECK(gpib_pp_set_ist(&ctx_b, true) == GPIB_OK);

    /* Configure B remotely: PPC + PPE(S=1, line=3) via A. */
    CHECK(gpib_ppoll_configure(&ctx_a, 2, 1, 3, &t, &b) == GPIB_OK);
    guard = 0;
    while ((gpib_result_peek(&ctx_a, t, &res) != GPIB_OK ||
            ctx_b.pp.state != GPIB_PP_PPSS) && guard++ < 20000) {
        CHECK(gpib_process(&ctx_a) == GPIB_OK);
        CHECK(gpib_process(&ctx_b) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(ctx_b.pp.sense == true);
    CHECK(ctx_b.pp.dio_line == 3);
    CHECK(gpib_result_ack(&ctx_a, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx_a, t) == GPIB_OK);

    /* Poll 1: ist == S -> exactly PPR4 on the wire. */
    CHECK(gpib_parallel_poll(&ctx_a, &t, &b) == GPIB_OK);
    guard = 0;
    while (gpib_result_peek(&ctx_a, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx_a) == GPIB_OK);
        CHECK(gpib_process(&ctx_b) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.stb == (1u << 3));
    CHECK(gpib_result_ack(&ctx_a, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx_a, t) == GPIB_OK);

    /* Poll 2: ist != S -> PASSIVE FALSE: the wired-OR bus reads zero. */
    CHECK(gpib_pp_set_ist(&ctx_b, false) == GPIB_OK);
    CHECK(gpib_parallel_poll(&ctx_a, &t, &b) == GPIB_OK);
    guard = 0;
    while (gpib_result_peek(&ctx_a, t, &res) != GPIB_OK && guard++ < 20000) {
        CHECK(gpib_process(&ctx_a) == GPIB_OK);
        CHECK(gpib_process(&ctx_b) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.stb == 0);
    CHECK(gpib_result_ack(&ctx_a, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx_a, t) == GPIB_OK);
    CHECK(gpib_pp_set_ist(&ctx_b, true) == GPIB_OK);

    /* Unconfigure via PPU: B is back to PPIS. */
    CHECK(gpib_ppoll_unconfigure(&ctx_a, &t, &b) == GPIB_OK);
    guard = 0;
    while ((gpib_result_peek(&ctx_a, t, &res) != GPIB_OK ||
            ctx_b.pp.state != GPIB_PP_PPIS) && guard++ < 20000) {
        CHECK(gpib_process(&ctx_a) == GPIB_OK);
        CHECK(gpib_process(&ctx_b) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(ctx_b.pp.state == GPIB_PP_PPIS);
    CHECK(gpib_result_ack(&ctx_a, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx_a, t) == GPIB_OK);

    CHECK(gpib_request_shutdown(&ctx_a) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx_b) == GPIB_OK);
    CHECK(gpib_process(&ctx_a) == GPIB_OK);
    CHECK(gpib_process(&ctx_b) == GPIB_OK);
    CHECK(gpib_deinit(&ctx_a) == GPIB_OK);
    CHECK(gpib_deinit(&ctx_b) == GPIB_OK);
}

void suite_dualcore(void) {
    tc_01_pass_control();
    tc_02_not_cic_refused();
    tc_04_parallel_poll_pp1();
}
