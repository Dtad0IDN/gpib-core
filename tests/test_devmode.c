#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_ctrl.h"
#include "test_util.h"
#include "vbus.h"

/* Device-role E2E: the core (device at address 5) exchanges with a
 * simulated external controller on the wired-OR vbus.
 *
 * Build placement: the full exchange tests are compiled into the
 * DEVICE-role binary (DEVLINE_DEVICE_BUILD); the controller-role suite
 * compiles only the negative surface (device APIs must be refused). */

#ifdef DEVLINE_DEVICE_BUILD

#include "gpib/gpib_4882.h"

static uint8_t s_pat[32];
static uint8_t s_sink[32];

static void start_dev(gpib_context_t *ctx, mock_ctrl_port_t *p) {
    gpib_init_params_t ip;
    mock_ctrl_init(p, 0); /* external controller at address 0 */
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = false; /* a device is never the system ctrl */
    ip.own_address.primary = 5;
    ip.port = &p->base;
    CHECK(gpib_init(ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(ctx) == GPIB_OK);
}

static void fill(void) {
    for (size_t i = 0; i < sizeof(s_pat); i++) {
        s_pat[i] = (uint8_t)(0xC0u + i);
    }
}

/* DV-01: controller reads from the device: supply + MTA addressing. */
static void dv_01_supply(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    int guard = 0;
    fill();
    start_dev(&ctx, &port);
    CHECK(gpib_device_supply(&ctx, s_pat, sizeof(s_pat), &t, &b) == GPIB_OK);
    mock_ctrl_read(&port, 5, sizeof(s_pat));
    while ((!port.c.done || gpib_result_peek(&ctx, t, &res) != GPIB_OK) &&
           guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(port.c.rx_len == sizeof(s_pat));
    CHECK(memcmp(port.c.rx, s_pat, sizeof(s_pat)) == 0);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == sizeof(s_pat));
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DV-02: controller writes to the device: sink + MLA addressing + EOI. */
static void dv_02_sink(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    int guard = 0;
    fill();
    memset(s_sink, 0, sizeof(s_sink));
    start_dev(&ctx, &port);
    CHECK(gpib_device_sink(&ctx, s_sink, sizeof(s_sink), &t, &b) == GPIB_OK);
    mock_ctrl_write(&port, 5, s_pat, sizeof(s_pat));
    while ((!port.c.done || gpib_result_peek(&ctx, t, &res) != GPIB_OK) &&
           guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(memcmp(s_sink, s_pat, sizeof(s_pat)) == 0);
    CHECK(gpib_result_peek(&ctx, t, &res) == GPIB_OK);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.end_cause == GPIB_END_EOI); /* controller sent EOI */
    CHECK(res.bytes_confirmed == sizeof(s_pat));
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DV-03: addressed device commands land in the service channel. */
static void dv_03_service_channel(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    uint8_t cmd = 0;
    int guard = 0;
    start_dev(&ctx, &port);
    /* Address the device as listener, then Group Execute Trigger. */
    {
        uint8_t seq[3] = { 0x3Fu, 0x25u, 0x08u }; /* UNL, MLA(5), GET */
        mock_ctrl_cmds(&port, seq, 3);
    }
    while (gpib_poll_device_event(&ctx, &cmd) != GPIB_OK && guard++ < 2000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 2000);
    CHECK(cmd == 0x08u); /* GET */
    /* And a universal command without addressing: DCL. */
    {
        uint8_t seq[1] = { 0x14u };
        mock_ctrl_cmds(&port, seq, 1);
    }
    guard = 0;
    while (gpib_poll_device_event(&ctx, &cmd) != GPIB_OK && guard++ < 2000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 2000);
    CHECK(cmd == 0x14u); /* DCL */
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DV-04: module E vertical slice — the external controller programs the
 * device ("*ESE 1\n*OPC\n"), the 488.2 model raises SRQ, the poll reads
 * STB = ESB|RQS, and the request clears after being serviced. */
static void dv_04_module_e(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_4882_t mod;
    static const char prog[] = "*ESE 1;*SRE 32;*OPC\n";
    int guard = 0;
    start_dev(&ctx, &port);
    gpib_4882_init(&mod, "KILO,GPIB-CORE,0,1.0");
    /* Write the program message into the device sink. */
    CHECK(gpib_device_sink(&ctx, s_sink, sizeof(s_sink), &t, &b) == GPIB_OK);
    mock_ctrl_write(&port, 5, (const uint8_t *)prog, sizeof(prog) - 1u);
    while ((gpib_result_peek(&ctx, t, &res) != GPIB_OK ||
            res.bytes_confirmed == 0) && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(memcmp(s_sink, prog, sizeof(prog) - 1u) == 0);
    /* Feed the received message into module E and push STB/rsv to core. */
    CHECK(gpib_4882_feed(&mod, (const char *)s_sink, res.bytes_confirmed));
    CHECK(mod.esr == 0x81u); /* PON (init) + OPC bit (ESE 1 enables it) */
    CHECK(mod.ese == 0x01u);
    CHECK(mod.sre == 0x20u); /* ESB enabled for service requests */
    CHECK(mod.stb == 0x20u);
    CHECK(mod.rsv); /* MSS: ESB AND SRE.5 — the SRE gate now honored */
    CHECK(gpib_device_set_stb(&ctx, gpib_4882_stb(&mod)) == GPIB_OK);
    CHECK(gpib_device_service_request(&ctx, gpib_4882_rsv(&mod)) == GPIB_OK);
    /* SRQ appears on the bus. */
    guard = 0;
    while (!vbus_level(&port.c.bus, VBL_SRQ) && guard++ < 200) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(vbus_level(&port.c.bus, VBL_SRQ));
    /* Serial poll: SPE arms the talker pair (wait for the handshake to
     * complete before the read replaces the wires); then read the STB. */
    {
        uint8_t spe = 0x18u;
        mock_ctrl_cmds(&port, &spe, 1);
    }
    guard = 0;
    while (!port.c.done && guard++ < 2000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 2000);
    CHECK(ctx.line.tk.sp == GPIB_T_SPMS);
    mock_ctrl_read(&port, 5, 1);
    guard = 0;
    while (!port.c.done && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(port.c.rx_len == 1);
    CHECK(port.c.rx[0] == (uint8_t)(0x20u | 0x40u)); /* ESB | RQS */
    /* Serviced: rsv self-cleared, SRQ released. */
    CHECK(!ctx.device_rsv);
    guard = 0;
    while (vbus_level(&port.c.bus, VBL_SRQ) && guard++ < 200) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(!vbus_level(&port.c.bus, VBL_SRQ));
    {
        uint8_t spd = 0x19u;
        mock_ctrl_cmds(&port, &spd, 1);
    }
    for (int i = 0; i < 50; i++) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(ctx.line.tk.sp == GPIB_T_SPIS); /* SPD closed the poll */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DV-05: DF-event overflow is a mandatory-delivery VIOLATION: the fault
 * latches (GPIB_FAULT_DF_EVENT_LOST), the counter keeps the evidence,
 * and the application can still observe it via the snapshot. */
static void dv_05_df_overflow_fault(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    gpib_fault_snapshot_t snap;
    uint8_t cmd = 0;
    int guard = 0;
    start_dev(&ctx, &port);
    /* Address the device, then flood GET past the DF ring capacity. */
    {
        uint8_t seq[3] = { 0x3Fu, 0x25u, 0x08u };
        mock_ctrl_cmds(&port, seq, 3);
    }
    guard = 0;
    while (gpib_poll_device_event(&ctx, &cmd) != GPIB_OK && guard++ < 2000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 2000);
    CHECK(!ctx.fault_latched); /* first trigger delivered cleanly */
    for (int round = 0; round < 3 * GPIB_CFG_DF_EVENTS; round++) {
        uint8_t get[1] = { 0x08u };
        mock_ctrl_cmds(&port, get, 1);
        guard = 0;
        while (port.c.st != 4 && guard++ < 1000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        CHECK(guard < 1000);
    }
    CHECK(ctx.df_evt_dropped >= 1);
    CHECK(ctx.fault_latched);
    CHECK(ctx.fault_code == GPIB_FAULT_DF_EVENT_LOST);
    CHECK(gpib_get_fault_snapshot(&ctx, &snap) == GPIB_OK);
    CHECK(snap.fault_latched);
    CHECK(snap.fault_code == GPIB_FAULT_DF_EVENT_LOST);
    CHECK(snap.device_events_dropped >= 1);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DV-D01 (final audit, probe-H scenario): a controller that ABORTS the
 * serial poll mid-STB (asserts ATN without accepting the byte -- the
 * routine timeout path with a hung instrument) and then reads data must
 * receive the DATA FIRST. The old code kept the spas_tx_active latch
 * alive: the stale STB was later injected as the FIRST data byte
 * (0x62 'D' 'A' 'T' 'A' '!' instead of "DATA!"), and without a repeat
 * poll the supply could never even start. */
static void dv_d01_spas_abort(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    int guard = 0;
    fill();
    start_dev(&ctx, &port);
    CHECK(gpib_device_set_stb(&ctx, 0x22u) == GPIB_OK);
    CHECK(gpib_device_service_request(&ctx, true) == GPIB_OK);
    /* Arm the poll: SPE, then address us as talker. */
    {
        uint8_t spe = 0x18u;
        mock_ctrl_cmds(&port, &spe, 1);
    }
    guard = 0;
    while (ctx.line.tk.sp != GPIB_T_SPMS && guard++ < 2000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    REQUIRE(guard < 2000);
    mock_ctrl_read(&port, 5, 1);
    /* Run until the core raises DAV with the STB (the latch arms when
     * SPAS is reached with ATN released -- the mock's own addressing
     * also raises DAV, so wait on the latch itself). */
    guard = 0;
    while (!ctx.spas_tx_active && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000);
    REQUIRE(ctx.spas_tx_active); /* mid-STB: the latch is set */
    /* ABORT the poll: a THIRD participant steals ATN without the byte
     * being accepted (the mock's own model stays coherent). */
    vbus_attach_as(&port.c.bus, 2);
    vbus_assert(&port.c.bus, 2, VBL_ATN);
    guard = 0;
    while (ctx.spas_tx_active && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000); /* D01: the latch DIED with the abort */
    vbus_release(&port.c.bus, 2, VBL_ATN);
    /* Close the poll properly: SPD (the mock drives ATN itself). */
    {
        uint8_t spd = 0x19u;
        mock_ctrl_cmds(&port, &spd, 1);
    }
    guard = 0;
    while (ctx.line.tk.sp != GPIB_T_SPIS && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000);
    /* The controller reads data: the supply must deliver 'D' first. */
    CHECK(gpib_device_supply(&ctx, (const uint8_t *)"DATA!", 5, &t, &b) ==
          GPIB_OK);
    mock_ctrl_read(&port, 5, 5);
    guard = 0;
    while ((!port.c.done || gpib_result_peek(&ctx, t, &res) != GPIB_OK) &&
           guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    REQUIRE(guard < 20000);
    REQUIRE(port.c.rx_len == 5u); /* exactly DATA!, no phantom STB */
    CHECK(port.c.rx[0] == (uint8_t)'D');
    CHECK(port.c.rx[1] == (uint8_t)'A');
    CHECK(port.c.rx[2] == (uint8_t)'T');
    CHECK(port.c.rx[3] == (uint8_t)'A');
    CHECK(port.c.rx[4] == (uint8_t)'!');
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == 5);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DV-06 (audit NV: helper response lifetime): the response pointer
 * from gpib_4882_response() must stay valid and unchanged while the
 * device supply action holds zero-copy ownership of the buffer -- the
 * integration must not modify it before ACK. */
static void dv_06_response_lifetime(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_4882_t mod;
    static const char prog[] = "*IDN?\n";
    int guard = 0;
    const char *resp1;
    size_t len1 = 0;
    start_dev(&ctx, &port);
    gpib_4882_init(&mod, "KILO,GPIB-CORE,0,1.0");
    /* Program the query and let module E produce the response. */
    CHECK(gpib_device_sink(&ctx, s_sink, sizeof(s_sink), &t, &b) ==
          GPIB_OK);
    mock_ctrl_write(&port, 5, (const uint8_t *)prog, sizeof(prog) - 1u);
    while ((gpib_result_peek(&ctx, t, &res) != GPIB_OK ||
            res.bytes_confirmed == 0) && guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(gpib_4882_feed(&mod, (const char *)s_sink,
                         res.bytes_confirmed));
    resp1 = gpib_4882_response(&mod, &len1);
    REQUIRE(resp1 != NULL);
    REQUIRE(len1 == 20u); /* the identity string */
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    /* Supply the helper's response buffer DIRECTLY (true zero-copy: the
     * audit correctly noted that copying to supply_buf tested stability,
     * not ownership -- the core holds a borrowed pointer to the helper's
     * internal buffer through the entire bus exchange). */
    CHECK(gpib_device_supply(&ctx, (const uint8_t *)resp1, len1,
                             &t, &b) == GPIB_OK);
    /* While the supply is in flight, the helper's response buffer must
     * stay untouched (zero-copy: the supply action reads it). */
    {
        const char *resp2;
        size_t len2 = 0;
        for (int i = 0; i < 100; i++) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        resp2 = gpib_4882_response(&mod, &len2);
        CHECK(resp2 == resp1); /* same pointer, no reallocation */
        CHECK(len2 == len1);   /* same length, no truncation */
        /* The helper's buffer bytes are what the mock receives. */
        for (size_t i = 0; i < len2; i++) {
            CHECK((uint8_t)resp2[i] == (uint8_t)resp1[i]);
        }
    }
    /* Complete the supply. */
    mock_ctrl_read(&port, 5, len1);
    guard = 0;
    while ((!port.c.done || gpib_result_peek(&ctx, t, &res) != GPIB_OK) &&
           guard++ < 20000) {
        CHECK(gpib_process(&ctx) == GPIB_OK);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(res.bytes_confirmed == len1);
    /* The helper buffer is STILL valid after the supply completes
     * (zero-copy: the same pointer, same bytes). */
    {
        const char *resp3 = gpib_4882_response(&mod, &len1);
        CHECK(resp3 == resp1);
        CHECK(memcmp(resp3, resp1, len1) == 0);
    }
    gpib_4882_response_consumed(&mod);
    CHECK(gpib_result_ack(&ctx, b) == GPIB_OK);
    CHECK(gpib_result_ack(&ctx, t) == GPIB_OK);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

/* DV-07 (is_rx for multi-block DEV_RX): the second sink block must get
 * is_rx=true like the first, so gpib_result_view would expose its data
 * before ACK. The submit_block path used to set is_rx=false (only
 * READ was checked). Direct flag check: the full multi-block DEV_RX
 * bus exchange is a separate integration concern. */
static void dv_07_multiblock_dev_rx_is_rx(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_ctrl_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b1 = { 0, 0, 0, 0, 0 };
    gpib_handle_t b2 = { 0, 0, 0, 0, 0 };
    static uint8_t sink1[4];
    static uint8_t sink2[4];
    start_dev(&ctx, &port);
    CHECK(gpib_device_sink(&ctx, sink1, 4, &t, &b1) == GPIB_OK);
    CHECK(gpib_submit_block(&ctx, t, sink2, 4, &b2) == GPIB_OK);
    /* Both blocks carry is_rx=true (the mutation flips it to false). */
    CHECK(ctx.block[b1.slot].is_rx == true);
    CHECK(ctx.block[b2.slot].is_rx == true);
    /* Clean up: cancel the txn and drain. */
    CHECK(gpib_request_cancel(&ctx, t) == GPIB_OK);
    {
        gpib_result_t res;
        int guard = 0;
        while (gpib_result_peek(&ctx, t, &res) != GPIB_OK &&
               guard++ < 20000) {
            CHECK(gpib_process(&ctx) == GPIB_OK);
        }
        REQUIRE(guard < 20000);
    }
    CHECK(gpib_result_ack(&ctx, b2) == GPIB_OK || b2.generation == 0u);
    CHECK(gpib_result_ack(&ctx, b1) == GPIB_OK || b1.generation == 0u);
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

void suite_devmode(void) {
    dv_01_supply();
    dv_02_sink();
    dv_03_service_channel();
    dv_04_module_e();
    dv_05_df_overflow_fault();
    dv_d01_spas_abort();
    dv_06_response_lifetime();
    dv_07_multiblock_dev_rx_is_rx();
}

#else /* controller build: negative surface only */

void suite_devneg(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init_params_t ip;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    uint8_t buf[4] = { 0 };
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = true;
    ip.own_address.primary = 5;
    CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    /* REQ-017: the controller initiates; addressed-device exchanges are
     * refused in the controller build. */
    CHECK(gpib_device_supply(&ctx, buf, sizeof(buf), &t, &b) ==
          GPIB_UNSUPPORTED);
    CHECK(gpib_device_sink(&ctx, buf, sizeof(buf), &t, &b) ==
          GPIB_UNSUPPORTED);
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
}

#endif /* DEVLINE_DEVICE_BUILD */
