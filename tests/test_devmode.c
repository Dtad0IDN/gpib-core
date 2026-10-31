#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"
#include "mock_ctrl.h"
#include "test_util.h"

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
    CHECK(mod.esr == 0x01u); /* OPC bit (ESE 1 enables it) */
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

void suite_devmode(void) {
    dv_01_supply();
    dv_02_sink();
    dv_03_service_channel();
    dv_04_module_e();
    dv_05_df_overflow_fault();
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
