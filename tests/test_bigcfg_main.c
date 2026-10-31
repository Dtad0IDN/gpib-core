/* Verification of the reviewer's exact scenario: with MAX_LISTENERS=11
 * and MAX_CMDS=13 the previous build silently truncated (13-byte SEND
 * COMMAND -> 12 executed, 11-listener WRITE_MANY -> 10 addressed).
 * With the config-tied buffers this must either work fully or reject. */
#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_storage.h"
#include "mock_lines.h"
#include "test_util.h"

unsigned g_checks, g_failures;

int main(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    mock_line_port_t port;
    gpib_handle_t t = { 0, 0, 0, 0, 0 };
    gpib_handle_t b = { 0, 0, 0, 0, 0 };
    gpib_result_t res;
    gpib_init_params_t ip;
    uint8_t cmds[13];
    int guard = 0;
    for (int i = 0; i < 13; i++) {
        cmds[i] = 0x14u; /* DCL x13 */
    }
    mock_lines_init(&port);
    memset(&ip, 0, sizeof(ip));
    ip.system_controller = true;
    ip.own_address.primary = 5;
    ip.port = &port.base;
    CHECK(gpib_init(&ctx, &ip) == GPIB_OK);
    CHECK(gpib_start(&ctx) == GPIB_OK);
    /* 13 ATN bytes with MAX_CMDS=13: accepted, all 13 handshaken. */
    CHECK(gpib_send_commands(&ctx, cmds, 13, &t, &b) == GPIB_OK);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 20000) {
        gpib_process(&ctx);
    }
    CHECK(guard < 20000);
    CHECK(res.reason == GPIB_TERM_COMPLETED);
    CHECK(port.ins.n_cmdlog == 13); /* no truncation */
    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);
    /* 11 listeners with MAX_LISTENERS=11: the prologue is
     * MTA+UNL+11*MLA = 13 bytes; cmds buffer covers 2+11 = 13. */
    {
        gpib_address_t listeners[11];
        gpib_request_t r;
        for (int i = 0; i < 11; i++) {
            listeners[i].primary = (uint8_t)(i == 0 ? 6 : (10 + i));
            listeners[i].secondary = 0;
            listeners[i].secondary_present = false;
        }
        memset(&r, 0, sizeof(r));
        r.kind = GPIB_REQ_WRITE_MANY;
        r.listeners = listeners;
        r.listener_count = 11;
        r.tx_data = cmds;
        r.tx_len = 4;
        CHECK(gpib_submit(&ctx, &r, &t, &b) == GPIB_OK);
        guard = 0;
        while (gpib_result_peek(&ctx, t, &res) != GPIB_OK && guard++ < 30000) {
            gpib_process(&ctx);
        }
        printf("wm guard=%d reason=%d cmdlog=%u rx=%u submit_ok`n", guard,
            (int)res.reason, (unsigned)port.ins.n_cmdlog,
            (unsigned)port.ins.rxlen);
        CHECK(guard < 30000);
        CHECK(res.reason == GPIB_TERM_COMPLETED);
        /* Instrument 6 heard the data; the command log grew by the
         * full prologue (MTA, UNL, 11 MLAs) + prior 13 DCLs. */
        CHECK(port.ins.rxlen == 4);
        gpib_result_ack(&ctx, b);
        gpib_result_ack(&ctx, t);
    }
    /* 14 bytes with MAX_CMDS=13: rejected before side effects. */
    {
        uint8_t cmds14[14];
        memset(cmds14, 0x14, sizeof(cmds14));
        CHECK(gpib_send_commands(&ctx, cmds14, 14, &t, &b) ==
              GPIB_INVALID_ARGUMENT);
    }
    CHECK(gpib_request_shutdown(&ctx) == GPIB_OK);
    CHECK(gpib_process(&ctx) == GPIB_OK);
    CHECK(gpib_deinit(&ctx) == GPIB_OK);
    printf("bigcfg: checks=%u failures=%u\n", g_checks, g_failures);
    return g_failures != 0;
}
