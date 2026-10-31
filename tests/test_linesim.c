#include <string.h>

#include "gpib/gpib_proto.h"
#include "line_sim.h"
#include "test_util.h"

/* Line-level integration: SH over wired-OR vbus lines against one or more
 * AH peers, the Annex B byte cycle end to end. */

static uint8_t s_pattern[16];
static uint8_t s_rx0[16];
static uint8_t s_rx1[16];

static void fill_pattern(void) {
    for (size_t i = 0; i < sizeof(s_pattern); i++) {
        s_pattern[i] = (uint8_t)(0x30u + i);
    }
}

/* LS-01: one source, one acceptor, full 16-byte transfer with integrity. */
static void ls_01_single_acceptor(void) {
    line_bus_t lb;
    sh_peer_t src;
    ah_peer_t acc[1];
    fill_pattern();
    memset(s_rx0, 0, sizeof(s_rx0));
    line_bus_init(&lb);
    sh_peer_init(&src, 0, s_pattern, sizeof(s_pattern), 2u, true);
    ah_peer_init(&acc[0], 1, s_rx0, sizeof(s_rx0), 1u, 0u, true);
    for (int round = 0; round < 2000; round++) {
        CHECK(line_bus_round(&lb, &src, acc, 1));
        if (src.sent == sizeof(s_pattern) && acc[0].accepted == sizeof(s_pattern)) {
            break;
        }
    }
    CHECK(src.sent == sizeof(s_pattern));
    CHECK(acc[0].len == sizeof(s_pattern));
    CHECK(acc[0].accepted == sizeof(s_pattern));
    CHECK(memcmp(s_rx0, s_pattern, sizeof(s_pattern)) == 0);
    CHECK(!src.sh.in_flight);
    CHECK(lb.bus.conflicts == 0);
}

/* LS-02: two acceptors, one slow: wired-OR NRFD/NDAC gate on the slowest;
 * both receive the identical stream. */
static void ls_02_two_acceptors_slowest_gates(void) {
    line_bus_t lb;
    sh_peer_t src;
    ah_peer_t acc[2];
    fill_pattern();
    memset(s_rx0, 0, sizeof(s_rx0));
    memset(s_rx1, 0, sizeof(s_rx1));
    line_bus_init(&lb);
    sh_peer_init(&src, 0, s_pattern, sizeof(s_pattern), 2u, true);
    ah_peer_init(&acc[0], 1, s_rx0, sizeof(s_rx0), 1u, 0u, true);
    ah_peer_init(&acc[1], 2, s_rx1, sizeof(s_rx1), 1u, 5u, true); /* slow */
    for (int round = 0; round < 4000; round++) {
        CHECK(line_bus_round(&lb, &src, acc, 2));
        /* T06: the wired-OR gating proven PER FRAME, not after the fact:
         * (a) the source never completes a byte the FAST acceptor has
         *     not accepted (it released NDAC before the byte could end);
         * (b) the source never advances past the SLOW acceptor -- NDAC
         *     (wired-OR) stays asserted until the last one accepts, so
         *     the whole bus runs at the slowest listener's pace. */
        CHECK(acc[0].accepted >= src.sent);
        CHECK(src.sent <= acc[1].accepted);
        if (src.sent == sizeof(s_pattern) && acc[0].accepted == sizeof(s_pattern) &&
            acc[1].accepted == sizeof(s_pattern)) {
            break;
        }
    }
    CHECK(src.sent == sizeof(s_pattern));
    CHECK(acc[0].accepted == sizeof(s_pattern));
    CHECK(acc[1].accepted == sizeof(s_pattern));
    CHECK(memcmp(s_rx0, s_pattern, sizeof(s_pattern)) == 0);
    CHECK(memcmp(s_rx1, s_pattern, sizeof(s_pattern)) == 0);
    /* The fast acceptor must never finish earlier than the slow one. */
    CHECK(acc[0].accepted >= acc[1].accepted);
    CHECK(lb.bus.conflicts == 0);
}

/* LS-03: ATN (role loss) interrupting the source mid-stream: in-flight byte
 * flagged unproven, DAV released, no bus deadlock afterwards. */
static void ls_03_role_loss_mid_stream(void) {
    line_bus_t lb;
    sh_peer_t src;
    ah_peer_t acc[1];
    fill_pattern();
    memset(s_rx0, 0, sizeof(s_rx0));
    line_bus_init(&lb);
    sh_peer_init(&src, 0, s_pattern, sizeof(s_pattern), 2u, true);
    ah_peer_init(&acc[0], 1, s_rx0, sizeof(s_rx0), 1u, 0u, true);
    bool interrupted = false;
    vbus_attach_as(&lb.bus, 99); /* the external controller stealing ATN */
    for (int round = 0; round < 2000 && !interrupted; round++) {
        CHECK(line_bus_round(&lb, &src, acc, 1));
        if (src.sh.state == GPIB_SH_STRS) {
            /* Command phase steals the bus: ATN asserted, data role gone. */
            vbus_assert(&lb.bus, 99, VBL_ATN);
            src.role_holdoff = true;
            interrupted = true;
        }
    }
    CHECK(interrupted);
    for (int round = 0; round < 50; round++) {
        CHECK(line_bus_round(&lb, &src, acc, 1));
    }
    CHECK(src.sh.in_flight);         /* interrupted byte: fate unproven */
    /* Deterministic: SIWS -> SIDS immediately (no pending byte, no role),
     * the in-flight uncertainty flag carries the incident. */
    CHECK(src.sh.state == GPIB_SH_SIDS);
    CHECK(!vbus_level(&lb.bus, VBL_DAV)); /* DAV released by the source */
    /* Captured prefix is intact. */
    CHECK(memcmp(s_rx0, s_pattern, acc[0].len) == 0);
    CHECK(lb.bus.conflicts == 0);
}

/* LS-04: T1 observed on the wire — DAV never asserts before the settle
 * window of the current byte has elapsed. */
static void ls_04_t1_on_the_wire(void) {
    line_bus_t lb;
    sh_peer_t src;
    ah_peer_t acc[1];
    uint64_t sdys_at = 0;
    uint64_t dav_first = 0;
    bool saw_sdys = false;
    bool saw_dav = false;
    fill_pattern();
    memset(s_rx0, 0, sizeof(s_rx0));
    line_bus_init(&lb);
    sh_peer_init(&src, 0, s_pattern, sizeof(s_pattern), 5u, true); /* T1=5us */
    ah_peer_init(&acc[0], 1, s_rx0, sizeof(s_rx0), 1u, 0u, true);
    for (int round = 0; round < 2000; round++) {
        CHECK(line_bus_round(&lb, &src, acc, 1));
        if (!saw_sdys && src.sh.state == GPIB_SH_SDYS) {
            saw_sdys = true;
            sdys_at = lb.now_us;
        }
        if (!saw_dav && src.sh.state == GPIB_SH_STRS) {
            saw_dav = true;
            dav_first = lb.now_us;
        }
        if (saw_sdys && saw_dav) {
            break;
        }
    }
    CHECK(saw_sdys);
    CHECK(saw_dav);
    CHECK(dav_first >= sdys_at + 5u); /* T1 settle honored on the wire */
}

/* LS-05: a command byte (ATN=1, CACS role) handshakes through the same
 * wires with the T3 accept path and decodes as the sent command. */
static void ls_05_command_byte(void) {
    line_bus_t lb;
    sh_peer_t src;
    ah_peer_t acc[1];
    static const uint8_t cmd[1] = { 0x3Fu }; /* UNL */
    fill_pattern();
    memset(s_rx0, 0, sizeof(s_rx0));
    line_bus_init(&lb);
    sh_peer_init(&src, 0, cmd, 1, 2u, false); /* command mode: CACS */
    ah_peer_init(&acc[0], 1, s_rx0, sizeof(s_rx0), 1u, 0u, true);
    for (int round = 0; round < 200; round++) {
        CHECK(line_bus_round(&lb, &src, acc, 1));
        if (src.sent == 1 && acc[0].accepted == 1) {
            break;
        }
    }
    CHECK(src.sent == 1);
    CHECK(acc[0].len == 1);
    CHECK(acc[0].accepted == 1);
    CHECK(s_rx0[0] == 0x3Fu);
    gpib_ifcmd_t dec = gpib_decode_atn_byte(s_rx0[0]);
    CHECK(dec.cls == GPIB_CMD_UNL);
    CHECK(lb.bus.conflicts == 0);
}

/* AE: primary addressing engine over decoded commands. */
static void ae_primary_addressing(void) {
    gpib_addr_engine_t e;
    gpib_addr_init(&e, 5);
    CHECK(!e.lads && !e.tads);
    CHECK(gpib_addr_step(&e, &(gpib_ifcmd_t){ GPIB_CMD_MLA, 0x25, 5 }) ==
          GPIB_AE_LISTEN_ON);
    CHECK(e.lads);
    /* MLA of another device never unlistens (multicast basis). */
    CHECK(gpib_addr_step(&e, &(gpib_ifcmd_t){ GPIB_CMD_MLA, 0x27, 7 }) ==
          GPIB_AE_NONE);
    CHECK(e.lads);
    /* MTA(other) untalks the previous talker but keeps listeners. */
    CHECK(gpib_addr_step(&e, &(gpib_ifcmd_t){ GPIB_CMD_MTA, 0x27, 7 }) ==
          GPIB_AE_NONE);
    CHECK(e.lads && !e.tads);
    /* MTA(own) unlistens and talk-addresses this device. */
    CHECK(gpib_addr_step(&e, &(gpib_ifcmd_t){ GPIB_CMD_MTA, 0x45, 5 }) ==
          GPIB_AE_LISTEN_OFF);
    CHECK(!e.lads && e.tads);
    CHECK(gpib_addr_step(&e, &(gpib_ifcmd_t){ GPIB_CMD_UNT, 0x5F, 0 }) ==
          GPIB_AE_TALK_OFF);
    CHECK(!e.tads);
    CHECK(gpib_addr_step(&e, &(gpib_ifcmd_t){ GPIB_CMD_SDC, 0x04, 0 }) ==
          GPIB_AE_DEVICE_CMD);
    CHECK(gpib_addr_step(&e, &(gpib_ifcmd_t){ GPIB_CMD_SECONDARY, 0x60, 0 }) ==
          GPIB_AE_NONE); /* TE/LE pending [P] */
}

void suite_linesim(void) {
    ls_01_single_acceptor();
    ls_02_two_acceptors_slowest_gates();
    ls_03_role_loss_mid_stream();
    ls_04_t1_on_the_wire();
    ls_05_command_byte();
    ae_primary_addressing();
}
