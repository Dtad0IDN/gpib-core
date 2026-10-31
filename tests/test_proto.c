#include <string.h>

#include "gpib/gpib_proto.h"
#include "test_util.h"

/* Verification vectors of §6 of the normative baseline
 * (docs/norms/IEEE_488_1_1987_SH_AH_Commands.md). */

/* --- command decoder vectors --------------------------------------------- */

static void cmd_fixed_commands(void) {
    struct {
        uint8_t raw;
        gpib_ifcmd_class_t cls;
    } v[] = {
        { 0x01, GPIB_CMD_GTL }, { 0x04, GPIB_CMD_SDC }, { 0x05, GPIB_CMD_PPC },
        { 0x08, GPIB_CMD_GET }, { 0x09, GPIB_CMD_TCT }, { 0x11, GPIB_CMD_LLO },
        { 0x14, GPIB_CMD_DCL }, { 0x15, GPIB_CMD_PPU }, { 0x18, GPIB_CMD_SPE },
        { 0x19, GPIB_CMD_SPD }
    };
    for (size_t i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
        gpib_ifcmd_t c = gpib_decode_atn_byte(v[i].raw);
        CHECK(c.cls == v[i].cls);
        CHECK(c.raw == v[i].raw);
    }
}

static void cmd_address_groups(void) {
    gpib_ifcmd_t c;
    c = gpib_decode_atn_byte(0x20);
    CHECK(c.cls == GPIB_CMD_MLA && c.arg == 0);
    c = gpib_decode_atn_byte(0x3E);
    CHECK(c.cls == GPIB_CMD_MLA && c.arg == 30);
    c = gpib_decode_atn_byte(0x3F);
    CHECK(c.cls == GPIB_CMD_UNL); /* not MLA31 */
    c = gpib_decode_atn_byte(0x40);
    CHECK(c.cls == GPIB_CMD_MTA && c.arg == 0);
    c = gpib_decode_atn_byte(0x5E);
    CHECK(c.cls == GPIB_CMD_MTA && c.arg == 30);
    c = gpib_decode_atn_byte(0x5F);
    CHECK(c.cls == GPIB_CMD_UNT); /* not MTA31 */
}

static void cmd_secondary_group(void) {
    uint8_t edges[4] = { 0x60, 0x6F, 0x70, 0x7F };
    uint8_t args[4] = { 0, 15, 16, 31 };
    for (int i = 0; i < 4; i++) {
        gpib_ifcmd_t c = gpib_decode_atn_byte(edges[i]);
        CHECK(c.cls == GPIB_CMD_SECONDARY);
        CHECK(c.arg == args[i]);
    }
}

static void cmd_undefined_and_masking(void) {
    /* Undefined codes (0x1F is CFE only in 2003: undefined here). */
    uint8_t und[4] = { 0x00, 0x02, 0x0A, 0x1F };
    for (int i = 0; i < 4; i++) {
        CHECK(gpib_decode_atn_byte(und[i]).cls == GPIB_CMD_UNDEFINED);
    }
    /* DIO8 does not participate in interface decoding. */
    CHECK(gpib_decode_atn_byte((uint8_t)(0x01 | 0x80u)).cls == GPIB_CMD_GTL);
    CHECK(gpib_decode_atn_byte((uint8_t)(0x20 | 0x80u)).cls == GPIB_CMD_MLA);
    CHECK(gpib_decode_atn_byte((uint8_t)(0x3F | 0x80u)).cls == GPIB_CMD_UNL);
}

/* --- SH vectors ----------------------------------------------------------- */

static void sh_fill(gpib_sh_in_t *in, bool atn) {
    memset(in, 0, sizeof(*in));
    in->atn = atn;
    in->tacs = !atn; /* data mode talker for these vectors */
    in->cacs = atn;
    in->rfd = true;
    in->dac = true;
    in->t1_us = 2u;
    in->now_us = 100u;
}

static void sh_basic_cycle(void) {
    gpib_sh_t sh;
    gpib_sh_in_t in;
    gpib_sh_out_t out;
    gpib_sh_init(&sh);
    CHECK(sh.state == GPIB_SH_SIDS);
    sh_fill(&in, false);
    in.nba = true;
    gpib_sh_step(&sh, &in, &out); /* SIDS -> SGNS (role active) */
    CHECK(sh.state == GPIB_SH_SGNS);
    CHECK(out.may_change_byte);
    CHECK(!out.dav_requested);
    in.now_us = 101;
    gpib_sh_step(&sh, &in, &out); /* SGNS -> SDYS (nba), T1 window starts */
    CHECK(sh.state == GPIB_SH_SDYS);
    CHECK(!out.may_change_byte); /* DIO/EOI stable in SDYS */
    in.now_us = 103;             /* T1=2us elapsed since 101 */
    in.rfd = true;
    gpib_sh_step(&sh, &in, &out); /* SDYS -> STRS (RFD && T1) */
    CHECK(sh.state == GPIB_SH_STRS);
    CHECK(out.dav_requested);
    CHECK(!out.may_change_byte);
    in.dac = false; /* slowest listener still holds NDAC */
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_STRS);
    CHECK(!out.byte_accepted);
    in.dac = true; /* last acceptor released NDAC */
    gpib_sh_step(&sh, &in, &out); /* STRS -> SWNS on DAC */
    CHECK(sh.state == GPIB_SH_SWNS);
    CHECK(out.byte_accepted);
    CHECK(!out.dav_requested); /* deterministic policy: DAV released */
    in.nba = false;
    gpib_sh_step(&sh, &in, &out); /* SWNS -> SGNS on !nba */
    CHECK(sh.state == GPIB_SH_SGNS);
}

static void sh_t1_gate(void) {
    gpib_sh_t sh;
    gpib_sh_in_t in;
    gpib_sh_out_t out;
    gpib_sh_init(&sh);
    sh_fill(&in, false);
    in.nba = true;
    gpib_sh_step(&sh, &in, &out);
    in.now_us = 102;
    gpib_sh_step(&sh, &in, &out); /* SDYS at t=102 */
    in.now_us = 103;              /* T1 not elapsed (needs >= 104) */
    in.rfd = true;
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_SDYS); /* held: settle window */
    in.now_us = 104;
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_STRS);
}

static void sh_slow_listeners(void) {
    gpib_sh_t sh;
    gpib_sh_in_t in;
    gpib_sh_out_t out;
    /* RFD held low: STRS cannot begin. */
    gpib_sh_init(&sh);
    sh_fill(&in, false);
    in.nba = true;
    gpib_sh_step(&sh, &in, &out);
    gpib_sh_step(&sh, &in, &out); /* SDYS at t=100, T1 deadline 102 */
    in.now_us = 1000u;            /* settle long over: only RFD gates now */
    in.rfd = false;
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_SDYS);
    /* DAC held low: byte not confirmed complete. */
    in.rfd = true;
    gpib_sh_step(&sh, &in, &out); /* STRS */
    in.dac = false;
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_STRS);
    CHECK(!out.byte_accepted);
}

static void sh_role_loss_from_states(void) {
    gpib_sh_t sh;
    gpib_sh_in_t in;
    gpib_sh_out_t out;
    /* From SDYS: back to SIDS, DAV stays released. */
    gpib_sh_init(&sh);
    sh_fill(&in, false);
    in.nba = true;
    gpib_sh_step(&sh, &in, &out); /* SGNS at t=100 */
    gpib_sh_step(&sh, &in, &out); /* SDYS at t=100, T1 deadline 102 */
    in.now_us = 1000u;            /* settle window long elapsed */
    in.tacs = false;              /* role revoked before transfer */
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_SIDS);
    CHECK(out.role_lost);
    CHECK(!out.dav_requested);
    /* From STRS: interrupted, in-flight byte flagged unproven. */
    gpib_sh_init(&sh);
    sh_fill(&in, false);
    in.nba = true;
    gpib_sh_step(&sh, &in, &out);
    gpib_sh_step(&sh, &in, &out); /* SDYS */
    in.now_us = 1000u;
    gpib_sh_step(&sh, &in, &out); /* STRS */
    CHECK(sh.state == GPIB_SH_STRS);
    in.tacs = false;
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_SIWS);
    CHECK(out.role_lost);
    CHECK(sh.in_flight);
    CHECK(!out.dav_requested);
    /* SWNS -> SIWS. */
    gpib_sh_init(&sh);
    sh_fill(&in, false);
    in.nba = true;
    gpib_sh_step(&sh, &in, &out);
    gpib_sh_step(&sh, &in, &out); /* SDYS */
    in.now_us = 1000u;
    gpib_sh_step(&sh, &in, &out); /* STRS */
    gpib_sh_step(&sh, &in, &out); /* SWNS (DAC) */
    CHECK(sh.state == GPIB_SH_SWNS);
    in.tacs = false;
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_SIWS);
    /* SIWS resume path: role returns -> SWNS, then cycles on. */
    in.tacs = true;
    in.nba = true;
    gpib_sh_step(&sh, &in, &out);
    CHECK(sh.state == GPIB_SH_SWNS);
}

/* --- AH vectors ----------------------------------------------------------- */

static void ah_atn_command_cycle(void) {
    gpib_ah_t ah;
    gpib_ah_in_t in;
    gpib_ah_out_t out;
    gpib_ah_init(&ah);
    memset(&in, 0, sizeof(in));
    CHECK(ah.state == GPIB_AH_AIDS);
    in.atn = true;
    in.dav = false;
    in.t3_us = 1u;
    in.now_us = 10u;
    gpib_ah_step(&ah, &in, &out); /* AIDS -> ANRS */
    CHECK(ah.state == GPIB_AH_ANRS);
    CHECK(!out.rfd_true && !out.dac_true);
    gpib_ah_step(&ah, &in, &out); /* ANRS -> ACRS (ATN && !DAV, !tcs) */
    CHECK(ah.state == GPIB_AH_ACRS);
    CHECK(out.rfd_true);
    in.dav = true;
    gpib_ah_step(&ah, &in, &out); /* ACRS -> ACDS, byte valid */
    CHECK(ah.state == GPIB_AH_ACDS);
    CHECK(out.byte_valid);
    CHECK(!out.rfd_true); /* hold-off the next byte */
    in.now_us = 10u;      /* T3 not elapsed */
    gpib_ah_step(&ah, &in, &out);
    CHECK(ah.state == GPIB_AH_ACDS);
    CHECK(!out.accepted_event);
    in.now_us = 11u;      /* T3 elapsed */
    gpib_ah_step(&ah, &in, &out); /* ACDS -> AWNS */
    CHECK(ah.state == GPIB_AH_AWNS);
    CHECK(out.accepted_event);
    CHECK(out.dac_true);
    in.dav = false;
    gpib_ah_step(&ah, &in, &out); /* AWNS -> ANRS */
    CHECK(ah.state == GPIB_AH_ANRS);
}

static void ah_data_backpressure_cycle(void) {
    gpib_ah_t ah;
    gpib_ah_in_t in;
    gpib_ah_out_t out;
    gpib_ah_init(&ah);
    memset(&in, 0, sizeof(in));
    in.lacs = true; /* addressed data listener, ATN=FALSE */
    in.t3_us = 1u;
    in.now_us = 100u;
    gpib_ah_step(&ah, &in, &out); /* AIDS -> ANRS */
    CHECK(ah.state == GPIB_AH_ANRS);
    in.rdy = true;
    gpib_ah_step(&ah, &in, &out); /* ANRS -> ACRS */
    CHECK(ah.state == GPIB_AH_ACRS);
    in.dav = true;
    gpib_ah_step(&ah, &in, &out); /* ACRS -> ACDS */
    CHECK(ah.state == GPIB_AH_ACDS);
    CHECK(out.byte_valid);
    /* Consumer not ready for the NEXT byte yet: acceptance completes the
     * current byte (rdy low = backpressure for the following one). */
    in.rdy = false;
    gpib_ah_step(&ah, &in, &out); /* ACDS -> AWNS */
    CHECK(ah.state == GPIB_AH_AWNS);
    CHECK(out.accepted_event);
    in.dav = false;
    gpib_ah_step(&ah, &in, &out); /* AWNS -> ANRS */
    CHECK(ah.state == GPIB_AH_ANRS);
    /* rdy stalls in ANRS/ACDS: holding rdy=TRUE in ACDS stalls completion. */
    in.rdy = true;
    gpib_ah_step(&ah, &in, &out); /* ANRS -> ACRS */
    in.dav = true;
    gpib_ah_step(&ah, &in, &out); /* ACDS */
    gpib_ah_step(&ah, &in, &out); /* rdy still TRUE: stays ACDS */
    CHECK(ah.state == GPIB_AH_ACDS);
    CHECK(!out.accepted_event);
}

static void ah_tcs_blocks(void) {
    gpib_ah_t ah;
    gpib_ah_in_t in;
    gpib_ah_out_t out;
    gpib_ah_init(&ah);
    memset(&in, 0, sizeof(in));
    in.atn = true;
    in.tcs = true; /* synchronous take control pending */
    in.now_us = 1u;
    gpib_ah_step(&ah, &in, &out); /* AIDS -> ANRS */
    gpib_ah_step(&ah, &in, &out); /* tcs blocks ANRS -> ACRS */
    CHECK(ah.state == GPIB_AH_ANRS);
    in.tcs = false;
    gpib_ah_step(&ah, &in, &out);
    CHECK(ah.state == GPIB_AH_ACRS);
}

static void ah_not_addressed(void) {
    gpib_ah_t ah;
    gpib_ah_in_t in;
    gpib_ah_out_t out;
    gpib_ah_init(&ah);
    memset(&in, 0, sizeof(in));
    in.atn = true;
    in.now_us = 1u;
    gpib_ah_step(&ah, &in, &out);
    gpib_ah_step(&ah, &in, &out);
    CHECK(ah.state == GPIB_AH_ACRS);
    in.atn = false; /* bus leaves command mode: unaddressed device idles */
    gpib_ah_step(&ah, &in, &out);
    CHECK(ah.state == GPIB_AH_AIDS);
    CHECK(out.rfd_true && out.dac_true); /* passive release */
}

void suite_proto(void) {
    cmd_fixed_commands();
    cmd_address_groups();
    cmd_secondary_group();
    cmd_undefined_and_masking();
    sh_basic_cycle();
    sh_t1_gate();
    sh_slow_listeners();
    sh_role_loss_from_states();
    ah_atn_command_cycle();
    ah_data_backpressure_cycle();
    ah_tcs_blocks();
    ah_not_addressed();
}
