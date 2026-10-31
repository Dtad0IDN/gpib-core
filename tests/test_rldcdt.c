#include <string.h>

#include "gpib/gpib_proto.h"
#include "test_util.h"

/* Directed vectors of the RL/DC/DT normative extract (§5). */

static gpib_ifcmd_t cmd_of(gpib_ifcmd_class_t cls, uint8_t arg) {
    gpib_ifcmd_t c;
    c.cls = cls;
    c.arg = arg;
    c.raw = 0;
    return c;
}

/* RL-01..RL-09 (RL-10 is the LE form: deferred with TE/LE, [P-EXT]). */
static void rl_vectors(void) {
    gpib_rl_t rl;
    gpib_rl_in_t in;
    gpib_rl_out_t out;
    memset(&in, 0, sizeof(in));
    in.cmd_valid = true;

    /* RL-01: LOCS, REN=0, MLA -> stays LOCS. */
    gpib_rl_init(&rl, true);
    in.ren = false;
    in.cmd = cmd_of(GPIB_CMD_MLA, 5);
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_LOCS && !out.changed);

    /* RL-02: LOCS, REN=1, !rtl, OWN MLA (listener selected: lads) -> REMS. */
    in.ren = true;
    in.lads = true;
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_REMS && out.changed);

    /* RL-11 (audit F09 regression): LOCS, REN=1, FOREIGN MLA (lads
     * false -- another device was addressed) must NOT enter REMS. */
    gpib_rl_init(&rl, true);
    in.lads = false;
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_LOCS && !out.changed);
    in.lads = true;

    /* RL-05: REMS, rtl (no simultaneous LLO) -> LOCS. */
    gpib_rl_init(&rl, true); /* re-enter REMS the qualified way */
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_REMS);
    in.cmd_valid = false;
    in.rtl = true;
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_LOCS && out.changed);
    in.rtl = false;

    /* RL-03: LOCS, REN=1, LLO -> LWLS. */
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_LLO, 0);
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_LWLS);

    /* RL-04: LWLS, OWN MLA (lads) -> RWLS. */
    in.cmd = cmd_of(GPIB_CMD_MLA, 5);
    in.lads = true;
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_RWLS);

    /* RL-07: RWLS, rtl -> stays (lockout). */
    in.cmd_valid = false;
    in.rtl = true;
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_RWLS && !out.changed);
    in.rtl = false;

    /* RL-08: RWLS, addressed, GTL -> LWLS (lockout preserved). */
    in.cmd_valid = true;
    in.lads = true;
    in.cmd = cmd_of(GPIB_CMD_GTL, 0);
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_LWLS);

    /* RL-09: RWLS, REN drop -> plain LOCS. */
    in.cmd_valid = false;
    in.ren = false;
    gpib_rl_step(&rl, &in, &out);
    CHECK(rl.state == GPIB_RL_LOCS);

    /* RL-06: REMS, rtl with simultaneously accepted LLO -> RWLS. */
    gpib_rl_init(&rl, true);
    in.ren = true;
    in.lads = true; /* own MLA: the qualified path into REMS */
    in.cmd_valid = true;
    in.rtl = false;
    in.cmd = cmd_of(GPIB_CMD_MLA, 5);
    gpib_rl_step(&rl, &in, &out); /* -> REMS first */
    CHECK(rl.state == GPIB_RL_REMS);
    in.rtl = true;
    in.cmd = cmd_of(GPIB_CMD_LLO, 0);
    gpib_rl_step(&rl, &in, &out); /* LLO wins over rtl */
    CHECK(rl.state == GPIB_RL_RWLS);

    /* RL2 subset: no lockout states, LLO is a no-op. */
    {
        gpib_rl_t rl2;
        gpib_rl_init(&rl2, false);
        in.rtl = false;
        in.cmd = cmd_of(GPIB_CMD_LLO, 0);
        gpib_rl_step(&rl2, &in, &out);
        CHECK(rl2.state == GPIB_RL_LOCS);
    }
}

static void dc_vectors(void) {
    gpib_dc_t dc;
    gpib_dc_in_t in;
    gpib_dc_out_t out;
    memset(&in, 0, sizeof(in));

    /* DC-01: unaddressed, DCL -> DCAS, exactly one pulse. */
    gpib_dc_init(&dc, true);
    in.acds = true;
    in.dcl = true;
    gpib_dc_step(&dc, &in, &out);
    CHECK(dc.state == GPIB_DC_DCAS && out.clear_pulse);
    gpib_dc_step(&dc, &in, &out);
    CHECK(dc.state == GPIB_DC_DCAS && !out.clear_pulse);

    /* DC-05: ACDS window closes -> DCIS, no addressing side effects. */
    in.acds = false;
    in.dcl = false;
    gpib_dc_step(&dc, &in, &out);
    CHECK(dc.state == GPIB_DC_DCIS && !out.clear_pulse);

    /* DC-02: unaddressed, SDC -> stays DCIS. */
    in.acds = true;
    in.sdc = true;
    in.lads = false;
    gpib_dc_step(&dc, &in, &out);
    CHECK(dc.state == GPIB_DC_DCIS && !out.clear_pulse);

    /* DC-03: addressed, SDC -> DCAS in DC1. */
    in.lads = true;
    gpib_dc_step(&dc, &in, &out);
    CHECK(dc.state == GPIB_DC_DCAS && out.clear_pulse);
    in.acds = false;
    in.sdc = false;
    in.lads = false;
    gpib_dc_step(&dc, &in, &out);
    CHECK(dc.state == GPIB_DC_DCIS);

    /* DC-04: addressed, SDC -> stays DCIS in DC2. */
    {
        gpib_dc_t dc2;
        gpib_dc_init(&dc2, false);
        in.acds = true;
        in.sdc = true;
        in.lads = true;
        gpib_dc_step(&dc2, &in, &out);
        CHECK(dc2.state == GPIB_DC_DCIS && !out.clear_pulse);
        in.sdc = false;
        in.dcl = true; /* DC2 still reacts to DCL */
        gpib_dc_step(&dc2, &in, &out);
        CHECK(dc2.state == GPIB_DC_DCAS && out.clear_pulse);
    }
}

static void dt_vectors(void) {
    gpib_dt_t dt;
    gpib_dt_in_t in;
    gpib_dt_out_t out;
    memset(&in, 0, sizeof(in));

    /* DT-01: unaddressed, GET -> stays. */
    gpib_dt_init(&dt);
    in.get = true;
    in.acds = true;
    in.lads = false;
    gpib_dt_step(&dt, &in, &out);
    CHECK(dt.state == GPIB_DT_DTIS && !out.trigger_pulse);

    /* DT-02: addressed, GET -> DTAS, one pulse. */
    in.lads = true;
    gpib_dt_step(&dt, &in, &out);
    CHECK(dt.state == GPIB_DT_DTAS && out.trigger_pulse);
    gpib_dt_step(&dt, &in, &out);
    CHECK(dt.state == GPIB_DT_DTAS && !out.trigger_pulse);

    /* DT-03: window ends -> DTIS (the DF operation continues on its own). */
    in.acds = false;
    gpib_dt_step(&dt, &in, &out);
    CHECK(dt.state == GPIB_DT_DTIS);

    /* DT-05 analog: a new GET pulses again (starting a second DF operation
     * is the application contract, not the FSM's). */
    in.acds = true;
    gpib_dt_step(&dt, &in, &out);
    CHECK(dt.state == GPIB_DT_DTAS && out.trigger_pulse);
}

void suite_rldcdt(void) {
    rl_vectors();
    dc_vectors();
    dt_vectors();
}
