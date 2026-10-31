#include <string.h>

#include "gpib/gpib_proto.h"
#include "test_util.h"

/* Vectors from the T/L/C/SR normative extract (§7.5 included). */

#define OWN 5u

static gpib_ifcmd_t cmd_of(gpib_ifcmd_class_t cls, uint8_t arg) {
    gpib_ifcmd_t c;
    c.cls = cls;
    c.arg = arg;
    c.raw = 0;
    return c;
}

/* --- Talker --------------------------------------------------------------- */

static void talker_basic(void) {
    gpib_talker_t t;
    gpib_t_in_t in;
    gpib_t_out_t out;
    gpib_talker_init(&t, OWN, true, false); /* T2-style: serial poll only */
    memset(&in, 0, sizeof(in));
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TIDS);
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_MTA, OWN);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TADS);
    CHECK(out.talk_addressed);
    in.cmd_valid = false;
    in.atn = false;
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TACS);
    in.atn = true;
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TADS); /* back on ATN within t2 */
    in.ifc = true;
    in.atn = false;
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TIDS);
    CHECK(out.talk_released);
}

static void talker_ota_untalk(void) {
    gpib_talker_t t;
    gpib_t_in_t in;
    gpib_t_out_t out;
    gpib_talker_init(&t, OWN, false, false);
    memset(&in, 0, sizeof(in));
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_MTA, OWN);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TADS);
    in.cmd = cmd_of(GPIB_CMD_MTA, 7); /* OTA: another talker addressed */
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TIDS);
    CHECK(out.talk_released);
}

static void talker_mla_unaddress_subset(void) {
    gpib_talker_t t4, t5;
    gpib_t_in_t in;
    gpib_t_out_t out;
    memset(&in, 0, sizeof(in));
    in.cmd_valid = true;
    in.atn = true; /* command phase: ATN asserted */
    /* T4: no unaddress on MLA. */
    gpib_talker_init(&t4, OWN, false, false);
    in.cmd = cmd_of(GPIB_CMD_MTA, OWN);
    gpib_talker_step(&t4, &in, &out);
    CHECK(t4.state == GPIB_T_TADS);
    in.cmd = cmd_of(GPIB_CMD_MLA, 3);
    gpib_talker_step(&t4, &in, &out);
    CHECK(t4.state == GPIB_T_TADS);
    /* T5: unaddress on MLA armed. */
    gpib_talker_init(&t5, OWN, true, true);
    in.cmd = cmd_of(GPIB_CMD_MTA, OWN);
    gpib_talker_step(&t5, &in, &out);
    CHECK(t5.state == GPIB_T_TADS);
    in.cmd = cmd_of(GPIB_CMD_MLA, 3);
    gpib_talker_step(&t5, &in, &out);
    CHECK(t5.state == GPIB_T_TIDS);
    CHECK(out.talk_released);
}

static void talker_serial_poll(void) {
    gpib_talker_t t;
    gpib_t_in_t in;
    gpib_t_out_t out;
    gpib_talker_init(&t, OWN, true, false);
    memset(&in, 0, sizeof(in));
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_SPE, 0);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.sp == GPIB_T_SPMS);
    in.cmd = cmd_of(GPIB_CMD_MTA, OWN);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TADS);
    in.cmd_valid = false;
    in.atn = false;
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_SPAS); /* SPMS routes to SPAS, not TACS */
    CHECK(out.entered_spas);
    in.atn = true;
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TADS);
    in.atn = false;
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_SPD, 0);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.sp == GPIB_T_SPIS);
    /* Without serial-poll capability the SPE command is ignored. */
    {
        gpib_talker_t t4;
        gpib_t_in_t in4;
        gpib_t_out_t out4;
        gpib_talker_init(&t4, OWN, false, false);
        memset(&in4, 0, sizeof(in4));
        in4.cmd_valid = true;
        in4.cmd = cmd_of(GPIB_CMD_SPE, 0);
        gpib_talker_step(&t4, &in4, &out4);
        CHECK(t4.sp == GPIB_T_SPIS);
    }
}

/* --- Listener ------------------------------------------------------------- */

static void listener_basic(void) {
    gpib_listener_t l;
    gpib_l_in_t in;
    gpib_l_out_t out;
    gpib_listener_init(&l, OWN, false); /* L2 */
    memset(&in, 0, sizeof(in));
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_MLA, OWN);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LADS);
    CHECK(out.listen_addressed);
    in.cmd_valid = false;
    in.atn = false;
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LACS);
    in.atn = true;
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LADS);
    in.atn = false;
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_UNL, 0);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LIDS);
    CHECK(out.listen_released);
    in.cmd = cmd_of(GPIB_CMD_MLA, OWN);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LADS);
    in.ifc = true;
    in.cmd_valid = false;
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LIDS);
}

static void listener_mta_unaddress_subset(void) {
    gpib_listener_t l2, l4;
    gpib_l_in_t in;
    gpib_l_out_t out;
    memset(&in, 0, sizeof(in));
    in.cmd_valid = true;
    in.atn = true; /* command phase: ATN asserted */
    /* L2: own MTA does NOT unlisten. */
    gpib_listener_init(&l2, OWN, false);
    in.cmd = cmd_of(GPIB_CMD_MLA, OWN);
    gpib_listener_step(&l2, &in, &out);
    CHECK(l2.state == GPIB_L_LADS);
    in.cmd = cmd_of(GPIB_CMD_MTA, OWN);
    gpib_listener_step(&l2, &in, &out);
    CHECK(l2.state == GPIB_L_LADS);
    /* L4: own MTA unlistens. */
    gpib_listener_init(&l4, OWN, true);
    in.cmd = cmd_of(GPIB_CMD_MLA, OWN);
    gpib_listener_step(&l4, &in, &out);
    in.cmd = cmd_of(GPIB_CMD_MTA, OWN);
    gpib_listener_step(&l4, &in, &out);
    CHECK(l4.state == GPIB_L_LIDS);
    CHECK(out.listen_released);
}

static void listener_controller_paths(void) {
    gpib_listener_t l;
    gpib_l_in_t in;
    gpib_l_out_t out;
    gpib_listener_init(&l, OWN, false);
    memset(&in, 0, sizeof(in));
    in.lon = true; /* listen-only */
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LADS);
    in.lon = false;
    in.lun = true;
    in.cacs = true; /* controller stops listening */
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LIDS);
    in.lun = false;
    in.ltn = true;
    in.cacs = true; /* controller listens (take control sync) */
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LADS);
    in.ltn = false;
    in.cacs = false;
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_UNL, 0);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LIDS);
}

/* --- Service Request ------------------------------------------------------- */

static void sr_cycle(void) {
    gpib_sr_t sr;
    gpib_sr_in_t in;
    gpib_sr_out_t out;
    gpib_sr_init(&sr);
    memset(&in, 0, sizeof(in));
    in.rsv = true;
    gpib_sr_step(&sr, &in, &out);
    CHECK(sr.state == GPIB_SR_SRQS);
    CHECK(out.srq_asserted && !out.rqs);
    in.spas = true; /* serial poll reads us */
    gpib_sr_step(&sr, &in, &out);
    CHECK(sr.state == GPIB_SR_APRS);
    CHECK(out.rqs && !out.srq_asserted);
    /* Negative: no resurrection of the old request from APRS. */
    in.spas = false;
    gpib_sr_step(&sr, &in, &out);
    CHECK(sr.state == GPIB_SR_APRS); /* rsv still true: retained */
    in.rsv = false;
    gpib_sr_step(&sr, &in, &out);
    CHECK(sr.state == GPIB_SR_NPRS);
}

/* --- Controller ------------------------------------------------------------ */

static void ctrl_fill(gpib_c_in_t *in, uint64_t now) {
    memset(in, 0, sizeof(*in));
    in->now_us = now;
    in->t6_us = 2u;
    in->t7_us = 2u;
    in->t8_us = 100u;
    in->t9_us = 2u;
    in->t10_us = 2u;
}

static void ctrl_ifc_takeover(void) {
    gpib_controller_t c;
    gpib_c_in_t in;
    gpib_c_out_t out;
    gpib_controller_init(&c);
    ctrl_fill(&in, 0);
    in.rsc = true;
    gpib_controller_step(&c, &in, &out); /* SACS */
    CHECK(c.sc == GPIB_C_SACS);
    in.sic = true;
    gpib_controller_step(&c, &in, &out); /* SIAS: system controller seizes */
    CHECK(c.ifc == GPIB_C_SIAS);
    CHECK(out.ifc_assert);
    CHECK(c.main == GPIB_C_CADS); /* IFC makes us CIC */
    CHECK(out.became_cic);
    in.sic = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.ifc == GPIB_C_SIAS); /* T8 hold keeps IFC asserted */
    in.now_us = 200;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.ifc == GPIB_C_SINS); /* T8 honored */
    CHECK(!out.ifc_assert);
    in.atn = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CACS);
    CHECK(out.cacs && out.cic);
    CHECK(out.atn_request);
}

static void ctrl_standby_cycle(void) {
    /* §7.5 vector: CSBS -> CSHS -> CSWS -> CAWS -> CACS. */
    gpib_controller_t c;
    gpib_c_in_t in;
    gpib_c_out_t out;
    gpib_controller_init(&c);
    ctrl_fill(&in, 0);
    in.rsc = true;
    in.sic = true;
    gpib_controller_step(&c, &in, &out);
    in.sic = false;
    in.now_us = 200;
    in.atn = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CACS);
    /* gts between bytes: standby, ATN released. */
    in.gts = true;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CSBS);
    CHECK(!out.atn_request);
    in.gts = false;
    /* mid-byte guards: rpp/gts refused while SDYS/STRS */
    in.rpp = true;
    in.sdys = true;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CSBS);
    in.rpp = false;
    in.sdys = false;
    /* synchronous takeover: wait ANRS, hold T10. */
    in.tcs = true;
    in.anrs = true;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CSHS);
    CHECK(!out.atn_request);
    in.now_us += 1;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CSHS); /* T10 not elapsed */
    in.now_us += 2;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CSWS); /* ATN asserted again */
    CHECK(out.atn_request);
    in.now_us += 3; /* T7 elapsed */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CAWS);
    in.now_us += 3; /* T9 elapsed */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CACS);
    CHECK(out.cacs);
}

static void ctrl_tct_transfer(void) {
    /* §7.5 vector: CACS -> CTRS -> CIDS. */
    gpib_controller_t c;
    gpib_c_in_t in;
    gpib_c_out_t out;
    gpib_controller_init(&c);
    ctrl_fill(&in, 0);
    in.rsc = true;
    in.sic = true;
    gpib_controller_step(&c, &in, &out);
    in.sic = false;
    in.now_us = 200;
    in.atn = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CACS);
    in.send_tct = true; /* sending TCT to another controller */
    in.strs = true;     /* byte is on the wire */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CTRS);
    CHECK(out.atn_request);
    in.strs = false;    /* byte completed */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CIDS);
    CHECK(out.lost_cic && !out.cic);
}

static void ctrl_parallel_poll(void) {
    gpib_controller_t c;
    gpib_c_in_t in;
    gpib_c_out_t out;
    gpib_controller_init(&c);
    ctrl_fill(&in, 0);
    in.rsc = true;
    in.sic = true;
    gpib_controller_step(&c, &in, &out);
    in.sic = false;
    in.now_us = 200;
    in.atn = false;
    gpib_controller_step(&c, &in, &out);
    in.rpp = true; /* parallel poll between bytes */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CPWS);
    CHECK(out.atn_request);
    in.now_us += 3; /* T6 elapsed */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CPPS);
    in.rpp = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CAWS);
}

static void ctrl_external_ifc_and_srq(void) {
    gpib_controller_t c;
    gpib_c_in_t in;
    gpib_c_out_t out;
    gpib_controller_init(&c);
    ctrl_fill(&in, 0);
    in.rsc = true;
    in.sic = true;
    gpib_controller_step(&c, &in, &out);
    in.sic = false;
    in.now_us = 200;
    in.atn = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CACS);
    /* Another controller's IFC throws us out (we hold SACS, so we would
     * re-seize; drop rsc first to model a plain device controller). */
    in.rsc = false;
    in.ifc = true;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.main == GPIB_C_CIDS);
    CHECK(out.lost_cic);
    in.ifc = false;
    /* SRQ monitor pair. */
    in.srq = true;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.srq == GPIB_C_CSRS);
    CHECK(out.srq_active);
    in.srq = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.srq == GPIB_C_CSNS);
}

static void ctrl_ren(void) {
    gpib_controller_t c;
    gpib_c_in_t in;
    gpib_c_out_t out;
    gpib_controller_init(&c);
    ctrl_fill(&in, 0);
    in.rsc = true;
    in.sre = true;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.ren == GPIB_C_SRNS); /* T8 stability gate */
    in.now_us += 50;
    gpib_controller_step(&c, &in, &out); /* window starts here */
    CHECK(c.ren == GPIB_C_SRNS);
    in.now_us += 40; /* 40 < T8 since window start */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.ren == GPIB_C_SRNS);
    in.now_us += 70; /* >= T8 since window start */
    gpib_controller_step(&c, &in, &out);
    CHECK(c.ren == GPIB_C_SRAS);
    CHECK(out.ren_assert);
    in.sre = false;
    gpib_controller_step(&c, &in, &out);
    CHECK(c.ren == GPIB_C_SRNS);
    /* R6-01 regression: init must clear EVERY field -- a poisoned struct
     * used to leave ren_released_us at garbage, making the re-assert
     * deadline unreachable (REN deterministically dead: SRNS forever). */
    {
        gpib_controller_t p;
        gpib_c_in_t pin;
        gpib_c_out_t pout;
        memset(&p, 0xA5, sizeof(p));
        gpib_controller_init(&p);
        ctrl_fill(&pin, 0);
        pin.rsc = true;
        pin.sre = true;
        pin.now_us = 160u; /* past T8 counted from wire-on (time 0) */
        /* A few steps: SRIS -> SRNS (SC group activates) -> armed ->
         * asserted. With a POISONED ren_released_us the deadline would
         * sit at 0xA5A5... and REN would never assert. */
        for (int i = 0; i < 4; i++) {
            gpib_controller_step(&p, &pin, &pout);
        }
        CHECK(p.ren == GPIB_C_SRAS); /* NOT stuck: deadline reachable */
        CHECK(pout.ren_assert);
    }
}

void suite_tlc(void) {
    talker_basic();
    talker_ota_untalk();
    talker_mla_unaddress_subset();
    talker_serial_poll();
    listener_basic();
    listener_mta_unaddress_subset();
    listener_controller_paths();
    sr_cycle();
    ctrl_ifc_takeover();
    ctrl_standby_cycle();
    ctrl_tct_transfer();
    ctrl_parallel_poll();
    ctrl_external_ifc_and_srq();
    ctrl_ren();
}
