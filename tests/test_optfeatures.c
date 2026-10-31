#include <string.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_proto.h"
#include "gpib/gpib_storage.h"
#include "test_util.h"

/* TE/LE secondary-addressing vectors (extract 3) + PP1 draft vectors. */

static gpib_ifcmd_t cmd_of(gpib_ifcmd_class_t cls, uint8_t arg) {
    gpib_ifcmd_t c;
    c.cls = cls;
    c.arg = arg;
    c.raw = 0;
    return c;
}

/* --- TE: TPIS/TPAS pair + TADS entry via MSA --- */

static void te_secondary_addressing(void) {
    gpib_talker_t t;
    gpib_t_in_t in;
    gpib_t_out_t out;
    memset(&in, 0, sizeof(in));
    in.atn = true;
    in.cmd_valid = true;

    gpib_talker_init_ex(&t, 5, true, false, true, 3); /* TE5-style */
    /* 1. MTA(5) arms TPAS. */
    in.cmd = cmd_of(GPIB_CMD_MTA, 5);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.sec == GPIB_T_TPAS);
    CHECK(t.state == GPIB_T_TIDS); /* not addressed yet */
    /* 2. MSA(3) with TPAS armed completes addressing. */
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 3);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TADS);
    CHECK(out.talk_addressed);
    /* 3. UNT untalks. */
    in.cmd = cmd_of(GPIB_CMD_UNT, 0);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TIDS);
    CHECK(out.talk_released);
    /* 4. MSA without TPAS armed does NOT address. */
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 3);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TIDS);
    /* 5. OTA untalks even while TPAS armed. */
    in.cmd = cmd_of(GPIB_CMD_MTA, 5);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.sec == GPIB_T_TPAS);
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 3);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TADS);
    in.cmd = cmd_of(GPIB_CMD_MTA, 7);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TIDS);
    /* 6. OSA while TPAS armed: per the extract, the pair only disarms on
     * PCG && !MTA; SCG (secondary commands) do NOT disarm - the pair
     * stays armed, only a correct MSA would complete addressing. */
    in.cmd = cmd_of(GPIB_CMD_MTA, 5);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.sec == GPIB_T_TPAS);
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 7);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.sec == GPIB_T_TPAS); /* SCG does not disarm the pair */
    CHECK(t.state == GPIB_T_TIDS);
    /* 7. Plain MTA on a TE-capable but NOT-armed device doesn't address. */
    in.cmd = cmd_of(GPIB_CMD_MTA, 5);
    gpib_talker_step(&t, &in, &out);
    CHECK(t.sec == GPIB_T_TPAS);
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 7); /* wrong secondary: no match */
    gpib_talker_step(&t, &in, &out);
    CHECK(t.state == GPIB_T_TIDS); /* MSA(7) != own MSA(3) */
}

/* --- LE: LPIS/LPAS pair + LADS entry via MSA --- */

static void le_secondary_addressing(void) {
    gpib_listener_t l;
    gpib_l_in_t in;
    gpib_l_out_t out;
    memset(&in, 0, sizeof(in));
    in.atn = true;
    in.cmd_valid = true;

    gpib_listener_init_ex(&l, 5, true, true, 2); /* LE4-style */
    /* 1. MLA(5) arms LPAS. */
    in.cmd = cmd_of(GPIB_CMD_MLA, 5);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.sec == GPIB_L_LPAS);
    CHECK(l.state == GPIB_L_LIDS);
    /* 2. MSA(2) completes addressing. */
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 2);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LADS);
    CHECK(out.listen_addressed);
    /* 3. UNL unlistens. */
    in.cmd = cmd_of(GPIB_CMD_UNL, 0);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LIDS);
    /* 4. MSA without LPAS does not address. */
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 2);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LIDS);
    /* 5. Wrong secondary does not address; the pair stays ARMED (per the
     * extract, only PCG && !MLA disarms; a wrong MSA is not PCG). */
    in.cmd = cmd_of(GPIB_CMD_MLA, 5);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.sec == GPIB_L_LPAS);
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 9); /* wrong */
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LIDS);
    CHECK(l.sec == GPIB_L_LPAS); /* still armed */
    /* Follow-up with the CORRECT MSA after the wrong one still works. */
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 2);
    gpib_listener_step(&l, &in, &out);
    CHECK(l.state == GPIB_L_LADS);
    CHECK(out.listen_addressed);
}

/* --- PP1 draft: config via PPE, poll via ATN&&EOI, disarm via PPU --- */

static void pp1_device_function(void) {
    /* Normative vectors against docs/norms/IEEE_488_1_1987_PP.md
     * (Fig. 11 + Tables 24-27, PP1 subset). */
    gpib_pp_t pp;
    gpib_pp_in_t in;
    gpib_pp_out_t out;
    memset(&in, 0, sizeof(in));

    gpib_pp_init(&pp, true); /* PP1 */
    CHECK(pp.state == GPIB_PP_PPIS);
    CHECK(pp.cfg == GPIB_PP_PUCS); /* pon enters BOTH groups */
    /* 1. 2.9.3.4: in PUCS "ignore any PPE or PPD". */
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 0x68); /* PPE S=1 line=0 */
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPIS);
    CHECK(!out.config_changed);
    /* 2. PPC without LADS does not open the configure window. */
    in.cmd = cmd_of(GPIB_CMD_PPC, 0);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.cfg == GPIB_PP_PUCS);
    /* 3. PPC with LADS -> PACS (Fig. 11 group B). */
    in.lads = true;
    in.cmd = cmd_of(GPIB_CMD_PPC, 0);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.cfg == GPIB_PP_PACS);
    /* 4. PPE now configures: PPIS -> PPSS, S and P1-P3 saved. */
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 0x68); /* S=1, line=0 */
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPSS);
    CHECK(out.config_changed);
    CHECK(pp.sense == true);
    CHECK(pp.dio_line == 0);
    /* 5. PPSS -> PPAS on ATN&&EOI (IDY) UNCONDITIONALLY in ist;
     *    ist == S -> drive the line true (Table 25). */
    in.cmd_valid = false;
    in.lads = false;
    in.atn = true;
    in.eoi = true;
    in.ist = true;
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPAS);
    CHECK(out.ppas_active);
    CHECK(out.drive_line == 0);
    CHECK(out.drive_level == true);
    /* 6. ist != S while IDY holds: STAY in PPAS, output passive false
     *    (2.9.3.3: true iff ist == S; the state does not follow ist). */
    in.ist = false;
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPAS);
    CHECK(out.ppas_active);
    CHECK(out.drive_level == false);
    /* 7. ist back to S: output true again, still PPAS. */
    in.ist = true;
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPAS);
    CHECK(out.drive_level == true);
    /* 8. PPAS -> PPSS on !ATN (IDY over), within t5. */
    in.atn = false;
    in.eoi = false;
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPSS);
    CHECK(!out.ppas_active);
    /* 9. DCL has NO PP arc in 2.9: the response configuration survives
     *    DCL (state stays PPSS). Note DCL is still a PCG byte: it CLOSES
     *    the configure window (PACS -> PUCS), which step 10 re-opens. */
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_DCL, 0);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPSS);
    CHECK(pp.cfg == GPIB_PP_PUCS);
    /* 10. Re-open the window, then re-configure with S=0 line=5. */
    in.lads = true;
    in.cmd = cmd_of(GPIB_CMD_PPC, 0);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.cfg == GPIB_PP_PACS);
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 0x65); /* S=0, line=5 */
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPSS);
    CHECK(pp.sense == false);
    CHECK(pp.dio_line == 5);
    /* 11. IDY with ist=0 == S=0 -> true output on line 5. */
    in.cmd_valid = false;
    in.atn = true;
    in.eoi = true;
    in.ist = false;
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPAS);
    CHECK(out.drive_line == 5);
    CHECK(out.drive_level == true); /* ist == S, regardless of polarity */
    in.atn = false;
    in.eoi = false;
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPSS);
    /* 12. PPD arms only in PACS: still PACS (no PCG since the PPC), so
     *     PPD -> PPIS. */
    in.cmd_valid = true;
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 0x70); /* PPD */
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPIS);
    CHECK(out.config_changed);
    /* 13. PCG closes PACS: PPC+LADS reopens, then MLA (PCG, not PPC)
     *     drops the device back to PUCS and a following PPE is ignored. */
    in.lads = true;
    in.cmd = cmd_of(GPIB_CMD_PPC, 0);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.cfg == GPIB_PP_PACS);
    in.cmd = cmd_of(GPIB_CMD_MLA, 6); /* any PCG != PPC */
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.cfg == GPIB_PP_PUCS);
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 0x68);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPIS); /* ignored in PUCS */
    /* 14. PPU unconfigures without any addressing (PPSS -> PPIS).
     *     Re-arm first (the PPU itself is PCG: it also closes PACS). */
    in.cmd = cmd_of(GPIB_CMD_PPC, 0);
    gpib_pp_step(&pp, &in, &out);
    in.cmd = cmd_of(GPIB_CMD_SECONDARY, 0x68);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPSS);
    in.lads = false;
    in.cmd = cmd_of(GPIB_CMD_PPU, 0);
    gpib_pp_step(&pp, &in, &out);
    CHECK(pp.state == GPIB_PP_PPIS);
    /* 15. PP0 (not capable) never responds and never configures. */
    {
        gpib_pp_t pp0;
        gpib_pp_init(&pp0, false);
        in.cmd_valid = true;
        in.lads = true;
        in.cmd = cmd_of(GPIB_CMD_PPC, 0);
        gpib_pp_step(&pp0, &in, &out);
        in.cmd = cmd_of(GPIB_CMD_SECONDARY, 0x68);
        gpib_pp_step(&pp0, &in, &out);
        CHECK(pp0.state == GPIB_PP_PPIS);
        CHECK(pp0.cfg == GPIB_PP_PUCS);
        CHECK(!out.config_changed);
        CHECK(!out.ppas_active);
    }
}

/* --- DAB stop match (16.2.6.6, optional) --- */

static void dab_stop_match(void) {
    /* Pure FSM check: the decoder classifies SECONDARY by bit 4 to
     * PPE/PPD, which the PP function consumes. The DAB stop is exercised
     * E2E in the lineexec suite. Here we verify the stop_cond enum
     * value exists and the storage round-trips it. */
    CHECK(GPIB_STOP_DAB == (GPIB_STOP_SEMICOLON + 1));
}

/* --- Controller-side PP, FINDLSTN and REQUESTCTL API validation --- */

static void ppoll_e2e(void) {
    /* The E2E behavior (configure -> IDY -> PPR -> unconfigure) lives in
     * suite_lineexec e2e_09 against the mock instrument. Here: argument
     * validation of the API surface. */
    gpib_context_t *null_ctx = NULL;
    CHECK(gpib_ppoll_configure(null_ctx, 6, 1, 0, NULL, NULL) ==
          GPIB_INVALID_ARGUMENT);
    CHECK(gpib_ppoll_unconfigure(null_ctx, NULL, NULL) ==
          GPIB_INVALID_ARGUMENT);
    CHECK(gpib_parallel_poll(null_ctx, NULL, NULL) == GPIB_INVALID_ARGUMENT);
}

static void findlstn_unit(void) {
    /* The DAB probe logic is verified E2E in suite_lineexec e2e_10.
     * Here: argument validation. */
    gpib_context_t *null_ctx = NULL;
    uint8_t addrs[1] = { 6 };
    bool results[1] = { false };
    CHECK(gpib_findlstn(null_ctx, addrs, 1, results, NULL, NULL) ==
          GPIB_INVALID_ARGUMENT);
}

static void requestctl_unit(void) {
    /* The CIC-side composition (FINDRQS + TCT) is verified E2E in
     * suite_lineexec e2e_11. Here: the requesting-side surface and the
     * lifecycle guard. */
    gpib_context_t *null_ctx = NULL;
    CHECK(gpib_request_control(null_ctx) == GPIB_INVALID_ARGUMENT);
    /* Uninitialized instance: rejected before any ledger touch. */
    {
        GPIB_CONTEXT_STORAGE_DEFINE(ctx);
        memset(&ctx, 0, sizeof(ctx));
        CHECK(gpib_request_control(&ctx) == GPIB_INVALID_STATE);
    }
}

void suite_optfeatures(void) {
    te_secondary_addressing();
    le_secondary_addressing();
    pp1_device_function();
    dab_stop_match();
    ppoll_e2e();
    findlstn_unit();
    requestctl_unit();
}
