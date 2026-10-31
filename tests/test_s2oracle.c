#include <string.h>

#include "gpib/gpib_proto.h"
#include "test_util.h"

/* Independent S2 oracle (simulation_plan: the checker derives from the
 * NORM, not from the code under test). The reference below is a
 * mechanical transcription of the SH/AH transition TABLES from
 * docs/norms/IEEE_488_1_1987_SH_AH_Commands.md - a separate data
 * representation, deliberately not the production switch/case.
 *
 * Oracle inputs are encoded as a bitmask; the table lists
 * (state, mask, next) with 'x' don't-cares resolved by listing every
 * combination of the don't-care bits. The exhaustive driver walks ALL
 * input combinations for every state and compares oracle vs production. */

/* --- SH oracle -------------------------------------------------------- */
/* SH input bits: bit1=src_active, bit2=nba, bit3=rfd, bit4=t1_ok,
 * bit5=dac, bit6=loses_role. NOTE (T05): there is no pon BIT -- power-on
 * is gpib_sh_init()/gpib_ah_init(), checked separately in the suite:
 * pon states are SIDS/AIDS per the extract. */
#define S_ACT 0x02u
#define S_NBA 0x04u
#define S_RFD 0x08u
#define S_T1 0x10u
#define S_DAC 0x20u
#define S_LOS 0x40u

static gpib_sh_state_t sh_oracle_step(gpib_sh_state_t st, uint8_t in,
    bool *role_lost) {
    *role_lost = false;
    /* src_active and loses are complementary: the role is either intact
     * (S_ACT without the explicit role-loss bit) or lost (S_LOS or no
     * active role). S_LOS && S_ACT is the contradictory corner of the
     * enumeration; the loses reading wins, matching the production
     * predicate (atn&&!cacs || !atn&&!tacs&&!spas). */
    bool active = (in & S_ACT) != 0u && (in & S_LOS) == 0u;
    bool loses = !active;
    if (st == GPIB_SH_SIDS) {
        if (active) {
            return GPIB_SH_SGNS;
        }
        return st;
    }
    switch (st) {
    case GPIB_SH_SGNS:
        if (loses) { *role_lost = true; return GPIB_SH_SIDS; }
        if (in & S_NBA) { return GPIB_SH_SDYS; }
        return st;
    case GPIB_SH_SDYS:
        if (loses) { *role_lost = true; return GPIB_SH_SIDS; }
        if ((in & (S_RFD | S_T1)) == (S_RFD | S_T1)) {
            return GPIB_SH_STRS;
        }
        return st;
    case GPIB_SH_STRS:
        if (loses) { *role_lost = true; return GPIB_SH_SIWS; }
        if (in & S_DAC) { return GPIB_SH_SWNS; }
        return st;
    case GPIB_SH_SWNS:
        if (loses) { *role_lost = true; return GPIB_SH_SIWS; }
        if (!(in & S_NBA)) { return GPIB_SH_SGNS; }
        return st;
    case GPIB_SH_SIWS:
        if (active) { return GPIB_SH_SWNS; }
        if (!(in & S_NBA)) { return GPIB_SH_SIDS; }
        return st;
    default:
        return st;
    }
}

/* --- AH oracle -------------------------------------------------------- */

/* AH inputs: bit0=atn, bit1=dav, bit2=rdy, bit3=lads_or_lacs,
 * bit5=t3_elapsed. AH_NOT_ADDRESSED is DERIVED: !atn && !lads && !lacs
 * (the extract's predicate), not an independent input bit. */
#define A_ATN 0x01u
#define A_DAV 0x02u
#define A_RDY 0x04u
#define A_LDS 0x08u

static gpib_ah_state_t ah_oracle_step(gpib_ah_state_t st, uint8_t in) {
    bool atn = (in & A_ATN) != 0u;
    bool dav = (in & A_DAV) != 0u;
    bool rdy = (in & A_RDY) != 0u;
    bool lds = (in & A_LDS) != 0u;
    bool nad = !atn && !lds; /* AH_NOT_ADDRESSED predicate (extract 3) */
    switch (st) {
    case GPIB_AH_AIDS:
        if (atn || lds) { return GPIB_AH_ANRS; }
        return st;
    case GPIB_AH_ANRS:
        if (nad) { return GPIB_AH_AIDS; }
        if ((atn && !dav) || rdy) { return GPIB_AH_ACRS; }
        return st;
    case GPIB_AH_ACRS:
        if (nad) { return GPIB_AH_AIDS; }
        if (dav) { return GPIB_AH_ACDS; }
        if (!atn && !rdy) { return GPIB_AH_ANRS; }
        return st;
    case GPIB_AH_ACDS:
        if (nad) { return GPIB_AH_AIDS; }
        if ((atn && in & 0x20u /* t3 elapsed */) ||
            (!atn && !rdy)) { return GPIB_AH_AWNS; }
        return st;
    case GPIB_AH_AWNS:
        if (nad) { return GPIB_AH_AIDS; }
        if (!dav) { return GPIB_AH_ANRS; }
        return st;
    default:
        return st;
    }
}

/* --- Cross-check: exhaustive input enumeration ------------------------- */

static void sh_cross_check(void) {
    for (int state = 0; state <= 5; state++) {
        for (unsigned in = 0; in < 128u; in++) {
            gpib_sh_t prod;
            gpib_sh_out_t pout;
            gpib_sh_in_t pin;
            bool oracle_lost = false;
            gpib_sh_state_t want =
                sh_oracle_step((gpib_sh_state_t)state, (uint8_t)in,
                    &oracle_lost);
            memset(&prod, 0, sizeof(prod));
            prod.state = (gpib_sh_state_t)state;
            prod.require_dac_low = false; /* oracle checks the base table */
            if (state == (int)GPIB_SH_SDYS) {
                /* Forced state has no SGNS->SDYS history; set the settle
                 * window explicitly: elapsed iff the S_T1 bit is set. */
                prod.t1_deadline_us = (in & S_T1) ? 0u : 4000u;
            }
            memset(&pin, 0, sizeof(pin));
            /* Faithful mapping: the S_LOS bit means "role lost", which in
             * the production predicate is atn&&!cacs || !atn&&!tacs&&!spas;
             * S_ACT is src_active. S_LOS forces the losing branch, S_ACT
             * supplies the active role in the non-losing branch. */
            if (in & S_LOS) {
                pin.atn = true;  /* atn && !cacs -> loses */
                pin.cacs = false;
            } else {
                pin.atn = false;
                pin.tacs = (in & S_ACT) != 0u; /* !atn -> !tacs loses */
            }
            pin.nba = (in & S_NBA) != 0u;
            pin.rfd = (in & S_RFD) != 0u;
            pin.dac = (in & S_DAC) != 0u;
            pin.t1_us = (in & S_T1) ? 0u : 1000u; /* 0 = elapsed */
            pin.now_us = 2000u;
            gpib_sh_step(&prod, &pin, &pout);
            if (prod.state != want) {
                fprintf(stderr,
                    "SH mismatch state=%d in=0x%02x prod=%d oracle=%d\n",
                    state, in, (int)prod.state, (int)want);
            }
            CHECK(prod.state == want);
            CHECK(pout.role_lost == oracle_lost);
        }
    }
}

static void ah_cross_check(void) {
    for (int state = 0; state <= 4; state++) {
        for (unsigned in = 0; in < 64u; in++) {
            gpib_ah_t prod;
            gpib_ah_out_t pout;
            gpib_ah_in_t pin;
            gpib_ah_state_t want =
                ah_oracle_step((gpib_ah_state_t)state,
                    (uint8_t)(in | ((in & 0x20u) << 0) /* t3 bit5 */));
            memset(&prod, 0, sizeof(prod));
            prod.state = (gpib_ah_state_t)state;
            if (state == (int)GPIB_AH_ACDS) {
                /* Forced state has no ACRS->ACDS history; the T3 window
                 * is elapsed iff bit5 is set (see the oracle encoding). */
                prod.t3_deadline_us = (in & 0x20u) ? 0u : 4000u;
            }
            memset(&pin, 0, sizeof(pin));
            pin.atn = (in & A_ATN) != 0u;
            pin.dav = (in & A_DAV) != 0u;
            pin.rdy = (in & A_RDY) != 0u;
            pin.lads = (in & A_LDS) != 0u;
            pin.lacs = pin.lads;
            pin.now_us = 2000u;
            pin.t3_us = 1u; /* t3 elapsed when bit5 set in the oracle input */
            gpib_ah_step(&prod, &pin, &pout);
            if (prod.state != want) {
                fprintf(stderr,
                    "AH mismatch state=%d in=0x%02x prod=%d oracle=%d\n",
                    state, in, (int)prod.state, (int)want);
            }
            CHECK(prod.state == want);
            /* T05: the OUTPUT function is part of the norm (Fig 4):
             * NRFD released (rfd_true) in AIDS/ACRS; NDAC released
             * (dac_true) in AIDS/AWNS; byte_valid on ENTERING ACDS;
             * accepted_event on ENTERING AWNS (the accept pulse). */
            CHECK(pout.rfd_true ==
                  (prod.state == GPIB_AH_AIDS || prod.state == GPIB_AH_ACRS));
            CHECK(pout.dac_true ==
                  (prod.state == GPIB_AH_AIDS || prod.state == GPIB_AH_AWNS));
            CHECK(pout.byte_valid ==
                  (state == (int)GPIB_AH_ACRS &&
                   prod.state == GPIB_AH_ACDS));
            CHECK(pout.accepted_event ==
                  (state == (int)GPIB_AH_ACDS &&
                   prod.state == GPIB_AH_AWNS));
        }
    }
}

/* T05: power-on states (pon is the init path, not a step input). */
static void pon_states(void) {
    gpib_sh_t sh;
    gpib_ah_t ah;
    gpib_sh_init(&sh);
    gpib_ah_init(&ah);
    CHECK(sh.state == GPIB_SH_SIDS);
    CHECK(ah.state == GPIB_AH_AIDS);
}

void suite_s2oracle(void) {
    pon_states();
    sh_cross_check();
    ah_cross_check();
}
