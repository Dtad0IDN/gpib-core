#include "gpib/gpib_proto.h"

/* --- ATN command decoding (--4 of the normative baseline) ----------------- */

gpib_ifcmd_t gpib_decode_atn_byte(uint8_t dio_byte) {
    gpib_ifcmd_t cmd;
    cmd.cls = GPIB_CMD_UNDEFINED;
    cmd.raw = dio_byte;
    cmd.arg = 0;
    uint8_t c = (uint8_t)(dio_byte & 0x7Fu); /* DIO8 does not participate */
    switch (c) {
    case 0x3Fu:
        cmd.cls = GPIB_CMD_UNL;
        break;
    case 0x5Fu:
        cmd.cls = GPIB_CMD_UNT;
        break;
    default:
        break;
    }
    if (cmd.cls != GPIB_CMD_UNDEFINED) {
        return cmd;
    }
    if (c >= 0x20u && c <= 0x3Eu) {
        cmd.cls = GPIB_CMD_MLA;
        cmd.arg = (uint8_t)(c & 0x1Fu);
        return cmd;
    }
    if (c >= 0x40u && c <= 0x5Eu) {
        cmd.cls = GPIB_CMD_MTA;
        cmd.arg = (uint8_t)(c & 0x1Fu);
        return cmd;
    }
    if (c >= 0x60u && c <= 0x7Fu) {
        cmd.cls = GPIB_CMD_SECONDARY;
        cmd.arg = (uint8_t)(c & 0x1Fu); /* PPE (0x60-0x6F) / PPD (0x70-0x7F) */
        return cmd;
    }
    switch (c) {
    case 0x01u:
        cmd.cls = GPIB_CMD_GTL;
        break;
    case 0x04u:
        cmd.cls = GPIB_CMD_SDC;
        break;
    case 0x05u:
        cmd.cls = GPIB_CMD_PPC;
        break;
    case 0x08u:
        cmd.cls = GPIB_CMD_GET;
        break;
    case 0x09u:
        cmd.cls = GPIB_CMD_TCT;
        break;
    case 0x11u:
        cmd.cls = GPIB_CMD_LLO;
        break;
    case 0x14u:
        cmd.cls = GPIB_CMD_DCL;
        break;
    case 0x15u:
        cmd.cls = GPIB_CMD_PPU;
        break;
    case 0x18u:
        cmd.cls = GPIB_CMD_SPE;
        break;
    case 0x19u:
        cmd.cls = GPIB_CMD_SPD;
        break;
    default:
        /* 0x1F is CFE only in the 2003 edition: stays undefined here. */
        cmd.cls = GPIB_CMD_UNDEFINED;
        break;
    }
    return cmd;
}

/* --- primary addressing engine (structural, [P] pending T/L tables) ------ */

void gpib_addr_init(gpib_addr_engine_t *e, uint8_t own_primary) {
    e->lads = false;
    e->tads = false;
    e->own_primary = own_primary;
}

gpib_addr_event_t gpib_addr_step(gpib_addr_engine_t *e,
    const gpib_ifcmd_t *cmd) {
    bool was_l = e->lads;
    bool was_t = e->tads;
    switch (cmd->cls) {
    case GPIB_CMD_UNL:
        e->lads = false;
        break;
    case GPIB_CMD_UNT:
        e->tads = false;
        break;
    case GPIB_CMD_MLA:
        if (cmd->arg == e->own_primary) {
            e->lads = true;
        }
        /* MLA of another device never unlistens: multicast basis. */
        break;
    case GPIB_CMD_MTA:
        /* MTA(own) untalks-and-unlistens the addressed device; MTA(other)
         * untalks the previous talker and leaves listeners untouched (the
         * controller can talk to previously addressed listeners). */
        if (cmd->arg == e->own_primary) {
            e->lads = false;
            e->tads = true;
        } else {
            e->tads = false;
        }
        break;
    case GPIB_CMD_GTL:
    case GPIB_CMD_SDC:
    case GPIB_CMD_PPC:
    case GPIB_CMD_GET:
    case GPIB_CMD_TCT:
    case GPIB_CMD_LLO:
    case GPIB_CMD_DCL:
    case GPIB_CMD_PPU:
    case GPIB_CMD_SPE:
    case GPIB_CMD_SPD:
        return GPIB_AE_DEVICE_CMD;
    default:
        /* SECONDARY and UNDEFINED do not change primary addressing. */
        return GPIB_AE_NONE;
    }
    if (e->lads && !was_l) {
        return GPIB_AE_LISTEN_ON;
    }
    if (!e->lads && was_l) {
        return GPIB_AE_LISTEN_OFF;
    }
    if (e->tads && !was_t) {
        return GPIB_AE_TALK_ON;
    }
    if (!e->tads && was_t) {
        return GPIB_AE_TALK_OFF;
    }
    return GPIB_AE_NONE;
}

/* --- Talker function (--2.5) ----------------------------------------------- */

void gpib_talker_init(gpib_talker_t *t, uint8_t own_primary,
    bool serial_poll_capable, bool unaddr_on_mla) {
    gpib_talker_init_ex(t, own_primary, serial_poll_capable, unaddr_on_mla,
        false, 0);
}

void gpib_talker_init_ex(gpib_talker_t *t, uint8_t own_primary,
    bool serial_poll_capable, bool unaddr_on_mla,
    bool secondary_capable, uint8_t own_secondary) {
    t->state = GPIB_T_TIDS;  /* pon */
    t->sp = GPIB_T_SPIS;     /* pon */
    t->sec = GPIB_T_TPIS;    /* pon */
    t->serial_poll_capable = serial_poll_capable;
    t->unaddr_on_mla = unaddr_on_mla;
    t->secondary_capable = secondary_capable;
    t->own_secondary = own_secondary;
    t->own_primary = own_primary;
}

void gpib_talker_step(gpib_talker_t *t, const gpib_t_in_t *in,
    gpib_t_out_t *out) {
    out->talk_addressed = false;
    out->talk_released = false;
    out->entered_spas = false;

    /* Serial poll pair: separate states, never replaced by TIDS/TACS. */
    if (t->serial_poll_capable) {
        switch (t->sp) {
        case GPIB_T_SPIS:
            if (!in->ifc && in->cmd_valid &&
                in->cmd.cls == GPIB_CMD_SPE) {
                t->sp = GPIB_T_SPMS;
            }
            break;
        case GPIB_T_SPMS:
            if (in->cmd_valid && in->cmd.cls == GPIB_CMD_SPD) {
                t->sp = GPIB_T_SPIS;
            } else if (in->ifc) {
                t->sp = GPIB_T_SPIS; /* within t4 */
            }
            break;
        default:
            break;
        }
    }

    bool mta_own = in->cmd_valid && in->cmd.cls == GPIB_CMD_MTA &&
                   in->cmd.arg == t->own_primary;
    bool ota = in->cmd_valid && in->cmd.cls == GPIB_CMD_MTA &&
               in->cmd.arg != t->own_primary;
    bool mla = in->cmd_valid && in->cmd.cls == GPIB_CMD_MLA;
    /* UNT untalks every talker (the extract table omits this arrow; it is
     * the standard's Untalk semantics). */
    bool unt = in->cmd_valid && in->cmd.cls == GPIB_CMD_UNT;
    /* Secondary addressing (TE): MSA(own) completes our addressing when
     * the TPAS pair was armed by our primary; OSA (other secondary)
     * untalks while we are TPAS-armed (extract 3). */
    bool msa_own = in->cmd_valid && in->cmd.cls == GPIB_CMD_SECONDARY &&
                   t->secondary_capable &&
                   in->cmd.arg == t->own_secondary;
    bool osa = in->cmd_valid && in->cmd.cls == GPIB_CMD_SECONDARY &&
               t->secondary_capable &&
               in->cmd.arg != t->own_secondary;

    /* TE secondary pair: TPIS --MTA&&ACDS--> TPAS; TPAS --PCG&&!MTA--> TPIS */
    if (t->secondary_capable) {
        switch (t->sec) {
        case GPIB_T_TPIS:
            if (mta_own) {
                t->sec = GPIB_T_TPAS;
            }
            break;
        case GPIB_T_TPAS:
            /* Disarm only on a PRIMARY command that is not our MTA
             * (extract: PCG && !MTA && ACDS). Secondary commands (MSA/OSA)
             * do NOT disarm the pair. */
            if (in->ifc || (in->cmd_valid &&
                            in->cmd.cls != GPIB_CMD_SECONDARY && !mta_own)) {
                t->sec = GPIB_T_TPIS;
            }
            break;
        default:
            break;
        }
    }

    /* Addressing entry: TE requires MSA(own)&&TPAS; plain T uses MTA(own) */
    bool address_match = t->secondary_capable
                             ? (msa_own && t->sec == GPIB_T_TPAS)
                             : mta_own;

    switch (t->state) {
    case GPIB_T_TIDS:
        if (!in->ifc && (address_match || in->ton)) {
            t->state = GPIB_T_TADS;
            out->talk_addressed = true;
            t->sec = GPIB_T_TPIS; /* pair consumed by the addressing */
        }
        break;
    case GPIB_T_TADS:
        if (in->ifc) {
            t->state = GPIB_T_TIDS; /* within t4 */
            out->talk_released = true;
        } else if (ota || unt) {
            t->state = GPIB_T_TIDS;
            out->talk_released = true;
        } else if (t->secondary_capable && osa && t->sec == GPIB_T_TPAS) {
            t->state = GPIB_T_TIDS;
            t->sec = GPIB_T_TPIS;
            out->talk_released = true;
        } else if (t->unaddr_on_mla && mla) {
            t->state = GPIB_T_TIDS; /* T5-T8 feature */
            out->talk_released = true;
        } else if (!in->atn) {
            if (t->serial_poll_capable && t->sp == GPIB_T_SPMS) {
                t->state = GPIB_T_SPAS;
                out->entered_spas = true;
            } else {
                t->state = GPIB_T_TACS;
            }
        }
        break;
    case GPIB_T_TACS:
        if (in->ifc) {
            t->state = GPIB_T_TIDS; /* within t4 */
            out->talk_released = true;
        } else if (in->atn) {
            t->state = GPIB_T_TADS; /* within t2 */
        }
        break;
    case GPIB_T_SPAS:
        if (in->ifc) {
            t->state = GPIB_T_TIDS; /* within t4 */
            out->talk_released = true;
        } else if (in->atn) {
            t->state = GPIB_T_TADS; /* within t2 */
        }
        break;
    default:
        break;
    }
}

/* --- Listener function (--2.6) --------------------------------------------- */

void gpib_listener_init(gpib_listener_t *l, uint8_t own_primary,
    bool unaddr_on_mta) {
    gpib_listener_init_ex(l, own_primary, unaddr_on_mta, false, 0);
}

void gpib_listener_init_ex(gpib_listener_t *l, uint8_t own_primary,
    bool unaddr_on_mta, bool secondary_capable, uint8_t own_secondary) {
    l->state = GPIB_L_LIDS; /* pon */
    l->sec = GPIB_L_LPIS;   /* pon */
    l->secondary_capable = secondary_capable;
    l->own_secondary = own_secondary;
    l->unaddr_on_mta = unaddr_on_mta;
    l->own_primary = own_primary;
}

void gpib_listener_step(gpib_listener_t *l, const gpib_l_in_t *in,
    gpib_l_out_t *out) {
    out->listen_addressed = false;
    out->listen_released = false;

    bool mla_own = in->cmd_valid && in->cmd.cls == GPIB_CMD_MLA &&
                   in->cmd.arg == l->own_primary;
    bool mta_own = in->cmd_valid && in->cmd.cls == GPIB_CMD_MTA &&
                   in->cmd.arg == l->own_primary;
    (void)mta_own; /* used only in the L3/L4 branch below */
    bool unl = in->cmd_valid && in->cmd.cls == GPIB_CMD_UNL;
    /* LE secondary pair: LPIS --MLA&&ACDS--> LPAS; LPAS --PCG&&!MLA--> LPIS */
    bool msa_own = in->cmd_valid && in->cmd.cls == GPIB_CMD_SECONDARY &&
                   l->secondary_capable &&
                   in->cmd.arg == l->own_secondary;

    if (l->secondary_capable) {
        switch (l->sec) {
        case GPIB_L_LPIS:
            if (mla_own) {
                l->sec = GPIB_L_LPAS;
            }
            break;
        case GPIB_L_LPAS:
            /* Disarm only on a PRIMARY command that is not our MLA
             * (extract: PCG && !MLA && ACDS). Secondary commands do NOT
             * disarm the pair. */
            if (in->ifc || (in->cmd_valid &&
                            in->cmd.cls != GPIB_CMD_SECONDARY && !mla_own)) {
                l->sec = GPIB_L_LPIS;
            }
            break;
        default:
            break;
        }
    }

    /* Addressing entry: LE requires MSA(own)&&LPAS; plain L uses MLA(own) */
    bool address_match = l->secondary_capable
                             ? (msa_own && l->sec == GPIB_L_LPAS)
                             : mla_own;

    switch (l->state) {
    case GPIB_L_LIDS:
        if (!in->ifc && (address_match || in->lon ||
                         (in->ltn && in->cacs))) {
            l->state = GPIB_L_LADS;
            out->listen_addressed = true;
            l->sec = GPIB_L_LPIS; /* pair consumed by the addressing */
        }
        break;
    case GPIB_L_LADS:
        if (in->ifc) {
            l->state = GPIB_L_LIDS; /* within t4 */
            out->listen_released = true;
        } else if (unl) {
            l->state = GPIB_L_LIDS;
            out->listen_released = true;
        } else if (in->lun && in->cacs) {
            l->state = GPIB_L_LIDS;
            out->listen_released = true;
        } else if (l->unaddr_on_mta && mta_own) {
            /* L3/L4 feature: being talk-addressed unstens the listener. */
            l->state = GPIB_L_LIDS;
            out->listen_released = true;
        } else if (!in->atn) {
            l->state = GPIB_L_LACS; /* within t2 */
        }
        break;
    case GPIB_L_LACS:
        if (in->ifc) {
            l->state = GPIB_L_LIDS; /* within t4 */
            out->listen_released = true;
        } else if (in->atn) {
            l->state = GPIB_L_LADS; /* within t2 */
        }
        break;
    default:
        break;
    }
}

/* --- Service Request function (--2.7) -------------------------------------- */

void gpib_sr_init(gpib_sr_t *sr) {
    sr->state = GPIB_SR_NPRS; /* pon */
}

void gpib_sr_step(gpib_sr_t *sr, const gpib_sr_in_t *in, gpib_sr_out_t *out) {
    switch (sr->state) {
    case GPIB_SR_NPRS:
        if (in->rsv && !in->spas) {
            sr->state = GPIB_SR_SRQS;
        }
        break;
    case GPIB_SR_SRQS:
        if (in->spas) {
            sr->state = GPIB_SR_APRS;
        } else if (!in->rsv) {
            sr->state = GPIB_SR_NPRS;
        }
        break;
    case GPIB_SR_APRS:
        /* No direct APRS -> SRQS: the request must not resurrect itself. */
        if (!in->rsv && !in->spas) {
            sr->state = GPIB_SR_NPRS;
        }
        break;
    default:
        break;
    }
    out->srq_asserted = (sr->state == GPIB_SR_SRQS);
    out->rqs = (sr->state == GPIB_SR_APRS);
}

/* --- Controller function (--2.12), five independent groups ----------------- */

void gpib_controller_init(gpib_controller_t *c) {
    c->main = GPIB_C_CIDS;
    c->srq = GPIB_C_CSNS;
    c->sc = GPIB_C_SNAS;
    c->ifc = GPIB_C_SIIS;
    c->ren = GPIB_C_SRIS;
    c->main_hold = false;
    c->ifc_hold = false;
    c->ren_hold = false;
    c->main_deadline = 0;
    c->ifc_deadline = 0;
    c->ren_deadline = 0;
    /* R6-01: EVERY field must be written -- a field-by-field init leaves
     * stale memory alive; a poisoned ren_released_us made the REN
     * re-assertion deadline unreachable and REN deterministically dead.
     * Time zero is the wire-on instant: REN has been FALSE since power
     * up, so the first sre only waits out the remaining T8 window. */
    c->ren_released_us = 0;
}

static void hold_main(gpib_controller_t *c, uint64_t now, uint32_t dur) {
    c->main_deadline = now + (uint64_t)dur;
    c->main_hold = true;
}

void gpib_controller_step(gpib_controller_t *c, const gpib_c_in_t *in,
    gpib_c_out_t *out) {
    out->cic = false;
    out->cacs = false;
    out->atn_request = false;
    out->ifc_assert = false;
    out->ren_assert = false;
    out->srq_active = false;
    out->became_cic = false;
    out->lost_cic = false;

    /* Group: system controller authority. */
    if (c->sc == GPIB_C_SNAS) {
        if (in->rsc) {
            c->sc = GPIB_C_SACS;
        }
    } else if (!in->rsc) {
        c->sc = GPIB_C_SNAS;
    }
    bool sc_active = (c->sc == GPIB_C_SACS);

    /* Group: IFC driver. SIAS asserts immediately; the exit to SINS waits
     * for the mandatory T8 hold (length of IFC false >= T8). */
    switch (c->ifc) {
    case GPIB_C_SIIS:
        if (sc_active) {
            if (in->sic) {
                c->ifc = GPIB_C_SIAS;
                c->ifc_deadline = in->now_us + (uint64_t)in->t8_us;
                c->ifc_hold = true;
            } else {
                c->ifc = GPIB_C_SINS;
            }
        }
        break;
    case GPIB_C_SINS:
        if (!sc_active) {
            c->ifc = GPIB_C_SIIS;
        } else if (in->sic) {
            c->ifc = GPIB_C_SIAS;
            c->ifc_deadline = in->now_us + (uint64_t)in->t8_us;
            c->ifc_hold = true;
        }
        break;
    case GPIB_C_SIAS:
        if (!sc_active) {
            c->ifc = GPIB_C_SIIS;
            c->ifc_hold = false;
        } else if (!in->sic &&
                   (!c->ifc_hold || in->now_us >= c->ifc_deadline)) {
            c->ifc = GPIB_C_SINS; /* T8 honored */
            c->ifc_hold = false;
        }
        break;
    default:
        break;
    }
    out->ifc_assert = (c->ifc == GPIB_C_SIAS);

    /* Group: REN driver. Asserting REN from SRNS requires sre stable for
     * T8 (length of REN false). SRIS collapses into SRNS on SACS ([P]
     * one-step simplification of the direct SRIS -> SRAS arrow). */
    switch (c->ren) {
    case GPIB_C_SRIS:
        if (sc_active) {
            c->ren = GPIB_C_SRNS;
            c->ren_hold = false;
        }
        break;
    case GPIB_C_SRNS:
        if (!sc_active) {
            c->ren = GPIB_C_SRIS;
            c->ren_hold = false;
        } else if (in->sre) {
            /* F14: the low-time budget is anchored to the actual wire
             * edge (SRAS->SRNS stamped ren_released_us). If the line
             * has already been false long enough, assert immediately;
             * otherwise wait out the remainder -- never from scratch. */
            if (!c->ren_hold) {
                c->ren_deadline = c->ren_released_us + (uint64_t)in->t8_us;
                c->ren_hold = true;
            }
            if (in->now_us >= c->ren_deadline) {
                c->ren = GPIB_C_SRAS;
                c->ren_hold = false;
            }
        } else {
            c->ren_hold = false;
        }
        break;
    case GPIB_C_SRAS:
        if (!sc_active) {
            c->ren = GPIB_C_SRIS;
        } else if (!in->sre) {
            c->ren = GPIB_C_SRNS;
            c->ren_released_us = in->now_us; /* the wire edge (F14) */
        }
        break;
    default:
        break;
    }
    out->ren_assert = (c->ren == GPIB_C_SRAS);

    /* Group: SRQ monitor. */
    if (c->srq == GPIB_C_CSNS) {
        if (in->srq) {
            c->srq = GPIB_C_CSRS;
        }
    } else if (!in->srq) {
        c->srq = GPIB_C_CSNS;
    }
    out->srq_active = (c->srq == GPIB_C_CSRS);

    /* Group: main transfer/command/poll chain. */
    if (c->main_hold && in->now_us >= c->main_deadline) {
        c->main_hold = false;
    }
    bool ext_ifc = in->ifc && !out->ifc_assert; /* someone else's IFC */
    switch (c->main) {
    case GPIB_C_CIDS:
        if ((in->tct_received && in->tads) || out->ifc_assert) {
            c->main = GPIB_C_CADS;
            out->became_cic = true;
        }
        break;
    case GPIB_C_CADS:
        if (!in->atn) {
            c->main = GPIB_C_CACS;
        } else if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        }
        break;
    case GPIB_C_CACS:
        if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS; /* within t4 */
            c->main_hold = false;
            out->lost_cic = true;
        } else if (in->send_tct && !in->tads) {
            c->main = GPIB_C_CTRS; /* transfer control away */
        } else if (in->rpp && !in->sdys && !in->strs) {
            c->main = GPIB_C_CPWS;
            hold_main(c, in->now_us, in->t6_us);
        } else if (in->gts && !in->sdys && !in->strs) {
            c->main = GPIB_C_CSBS; /* go to standby between bytes */
        }
        break;
    case GPIB_C_CTRS:
        if (!in->strs) { /* the TCT byte fully completed */
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        } else if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        }
        break;
    case GPIB_C_CSBS:
        if (in->tcs && in->anrs) {
            c->main = GPIB_C_CSHS; /* synchronous takeover */
            hold_main(c, in->now_us, in->t10_us);
        } else if (in->tca) {
            c->main = GPIB_C_CSWS; /* asynchronous takeover */
            hold_main(c, in->now_us, in->t7_us);
        } else if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        }
        break;
    case GPIB_C_CSHS:
        if (!in->tcs) {
            c->main = GPIB_C_CSBS;
            c->main_hold = false;
        } else if (!c->main_hold) { /* T10 elapsed */
            c->main = GPIB_C_CSWS;
            hold_main(c, in->now_us, in->t7_us);
        } else if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            c->main_hold = false;
            out->lost_cic = true;
        }
        break;
    case GPIB_C_CSWS:
        if (!c->main_hold || in->tads) { /* T7 elapsed or addressed */
            c->main = GPIB_C_CAWS;
            hold_main(c, in->now_us, in->t9_us);
        } else if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        }
        break;
    case GPIB_C_CAWS:
        if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        } else if (in->rpp) {
            c->main = GPIB_C_CPWS;
            hold_main(c, in->now_us, in->t6_us);
        } else if (!in->rpp && !c->main_hold) { /* T9 elapsed */
            c->main = GPIB_C_CACS;
        }
        break;
    case GPIB_C_CPWS:
        if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        } else if (!c->main_hold) { /* T6 elapsed */
            c->main = GPIB_C_CPPS;
        } else if (!in->rpp) {
            c->main = GPIB_C_CAWS;
            c->main_hold = false;
        }
        break;
    case GPIB_C_CPPS:
        if (ext_ifc && !sc_active) {
            c->main = GPIB_C_CIDS;
            out->lost_cic = true;
        } else if (!in->rpp) {
            c->main = GPIB_C_CAWS;
        }
        break;
    default:
        break;
    }

    out->cic = (c->main != GPIB_C_CIDS);
    out->cacs = (c->main == GPIB_C_CACS);
    /* ATN is driven by the active controller states; CSBS/CSHS keep it
     * released for the data transfer, CSWS+ re-assert it. */
    out->atn_request =
        (c->main == GPIB_C_CACS || c->main == GPIB_C_CTRS ||
         c->main == GPIB_C_CSWS || c->main == GPIB_C_CAWS ||
         c->main == GPIB_C_CPWS || c->main == GPIB_C_CPPS);
}

/* --- Remote/Local function (--2.8) ----------------------------------------- */

void gpib_rl_init(gpib_rl_t *rl, bool lockout_capable) {
    rl->state = GPIB_RL_LOCS; /* pon */
    rl->lockout_capable = lockout_capable;
}

void gpib_rl_step(gpib_rl_t *rl, const gpib_rl_in_t *in, gpib_rl_out_t *out) {
    gpib_rl_state_t from = rl->state;
    /* F09: the MLA event must be OWN-address qualified. The listener
     * selection (LADS, including the LE pair after MSA(own)) is the
     * normative "this device was addressed" fact; a FOREIGN MLA byte
     * must not push an unaddressed device into REMS/RWLS. */
    bool mla = in->cmd_valid && in->cmd.cls == GPIB_CMD_MLA && in->lads;
    bool llo = in->cmd_valid && in->cmd.cls == GPIB_CMD_LLO;
    bool gtl = in->cmd_valid && in->cmd.cls == GPIB_CMD_GTL;

    switch (rl->state) {
    case GPIB_RL_LOCS:
        if (in->ren && !in->rtl && mla) {
            rl->state = GPIB_RL_REMS;
        } else if (in->ren && llo && rl->lockout_capable) {
            rl->state = GPIB_RL_LWLS;
        }
        break;
    case GPIB_RL_LWLS:
        if (mla) {
            rl->state = GPIB_RL_RWLS;
        } else if (!in->ren) {
            rl->state = GPIB_RL_LOCS; /* within t4 */
        }
        break;
    case GPIB_RL_REMS:
        if (llo && rl->lockout_capable) {
            rl->state = GPIB_RL_RWLS;
        } else if (!in->ren) {
            rl->state = GPIB_RL_LOCS; /* within t4 */
        } else if (gtl && in->lads) {
            rl->state = GPIB_RL_LOCS;
        } else if (in->rtl && !(in->cmd_valid &&
                                in->cmd.cls == GPIB_CMD_LLO)) {
            /* rtl is shadowed by a simultaneously accepted LLO. */
            rl->state = GPIB_RL_LOCS;
        }
        break;
    case GPIB_RL_RWLS:
        if (!in->ren) {
            rl->state = GPIB_RL_LOCS; /* within t4, lockout cleared */
        } else if (gtl && in->lads) {
            rl->state = GPIB_RL_LWLS; /* lockout PRESERVED by GTL */
        }
        /* rtl ignored under lockout */
        break;
    default:
        break;
    }
    out->changed = (rl->state != from);
    out->entered = rl->state;
}

/* --- Device Clear function (--2.10) ----------------------------------------- */

void gpib_dc_init(gpib_dc_t *dc, bool sdc_capable) {
    dc->state = GPIB_DC_DCIS; /* project convention: no pon arrow */
    dc->sdc_capable = sdc_capable;
}

void gpib_dc_step(gpib_dc_t *dc, const gpib_dc_in_t *in, gpib_dc_out_t *out) {
    bool armed = in->dcl || (dc->sdc_capable && in->sdc && in->lads);
    gpib_dc_state_t from = dc->state;
    if (dc->state == GPIB_DC_DCIS) {
        if (in->acds && armed) {
            dc->state = GPIB_DC_DCAS;
        }
    } else {
        if (!in->acds || !armed) {
            dc->state = GPIB_DC_DCIS; /* addressing is NOT reset */
        }
    }
    /* Pulse strictly on the ENTRY into DCAS (single DF event per byte). */
    out->clear_pulse = (dc->state == GPIB_DC_DCAS && from == GPIB_DC_DCIS);
}

/* --- Device Trigger function (--2.11) ---------------------------------------- */

void gpib_dt_init(gpib_dt_t *dt) {
    dt->state = GPIB_DT_DTIS; /* project convention */
}

void gpib_dt_step(gpib_dt_t *dt, const gpib_dt_in_t *in, gpib_dt_out_t *out) {
    gpib_dt_state_t from = dt->state;
    if (dt->state == GPIB_DT_DTIS) {
        if (in->get && in->lads && in->acds) {
            dt->state = GPIB_DT_DTAS;
        }
    } else {
        if (!in->get || !in->lads || !in->acds) {
            dt->state = GPIB_DT_DTIS;
        }
    }
    out->trigger_pulse = (dt->state == GPIB_DT_DTAS && from == GPIB_DT_DTIS);
}

/* --- Parallel Poll function (488.1 --2.9, PP1) ----------------------------
 * Normative machine per docs/norms/IEEE_488_1_1987_PP.md: Fig. 11 has
 * TWO independent groups (response PPIS/PPSS/PPAS, configure
 * PUCS/PACS); Tables 24-27 define the messages, outputs and subsets. */

void gpib_pp_init(gpib_pp_t *pp, bool capable) {
    pp->state = GPIB_PP_PPIS;
    pp->cfg = GPIB_PP_PUCS;
    pp->capable = capable;
    pp->dio_line = 0;
    pp->sense = false;
    pp->ist = false;
}

void gpib_pp_step(gpib_pp_t *pp, const gpib_pp_in_t *in, gpib_pp_out_t *out) {
    out->ppas_active = false;
    out->drive_line = 0;
    out->drive_level = false;
    out->config_changed = false;

    if (!pp->capable) {
        return; /* PP0: no capability, states stay at pon values */
    }
    pp->ist = in->ist;

    /* Command classification. cmd_valid implies the byte was handshaken
     * (ACDS active). SCG (0x60-0x7F) splits by bit 4 into PPE/PPD;
     * every non-secondary ATN byte is PCG. */
    if (in->cmd_valid) {
        bool is_secondary = in->cmd.cls == GPIB_CMD_SECONDARY;
        bool is_ppe = is_secondary && (in->cmd.arg & 0x10u) == 0u;
        bool is_ppd = is_secondary && (in->cmd.arg & 0x10u) != 0u;
        bool is_pcg = !is_secondary;
        bool is_ppc = in->cmd.cls == GPIB_CMD_PPC;

        /* Group B (configure): PUCS -> PACS on PPC&&LADS&&ACDS;
         * PACS -> PUCS on PCG&&!PPC&&ACDS (any addressed/universal/
         * address byte ends the configure window). */
        if (is_pcg) {
            if (is_ppc && in->lads) {
                pp->cfg = GPIB_PP_PACS;
            } else {
                pp->cfg = GPIB_PP_PUCS;
            }
        }

        /* Group A (response) configure arcs: PPE arms only in PACS
         * (2.9.3.4: PUCS shall ignore PPE/PPD); the PPD arc is
         * [PPD&&PACS&&ACDS]; PPU needs no addressing. PP1 excludes
         * the lpe arcs (Table 27). No DCL arc exists in 2.9. */
        if (in->cmd.cls == GPIB_CMD_PPU) {
            if (pp->state != GPIB_PP_PPIS) {
                pp->state = GPIB_PP_PPIS;
                out->config_changed = true;
            }
        } else if (is_ppd && pp->cfg == GPIB_PP_PACS) {
            if (pp->state != GPIB_PP_PPIS) {
                pp->state = GPIB_PP_PPIS;
                out->config_changed = true;
            }
        } else if (is_ppe && pp->cfg == GPIB_PP_PACS) {
            /* PPE = 110S P1 P2 P3: save S and the PPR assignment
             * (Table 26: PPR1..PPR8 by P1..P3). */
            pp->sense = (in->cmd.arg & 0x08u) != 0u;
            pp->dio_line = (uint8_t)(in->cmd.arg & 0x07u);
            pp->state = GPIB_PP_PPSS;
            out->config_changed = true;
        }
    }

    /* Group A state transitions (Fig. 11): PPSS -> PPAS on IDY&&ATN
     * (unconditional in ist — the ist comparison is an OUTPUT function,
     * Table 25); PPAS -> PPSS on !ATN||!IDY. Both within t5. */
    switch (pp->state) {
    case GPIB_PP_PPIS:
        break;
    case GPIB_PP_PPSS:
        if (in->atn && in->eoi) {
            pp->state = GPIB_PP_PPAS;
        }
        break;
    case GPIB_PP_PPAS:
        if (!in->atn || !in->eoi) {
            pp->state = GPIB_PP_PPSS;
        }
        break;
    default:
        break;
    }

    /* Output mapping (Table 25): in PPAS the assigned PPR line is sent
     * TRUE iff ist == S (2.9.3.3); ist != S sends passive false. In
     * PPIS/PPSS every PPR is passive false. */
    if (pp->state == GPIB_PP_PPAS) {
        out->ppas_active = true;
        out->drive_line = pp->dio_line;
        out->drive_level = (pp->ist == pp->sense);
    }
}
/* --- Source Handshake (--2) ----------------------------------------------- */

void gpib_sh_init(gpib_sh_t *sh) {
    sh->state = GPIB_SH_SIDS; /* pon */
    sh->t1_deadline_us = 0;
    sh->in_flight = false;
    sh->require_dac_low = false;
}

static bool sh_loses_role(const gpib_sh_in_t *in) {
    if (in->atn) {
        return !in->cacs && !in->ctrs;
    }
    return !in->tacs && !in->spas;
}

void gpib_sh_step(gpib_sh_t *sh, const gpib_sh_in_t *in, gpib_sh_out_t *out) {
    out->dav_requested = false;
    out->may_change_byte = false;
    out->byte_accepted = false;
    out->role_lost = false;

    bool src_active = in->tacs || in->spas || in->cacs;
    bool loses = sh_loses_role(in);

    switch (sh->state) {
    case GPIB_SH_SIDS:
        if (src_active) {
            sh->state = GPIB_SH_SGNS;
        }
        break;
    case GPIB_SH_SGNS:
        if (loses) {
            sh->state = GPIB_SH_SIDS;
            out->role_lost = true;
        } else if (in->nba) {
            sh->state = GPIB_SH_SDYS;
            sh->t1_deadline_us = in->now_us + (uint64_t)in->t1_us;
        }
        break;
    case GPIB_SH_SDYS:
        if (loses) {
            sh->state = GPIB_SH_SIDS;
            out->role_lost = true;
        } else if (in->rfd && in->now_us >= sh->t1_deadline_us &&
                   (!in->dac || !sh->require_dac_low)) {
            /* Optional !DAC qualifier (Annex B step 3 error check): with
             * the policy on, refuse to transfer while RFD and DAC are
             * both TRUE (no active acceptors on the bus). */
            sh->state = GPIB_SH_STRS;
        }
        break;
    case GPIB_SH_STRS:
        if (loses) {
            /* Asynchronous interruption: the in-flight byte fate is
             * unproven and must be reported as uncertain. */
            sh->state = GPIB_SH_SIWS;
            sh->in_flight = true;
            out->role_lost = true;
        } else if (in->dac) {
            sh->state = GPIB_SH_SWNS;
            out->byte_accepted = true; /* accepted by ALL acceptors */
        }
        break;
    case GPIB_SH_SWNS:
        if (loses) {
            sh->state = GPIB_SH_SIWS;
            out->role_lost = true;
        } else if (!in->nba) {
            sh->state = GPIB_SH_SGNS;
        }
        break;
    case GPIB_SH_SIWS:
        if (!in->nba && !src_active) {
            sh->state = GPIB_SH_SIDS;
        } else if (src_active) {
            sh->state = GPIB_SH_SWNS;
        }
        break;
    default:
        break;
    }

    /* Deterministic output policy: DAV is driven only in STRS; SWNS keeps
     * DAV released (Annex B step 10: DAV high before the next cycle). */
    out->dav_requested = (sh->state == GPIB_SH_STRS);
    switch (sh->state) {
    case GPIB_SH_SIDS:
    case GPIB_SH_SGNS:
    case GPIB_SH_SWNS:
    case GPIB_SH_SIWS:
        out->may_change_byte = true;
        break;
    default:
        out->may_change_byte = false; /* SDYS/STRS: DIO/EOI held stable */
        break;
    }
}

/* --- Acceptor Handshake (--3) --------------------------------------------- */

void gpib_ah_init(gpib_ah_t *ah) {
    ah->state = GPIB_AH_AIDS; /* pon */
    ah->t3_deadline_us = 0;
}

static bool ah_not_addressed(const gpib_ah_in_t *in) {
    return !in->atn && !in->lads && !in->lacs;
}

void gpib_ah_step(gpib_ah_t *ah, const gpib_ah_in_t *in, gpib_ah_out_t *out) {
    out->rfd_true = false;
    out->dac_true = false;
    out->byte_valid = false;
    out->accepted_event = false;

    bool not_addressed = ah_not_addressed(in);

    switch (ah->state) {
    case GPIB_AH_AIDS:
        if (in->atn || in->lacs || in->lads) {
            ah->state = GPIB_AH_ANRS;
        }
        break;
    case GPIB_AH_ANRS:
        if (not_addressed) {
            ah->state = GPIB_AH_AIDS;
        } else if (!in->tcs && ((in->atn && !in->dav) || in->rdy)) {
            ah->state = GPIB_AH_ACRS;
        }
        /* Optional ANRS -> AWNS on DAV deliberately omitted ([O]). */
        break;
    case GPIB_AH_ACRS:
        if (not_addressed) {
            ah->state = GPIB_AH_AIDS;
        } else if (in->dav) {
            ah->state = GPIB_AH_ACDS;
            ah->t3_deadline_us = in->now_us + (uint64_t)in->t3_us;
            out->byte_valid = true; /* capture DIO/EOI/ATN now */
        } else if (!in->atn && !in->rdy) {
            ah->state = GPIB_AH_ANRS;
        }
        break;
    case GPIB_AH_ACDS:
        if (not_addressed) {
            ah->state = GPIB_AH_AIDS;
        } else if ((in->atn && in->now_us >= ah->t3_deadline_us) ||
                   (!in->atn && !in->rdy)) {
            ah->state = GPIB_AH_AWNS;
            out->accepted_event = true;
        }
        /* Optional ACDS -> ACRS on !DAV deliberately omitted ([O]). */
        break;
    case GPIB_AH_AWNS:
        if (not_addressed) {
            ah->state = GPIB_AH_AIDS;
        } else if (!in->dav) {
            ah->state = GPIB_AH_ANRS;
        }
        break;
    default:
        break;
    }

    /* Passive/output mapping per the baseline table. */
    switch (ah->state) {
    case GPIB_AH_AIDS:
        out->rfd_true = true;  /* passive */
        out->dac_true = true;  /* passive */
        break;
    case GPIB_AH_ANRS:
        out->rfd_true = false;
        out->dac_true = false;
        break;
    case GPIB_AH_ACRS:
        out->rfd_true = true;  /* passive */
        out->dac_true = false;
        break;
    case GPIB_AH_ACDS:
        out->rfd_true = false;
        out->dac_true = false;
        break;
    case GPIB_AH_AWNS:
        out->rfd_true = false;
        out->dac_true = true;  /* passive */
        break;
    default:
        break;
    }
}
