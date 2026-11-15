#include <string.h>

#include "gpib_internal.h"

/* Line-level engine: the core's real byte executor for line-capable ports
 * (MAILBOX/executor contracts on top of the SH/AH/decoder modules from
 * gpib_proto.h). One step = one bounded bus exchange:
 *   read snapshot -> step FSMs -> apply drive requests.
 * The mailbox action layer (PUBLISHED/ACTIVE/QUIESCED/accounting) is kept;
 * only the byte transport changes (byte-pump transfer() vs SH/AH on lines).
 *
 * Controller sequencing ([P] pending the full C-function tables):
 *   CMD phase:  ATN asserted by us, command prologue via SH as CACS
 *               (UNL, MLA targets / MTA source).
 *   DATA_TX:    ATN released, data bytes via SH as TACS, EOI on last byte.
 *   DATA_RX:    ATN released, bytes via AH with our own lads; EOI ends the
 *               stream (end cause EOI), block length ends the block
 *               (end cause LENGTH; stream logic continues).
 *   LISTEN_CMDS: an external controller stole the bus (ATN we do not
 *               drive): keep handshaking command bytes via AH and route
 *               them to the device service channel; the interrupted data
 *               action has already been terminalized with the confirmed
 *               prefix (P1 YIELD semantics). */


#define LNX_T3_US 1u   /* implementation dependent, > 0 */
/* Listener capacity is the published config knob: the submit layer
 * rejects larger lists, so the sequencer never truncates. */
#if GPIB_CFG_MAX_LISTENERS < 1
#error "GPIB_CFG_MAX_LISTENERS must be at least 1"
#endif
#if GPIB_CFG_MAX_CMDS < 1
#error "GPIB_CFG_MAX_CMDS must be at least 1"
#endif
#define LNX_MAX_LISTENERS ((size_t)GPIB_CFG_MAX_LISTENERS)

bool gpib_line_mode(const struct gpib_context *ctx) {
    return ctx->port != NULL && ctx->port->read_lines != NULL;
}

/* Reset the source handshake for a fresh byte sequence with the Annex B
 * empty-bus error check armed. Every sequencer phase MUST go through
 * this: a bare gpib_sh_init() leaves the guard off and the empty-bus
 * probe/handshake silently misjudges an absent acceptor. */
static void sh_reset(struct gpib_context *ctx) {
    gpib_sh_init(&ctx->line.sh);
    ctx->line.sh.require_dac_low = true;
}

/* R7-01: THE drive composer. Every port->drive() from this sequencer
 * flows through here: the persistent SC lines (REN level, the IFC pulse
 * window) are merged into EVERY request -- an exchange-scoped release
 * (phase boundary, stop-state, cancel) must never electrically drop
 * them, or a real port translates the one-round glitch into a physical
 * REN edge and devices fall REMOTE->LOCAL. Electrical release of ALL
 * outputs belongs to the PORT layer only (emergency_inhibit /
 * safe_detached), never to an exchange-scoped drive. */
void gpib_lineexec_drive_out(struct gpib_context *ctx,
    gpib_line_drive_t *req) {
    /* AUD-07: drive() carries the FULL state of all outputs. The
     * persistent SR service request must ride EVERY exchange-scoped
     * drive exactly like REN/IFC: a phase-local zero used to drop SRQ
     * on the wire while SR stayed in SRQS -- a lost service request. */
    req->ren = ctx->line.sc_ren;
    req->ifc = ctx->line.sc_ifc;
    req->srq = (ctx->sr.state == GPIB_SR_SRQS);
    ctx->port->drive(ctx->port->user, req);
}

static void apply_release(struct gpib_context *ctx) {
    gpib_line_drive_t req;
    memset(&req, 0, sizeof(req)); /* transfer lines deasserted; the SC
                                   * pair survives via the composer */
    gpib_lineexec_drive_out(ctx, &req);
    ctx->port->drive_data(ctx->port->user, NULL);
}

/* AUD-10 (488.2 16.2.1): the release at the END of a command action
 * (SEND COMMAND, epilogues) must NOT open an ATN=0 gap before the idle
 * drive re-asserts: ATN stays until the next sequence changes it. Data
 * phases call apply_release() (ATN legitimately falls for the DABs). */
static void apply_release_hold_atn(struct gpib_context *ctx) {
    gpib_line_drive_t req;
    memset(&req, 0, sizeof(req));
    if (ctx->line.ctrl.main == GPIB_C_CACS ||
        ctx->line.ctrl.main == GPIB_C_CTRS ||
        ctx->line.we_drive_atn) {
        req.atn = true;
    }
    gpib_lineexec_drive_out(ctx, &req);
    ctx->port->drive_data(ctx->port->user, NULL);
}

void gpib_lineexec_detach(struct gpib_context *ctx) {
    /* The OFF-BUS detach (Rev7 §R7-01 / Rev8 §T8-01): the one place the
     * sequencer releases EVERYTHING electrically, SC lines included --
     * this instance is leaving the bus, so REN/ATN must drop. Called
     * once at the STOPPING->DRAINING boundary (instance-wide quiescence
     * proven), never inside an exchange. Deliberately NOT routed through
     * drive_out: the composer's job is to PERSIST SC lines, the detach's
     * job is the opposite. Byte-pump ports have no electrical lines --
     * the drive callbacks are optional there. */
    gpib_line_drive_t req;
    memset(&req, 0, sizeof(req));
    if (ctx->port->drive != NULL) {
        ctx->port->drive(ctx->port->user, &req);
    }
    if (ctx->port->drive_data != NULL) {
        ctx->port->drive_data(ctx->port->user, NULL);
    }
}

void gpib_lineexec_release(struct gpib_context *ctx) {
    if (!gpib_line_mode(ctx)) {
        return;
    }
    ctx->line.we_drive_atn = false;
    apply_release(ctx);
}

/* AUD-10: command-action end keeps ATN (16.2.1). */
void gpib_lineexec_release_hold_atn(struct gpib_context *ctx) {
    if (!gpib_line_mode(ctx)) {
        return;
    }
    apply_release_hold_atn(ctx);
}

void gpib_lineexec_begin(struct gpib_context *ctx) {
    struct gpib_action *a = &ctx->action;
    struct gpib_txn_record *tr = &ctx->txn[a->txn_slot];
    uint8_t own = ctx->own_address.primary;
    uint8_t *c = ctx->line.cmds;
    size_t n = 0;
    /* A new action ends any RECEIVE stop-state hold (F13): the next
     * control sequence owns the bus from here. */
    ctx->line.rx_stop_hold = false;
    ctx->spas_tx_active = false; /* D01: a fresh action never inherits
                                    a phantom STB from an interrupted
                                    poll */
    if (tr->kind == GPIB_REQ_READ && ctx->line.rx_session_open &&
        ctx->line.rx_session_slot == (uint16_t)a->txn_slot &&
        ctx->line.rx_session_gen ==
            ctx->txn[a->txn_slot].generation) {
        /* F05 + C5: continuation of THIS transaction's OPEN stream (the
         * slot+generation binding is mandatory: another READ -- or a
         * cancelled and reused slot -- must run a fresh RECEIVE SETUP,
         * never resume a stranger's stream). The remote talker is
         * still addressed (TACS) and our listener state is latched. */
        ctx->line.n_post = 0;
        ctx->line.n_cmds = 0;
        ctx->line.cmd_pos = 0;
        ctx->line.rx_captured = 0;
        ctx->line.rx_eoi = false;
        ctx->line.rx_holdoff = 0;
        ctx->line.stop_char_hit = false;
        ctx->line.seq = (uint8_t)GPIB_LSEQ_DATA_RX;
        ctx->line.we_drive_atn = false;
        ctx->line.rx_session_open = false; /* re-armed at block finish */
        return;
    }

    sh_reset(ctx);
    gpib_ah_init(&ctx->line.ah);
    ctx->line.n_post = 0;
    ctx->line.post_pos = 0;
    if (tr->kind == GPIB_REQ_DEV_TX || tr->kind == GPIB_REQ_DEV_RX) {
        /* Device role: no controller prologue; wait for the external
         * controller to address us, then exchange on the same SH/AH data
         * phases. */
        ctx->line.seq = (uint8_t)GPIB_LSEQ_DEV_WAIT;
        ctx->line.we_drive_atn = false;
        return;
    }
    if (tr->kind == GPIB_REQ_TCT) {
        /* Pass control: address the target controller as talker, then TCT.
         * No data phase: the action completes when the prologue is sent. */
        c[n++] = gpib_mk_mta(tr->address.primary); /* MTA(target) */
        c[n++] = GPIB_TCT_BYTE;
        ctx->line.n_cmds = n;
        ctx->line.cmd_pos = 0;
        ctx->line.send_tct = true;
        ctx->line.seq = (uint8_t)GPIB_LSEQ_CMD;
        ctx->line.we_drive_atn = true;
        /* No CIDS -> CADS forcing here: CIC authority comes ONLY from the
         * controller FSM (power-on IFC for the SC, TCT reception for
         * others). The dispatcher gates on CIC before publishing. */
        return;
    }
    if (tr->kind == GPIB_REQ_PPOLL) {
        /* PERFORM PARALLEL POLL (16.15): ATN true + EOI true (IDY),
         * wait T6, read PPR lines, release. No command prologue. */
        ctx->line.n_post = 0;
        ctx->line.n_cmds = 0;
        ctx->line.cmd_pos = 0;
        ctx->line.idy_rounds = 0;
        ctx->line.seq = (uint8_t)GPIB_LSEQ_PPOLL_IDY;
        ctx->line.we_drive_atn = true;
        return;
    }
    if (tr->kind == GPIB_REQ_FINDLSTN) {
        /* FINDLSTN (17.6): probe each candidate by MLA(addr) then
         * observing NDAC. No prologue; the sequencer runs probes. */
        ctx->line.n_post = 0;
        ctx->line.n_cmds = 0;
        ctx->line.seq = (uint8_t)GPIB_LSEQ_FINDLSTN_PROBE;
        ctx->line.we_drive_atn = true;
        ctx->line.cmd_pos = 0;
        return;
    }
    if (tr->kind == GPIB_REQ_CMDS) {
        /* SEND COMMAND (488.2 16.2.1): the user's ATN byte list is the
         * whole prologue; no data phase. */
        for (size_t i = 0; i < tr->cmd_len && n < sizeof(ctx->line.cmds);
             i++) {
            c[n++] = tr->cmd_stored[i]; /* DIO8 already masked in submit */
        }
        ctx->line.n_cmds = n;
        ctx->line.cmd_pos = 0;
        ctx->line.seq = (uint8_t)GPIB_LSEQ_CMD;
        ctx->line.we_drive_atn = true;
        /* No CIC self-promotion: see the TCT branch above. */
        return;
    }
    if (tr->kind == GPIB_REQ_ALLSPOLL || tr->kind == GPIB_REQ_FINDRQS) {
        /* ALLSPOLL (17.3) / FINDRQS (17.2): UNL, MLA(controller), SPE
         * armed once; per address MTA + one STB read (FINDRQS stops at
         * the first RQS); epilogue SPD, UNT. */
        c[n++] = GPIB_UNL_BYTE; /* UNL */
        c[n++] = gpib_mk_mla(own); /* MLA(controller) */
        c[n++] = GPIB_SPE_BYTE; /* SPE */
        ctx->line.post[0] = GPIB_SPD_BYTE; /* SPD */
        ctx->line.post[1] = GPIB_UNT_BYTE; /* UNT */
        ctx->line.n_post = 2;
        /* After the prologue, the sequencer enters POLL_ADDR (not data). */
    } else if (tr->kind == GPIB_REQ_RESET) {
        /* RESET (17.1) level 2: DCL right after the (empty) addressing
         * prologue; level 3 (*RST per address) runs as the data phase. */
        c[n++] = 0x14u; /* DCL */
    } else if (tr->kind == GPIB_REQ_SPOLL) {
        /* READ STATUS BYTE (488.2 16.2.18): UNL, MLA(controller), SPE,
         * MTA(device)[, SAD]; epilogue SPD, UNT. AUD-12: the secondary
         * address form -- an extended-address device is polled through
         * its full MTA+SAD pair (16.2.18 "if the device is addressed
         * by an extended address, the controller sends the secondary
         * address as well"). */
        c[n++] = GPIB_UNL_BYTE; /* UNL */
        c[n++] = gpib_mk_mla(own); /* MLA(controller) */
        c[n++] = GPIB_SPE_BYTE; /* SPE: serial poll enable */
        c[n++] = gpib_mk_mta(tr->address.primary); /* MTA(device) */
        if (tr->address.secondary_present &&
            tr->address.secondary <= 31u) {
            c[n++] = gpib_mk_sad(tr->address.secondary);
        }
        ctx->line.post[0] = GPIB_SPD_BYTE; /* SPD: serial poll disable */
        ctx->line.post[1] = GPIB_UNT_BYTE; /* UNT */
        ctx->line.n_post = 2;
    } else {
        if (tr->kind == GPIB_REQ_READ) {
            /* RECEIVE SETUP (488.2 16.2.5): UNL, MLA(controller), MTA.
             * LE: append SAD(secondary) after MTA for secondary addressing. */
            c[n++] = GPIB_UNL_BYTE; /* UNL */
            c[n++] = gpib_mk_mla(own);       /* MLA(controller) */
            c[n++] = gpib_mk_mta(tr->address.primary); /* MTA(device) */
            if (tr->address.secondary_present) {
                c[n++] = gpib_mk_sad(tr->address.secondary);
            }
        } else {
            /* SEND SETUP (488.2 16.2.2): MTA(controller), UNL, MLAs ---
             * MTA first so the following UNL cannot unaddress us. */
            uint8_t mlaprim[LNX_MAX_LISTENERS];
            uint8_t mlasec[LNX_MAX_LISTENERS];
            bool mlahassec[LNX_MAX_LISTENERS];
            size_t count = 1;
            size_t pos = 0;
            if (tr->kind == GPIB_REQ_WRITE_MANY) {
                count = (tr->listener_count < LNX_MAX_LISTENERS)
                            ? tr->listener_count
                            : LNX_MAX_LISTENERS;
            }
            for (size_t i = 0; i < count; i++) {
                uint8_t prim = tr->address.primary;
                bool hassec = tr->address.secondary_present;
                uint8_t sec = tr->address.secondary;
                if (tr->kind == GPIB_REQ_WRITE_MANY &&
                    tr->listeners_first != NULL) {
                    prim = tr->listeners_first[i].primary;
                    hassec = tr->listeners_first[i].secondary_present;
                    sec = tr->listeners_first[i].secondary;
                }
                mlaprim[pos] = gpib_mk_mla(prim);
                mlahassec[pos] = hassec;
                mlasec[pos] = sec;
                pos++;
            }
            c[n++] = gpib_mk_mta(own); /* MTA(controller talks) */
            c[n++] = GPIB_UNL_BYTE; /* UNL */
            for (size_t i = 0; i < pos; i++) {
                c[n++] = mlaprim[i];
                if (mlahassec[i]) {
                    c[n++] = gpib_mk_sad(mlasec[i]);
                }
            }
        }
    }
    ctx->line.n_cmds = n;
    ctx->line.cmd_pos = 0;
    ctx->line.tx_pos = 0;
    ctx->line.rx_captured = 0;
    ctx->line.rx_eoi = false;
    ctx->line.rx_holdoff = 0;
    ctx->line.stop_char_hit = false;
    ctx->line.seq = (uint8_t)GPIB_LSEQ_CMD;
    ctx->line.we_drive_atn = true;
}

static void finish_action(struct gpib_context *ctx, gpib_term_reason_t cause,
    bool eoi) {
    struct gpib_action *a = &ctx->action;
    a->stop_cause = (uint8_t)cause;
    a->eoi_terminated = eoi;
    a->last_byte_uncertain = ctx->line.sh.in_flight;
    a->state = (uint8_t)GPIB_ACTION_QUIESCED;
    ctx->line.seq = (uint8_t)GPIB_LSEQ_IDLE;
    /* F05: a READ block that filled its capacity WITHOUT the stream end
     * (no EOI, no stop character) is a BLOCK boundary, not a RECEIVE
     * end: keep the session -- the addressing stays latched in the
     * remote T/L functions and the acceptor holds NRFD (backpressure,
     * REQ-026/035). The next block resumes DATA_RX with no prologue;
     * 488.2 16.2.5 allows exactly one RECEIVE per response message. */
    bool session_kept = false;
    bool stop_state = false;
    if (cause == GPIB_TERM_COMPLETED &&
        ctx->txn[a->txn_slot].kind == GPIB_REQ_READ &&
        !a->eoi_terminated && !a->char_terminated &&
        gpib_line_mode(ctx)) {
        ctx->line.rx_session_open = true;
        ctx->line.rx_session_slot = (uint16_t)a->txn_slot;
        ctx->line.rx_session_gen = ctx->txn[a->txn_slot].generation;
        session_kept = true;
        /* no release: the idle drive holds NRFD/NDAC via the flag */
    } else if (cause == GPIB_TERM_COMPLETED &&
               (ctx->txn[a->txn_slot].kind == GPIB_REQ_READ ||
                ctx->txn[a->txn_slot].kind == GPIB_REQ_DEV_RX) &&
               gpib_line_mode(ctx)) {
        /* R6-02/R9 (found by the oracle's DIO-conflict invariant): the
         * terminal RECEIVE enters the stop state through the ACCEPTOR'S
         * OWN cycle -- the AWNS round emits the NDAC accept PULSE the
         * source needs (NRFD held), the next round asserts the ANRS
         * hold, and poll_idle sustains it. Driving the hold from HERE
         * used to clobber the accept pulse in the same round: the
         * source never saw its last byte accepted and kept its DIO
         * drivers forever (a real talker hangs in SWNS on hardware).
         * A03: the same AWNS tail applies to the DEVICE sink -- an
         * addressed listener must keep NRFD=true/NDAC released until
         * the external talker drops DAV (488.1 2.4.3.5); the generic
         * release revoked the accept pulse immediately. */
        stop_state = true;
        ctx->line.rx_stop_hold = true;
        ctx->line.we_drive_atn = false;
        /* no release, no immediate drive: the AH's final AWNS/ANRS
         * drive provides the continuity; the idle drive takes over. */
    }
    if (!session_kept && !stop_state) {
        if (cause == GPIB_TERM_COMPLETED && ctx->line.we_drive_atn &&
            gpib_line_mode(ctx) &&
            ctx->line.ctrl.main != GPIB_C_CIDS) {
            /* AUD-10: a command-action completing with the bus held:
             * ATN must NOT gap to zero before the idle drive. */
            gpib_lineexec_release_hold_atn(ctx);
        } else {
            gpib_lineexec_release(ctx);
        }
    }
    /* F13 (488.2 16.2.6): a TERMINAL RECEIVE leaves the acceptor in the
     * stop state -- NRFD and NDAC held TRUE (electrically LOW) until the
     * next control sequence takes the bus (ATN) or a new action begins.
     * poll_idle's idle drive materializes the hold on the wire. */
    if (cause == GPIB_TERM_COMPLETED &&
        ctx->txn[a->txn_slot].kind == GPIB_REQ_READ && !session_kept) {
        ctx->line.rx_stop_hold = true;
    }
    if (!session_kept) {
        ctx->line.rx_session_open = false; /* any terminal path closes it */
    }
}
static void feed_self_addressing(struct gpib_context *ctx, uint8_t raw) {
    /* Our own ATN sequences address US too: a controller that sends
     * MTA(own) must land its talker in TADS (this is how a TCT takeover
     * through SEND COMMAND works, O18). The T/L pair steps on the
     * accepted byte exactly like the external path in dev_wait. */
    gpib_ifcmd_t cmd = gpib_decode_atn_byte(raw);
    gpib_addr_step(&ctx->line.addr, &cmd);
    {
        gpib_t_in_t tin;
        gpib_t_out_t tout;
        memset(&tin, 0, sizeof(tin));
        tin.atn = true;
        tin.cmd_valid = true;
        tin.cmd = cmd;
        gpib_talker_step(&ctx->line.tk, &tin, &tout);
        (void)tout;
    }
    {
        gpib_l_in_t lin;
        gpib_l_out_t lout;
        memset(&lin, 0, sizeof(lin));
        lin.atn = true;
        lin.cmd_valid = true;
        lin.cmd = cmd;
        gpib_listener_step(&ctx->line.ln, &lin, &lout);
        (void)lout;
    }
    /* F16 gate for SELF-sent sequences (O18): another controller (us,
     * addressing ourselves before a takeover) passes control via TCT --
     * legitimate ONLY when our talker is addressed. A stray self-sent
     * 0x09 must not hand us the bus. */
    if (cmd.cls == GPIB_CMD_TCT &&
        (ctx->line.tk.state == GPIB_T_TADS ||
         ctx->line.tk.state == GPIB_T_TACS)) {
        ctx->line.ctrl_tct_pulse = true;
    }
}

static gpib_request_kind_t tr_kind(struct gpib_context *ctx) {
    return ctx->txn[ctx->action.txn_slot].kind;
}

/* SH-driven phase (CMD prologue or DATA_TX). */
static void sh_phase(struct gpib_context *ctx, const gpib_line_snap_t *snap,
    bool cacs_role, const uint8_t *bytes, size_t len, size_t *pos,
    bool eoi_last) {
    struct gpib_action *a = &ctx->action;
    gpib_sh_in_t in;
    gpib_sh_out_t out;
    gpib_line_drive_t req;
    memset(&in, 0, sizeof(in));
    in.atn = snap->atn;
    in.cacs = cacs_role;
    in.tacs = !cacs_role && !snap->atn;
    in.nba = (ctx->line.sh.state == GPIB_SH_SGNS) && (*pos < len);
    in.rfd = !snap->nrfd;
    in.dac = !snap->ndac;
    in.now_us = snap->now_us;
    in.t1_us = GPIB_CFG_T1_SOURCE_US;
    gpib_sh_step(&ctx->line.sh, &in, &out);
    if (out.byte_accepted) {
        (*pos)++;
        if (ctx->line.seq == (uint8_t)GPIB_LSEQ_CMD ||
            ctx->line.seq == (uint8_t)GPIB_LSEQ_EPILOGUE) {
            feed_self_addressing(ctx, bytes[*pos - 1u]);
        } else {
            a->committed = *pos;
        }
    }
    memset(&req, 0, sizeof(req));
    req.dav = out.dav_requested;
    req.atn = cacs_role; /* we hold ATN through the command phase */
    req.eoi = eoi_last && !cacs_role && (*pos + 1u == len) &&
              (ctx->line.sh.state == GPIB_SH_SGNS ||
               ctx->line.sh.state == GPIB_SH_SDYS ||
               ctx->line.sh.state == GPIB_SH_STRS);
    req.ifc = ctx->line.sc_ifc; /* SC lines merged into every drive */
    req.ren = ctx->line.sc_ren;
    gpib_lineexec_drive_out(ctx, &req);
    if (ctx->line.sh.state == GPIB_SH_SGNS && *pos < len) {
        ctx->port->drive_data(ctx->port->user, &bytes[*pos]);
    }
    /* Phase completion. */
    if (ctx->line.seq == (uint8_t)GPIB_LSEQ_CMD) {
        if (tr_kind(ctx) == GPIB_REQ_TCT) {
            if (ctx->line.cmd_pos == ctx->line.n_cmds &&
                ctx->line.sh.state == GPIB_SH_SWNS) {
                /* TCT byte completed: CTRS -> CIDS happens in the FSM step;
                 * the action itself is done. */
                ctx->line.send_tct = false;
                finish_action(ctx, GPIB_TERM_COMPLETED, false);
            }
            return;
        }
        if (tr_kind(ctx) == GPIB_REQ_CMDS) {
            if (ctx->line.cmd_pos == ctx->line.n_cmds &&
                ctx->line.sh.state == GPIB_SH_SWNS) {
                /* SEND COMMAND: the ATN byte list is done. */
                finish_action(ctx, GPIB_TERM_COMPLETED, false);
            }
            return;
        }
        if (ctx->line.cmd_pos == ctx->line.n_cmds &&
            ctx->line.sh.state == GPIB_SH_SWNS) {
            struct gpib_txn_record *tr2 = &ctx->txn[a->txn_slot];
            if (tr_kind(ctx) == GPIB_REQ_RESET) {
                /* Level 2 done (DCL sent); level 3: address each device
                 * in turn and send *RST<NL> with END. */
                if (tr2->poll_index < tr2->poll_count) {
                    /* Full SEND SETUP per address (16.2.2): MTA(ctrl),
                     * UNL, MLA(addr) - the previous listener must not
                     * hear the next device's *RST. */
                    ctx->line.cmds[0] = gpib_mk_mta(ctx->own_address.primary);
                    ctx->line.cmds[1] = GPIB_UNL_BYTE;
                    ctx->line.cmds[2] = (uint8_t)(0x20u |
                        tr2->poll_addrs[tr2->poll_index]);
                    ctx->line.n_cmds = 3;
                    ctx->line.cmd_pos = 0;
                    ctx->line.tx_pos = 0;
                    a->data = tr2->rst_msg;
                    a->len = 5; /* *RST\n */
                    a->limit = 5;
                    a->committed = 0;
                    sh_reset(ctx);
                    ctx->line.seq = (uint8_t)GPIB_LSEQ_DATA_TX_PENDING;
                    return;
                }
                finish_action(ctx, GPIB_TERM_COMPLETED, false);
                return;
            }
            /* Prologue done: release ATN NOW so the first data step does
             * not mistake our own ATN for an external steal, then switch
             * to the data phase. */
            ctx->line.seq = (tr_kind(ctx) == GPIB_REQ_ALLSPOLL ||
                             tr_kind(ctx) == GPIB_REQ_FINDRQS)
                                ? (uint8_t)GPIB_LSEQ_POLL_ADDR
                                : (a->kind == (uint8_t)GPIB_ACTION_RX_BLOCK)
                                      ? (uint8_t)GPIB_LSEQ_DATA_RX
                                      : (uint8_t)GPIB_LSEQ_DATA_TX;
            ctx->line.we_drive_atn = false;
            /* Release ATN AND the DIO drivers: the instrument is about to
             * drive the data lines; a stale command byte on them is a
             * two-driver contention. */
            apply_release(ctx);
            sh_reset(ctx);
            ctx->line.tx_pos = 0;
            if (ctx->line.seq == (uint8_t)GPIB_LSEQ_POLL_ADDR) {
                /* ALLSPOLL/FINDRQS: the per-address MTA reuses cmd_pos as
                 * its own cursor --- restart it for the first address. */
                ctx->line.cmd_pos = 0;
                ctx->line.we_drive_atn = true;
            }
        }
        return;
    }
    if (ctx->line.seq == (uint8_t)GPIB_LSEQ_EPILOGUE) {
        if (ctx->line.post_pos == ctx->line.n_post &&
            ctx->line.sh.state == GPIB_SH_SWNS) {
            finish_action(ctx, GPIB_TERM_COMPLETED, false);
        }
        return;
    }
    if (ctx->line.sh.state == GPIB_SH_SWNS && *pos >= len &&
        ctx->line.seq == (uint8_t)GPIB_LSEQ_DATA_TX) {
        if (tr_kind(ctx) == GPIB_REQ_RESET) {
            /* *RST delivered to this address: the next address (if any)
             * restarts from its MLA; otherwise the protocol is done. */
            struct gpib_txn_record *tr3 = &ctx->txn[ctx->action.txn_slot];
            tr3->poll_index++;
            if (tr3->poll_index < tr3->poll_count) {
                ctx->line.cmds[0] = gpib_mk_mta(ctx->own_address.primary);
                ctx->line.cmds[1] = GPIB_UNL_BYTE;
                ctx->line.cmds[2] = (uint8_t)(0x20u |
                    tr3->poll_addrs[tr3->poll_index]);
                ctx->line.n_cmds = 3;
                ctx->line.cmd_pos = 0;
                ctx->line.tx_pos = 0;
                a->committed = 0;
                ctx->line.seq = (uint8_t)GPIB_LSEQ_DATA_TX_PENDING;
                ctx->line.we_drive_atn = true;
                sh_reset(ctx);
                return;
            }
            finish_action(ctx, GPIB_TERM_COMPLETED, false);
            return;
        }
        finish_action(ctx, GPIB_TERM_COMPLETED, false);
    }
}

/* AH-driven phase (DATA_RX or LISTEN_CMDS). */
static void ah_phase(struct gpib_context *ctx, const gpib_line_snap_t *snap,
    bool listen_cmds) {
    struct gpib_action *a = &ctx->action;
    gpib_ah_in_t in;
    gpib_ah_out_t out;
    gpib_line_drive_t req;
    memset(&req, 0, sizeof(req)); /* T03: EVERY drive call must carry
                                   * fully-defined fields -- the early
                                   * STB-ack below fires before the main
                                   * fill and used to read uninitialized
                                   * bools (UB, build-dependent). */
    memset(&in, 0, sizeof(in));
    in.atn = snap->atn;
    in.dav = snap->dav;
    in.lads = ctx->line.addr.lads;
    in.lacs = ctx->line.addr.lads;
    if (!listen_cmds && ctx->line.n_post > 0) {
        /* Serial poll data phase: the controller listens via its local
         * listen request (ltn semantics) without an explicit MLA. */
        in.lads = true;
        in.lacs = true;
    }
    if (listen_cmds) {
        in.rdy = false; /* command bytes accept via T3 */
    } else {
        /* Consumer readiness: the mandatory !rdy accept window after each
         * capture, then ready while buffer room remains. */
        if (ctx->line.rx_holdoff > 0) {
            in.rdy = false;
        } else {
            in.rdy = (a->len - ctx->line.rx_captured) > 0;
        }
    }
    in.now_us = snap->now_us;
    in.t3_us = LNX_T3_US;
    gpib_ah_step(&ctx->line.ah, &in, &out);
    if (out.byte_valid) {
        if (listen_cmds) {
            /* External controller's command byte: route to the service
             * channel and self-addressing; never part of any payload. */
            gpib_ifcmd_t cmd = gpib_decode_atn_byte(snap->dio);
            gpib_addr_step(&ctx->line.addr, &cmd);
            ctx->line.last_cmd = snap->dio;
            ctx->line.last_cmd_valid = true;
            ctx->dev_evt[ctx->dev_evt_head] = snap->dio;
            ctx->dev_evt_head =
                (ctx->dev_evt_head + 1u) % GPIB_CFG_DEVICE_EVENTS;
            if (ctx->dev_evt_count < GPIB_CFG_DEVICE_EVENTS) {
                ctx->dev_evt_count++;
            } else {
                ctx->dev_evt_dropped++;
            }
            GPIB_TRACE(ctx, GPIB_TRC_DEVICE_CMD, 0, 0, snap->dio);
        } else {
            ((uint8_t *)(uintptr_t)a->data)[ctx->line.rx_captured] = snap->dio;
            ctx->line.rx_captured++;
            ctx->line.rx_holdoff = 2; /* mandatory !rdy accept window */
            if (snap->eoi) {
                ctx->line.rx_eoi = true;
            }
            /* RECEIVE stop-character (488.2 16.2.6.2-4): the stop byte is
             * part of the received data; matching ends the message. */
            if (!listen_cmds && !ctx->line.stop_char_hit) {
                struct gpib_txn_record *trx = &ctx->txn[a->txn_slot];
                uint8_t sc = 0;
                bool armed = (trx->kind == GPIB_REQ_READ ||
                              trx->kind == GPIB_REQ_DEV_RX);
                if (armed) {
                    switch ((gpib_stop_cond_t)trx->stop_cond) {
                    case GPIB_STOP_NL:
                        sc = 0x0Au;
                        break;
                    case GPIB_STOP_COMMA:
                        sc = 0x2Cu;
                        break;
                    case GPIB_STOP_SEMICOLON:
                        sc = 0x3Bu;
                        break;
                    case GPIB_STOP_DAB:
                        sc = trx->stop_dab; /* arbitrary byte match */
                        break;
                    default:
                        sc = 0; /* EOI-only mode */
                        break;
                    }
                    if (sc != 0 && snap->dio == sc) {
                        ctx->line.stop_char_hit = true;
                    } else if (sc == 0 && trx->stop_cond ==
                               (uint8_t)GPIB_STOP_DAB &&
                               snap->dio == trx->stop_dab) {
                        ctx->line.stop_char_hit = true;
                    }
                }
            }
        }
    }
    if (!listen_cmds && out.accepted_event) {
        a->committed = ctx->line.rx_captured;
        if (ctx->line.rx_holdoff > 0) {
            ctx->line.rx_holdoff--;
        }
        /* R9 (oracle DIO-conflict invariant): the acceptor's OWN drive
         * for THIS round is the AWNS accept PULSE (NRFD held, NDAC
         * released) -- it MUST reach the wire before any terminal path
         * returns: the source of the just-accepted byte is waiting for
         * exactly this NDAC release; skipping it left the source stuck
         * in SWNS holding its DIO drivers forever. */
        {
            gpib_line_drive_t pulse;
            memset(&pulse, 0, sizeof(pulse));
            pulse.nrfd = !out.rfd_true;
            pulse.ndac = !out.dac_true;
            gpib_lineexec_drive_out(ctx, &pulse);
        }
        if (ctx->line.rx_eoi) {
            gpib_request_kind_t k = tr_kind(ctx);
            if (k == GPIB_REQ_SPOLL || k == GPIB_REQ_ALLSPOLL ||
                k == GPIB_REQ_FINDRQS) {
                /* 488.2 16.2.6: "handshake one DAB (STB & RQS), store
                 * it, ignore EOI" -- a device MAY assert EOI with its
                 * status byte; the poll must NOT treat it as END. The
                 * epilogue (SPD/UNT) still runs (found by the T04
                 * independent oracle: the mock never EOI'd its STB, so
                 * this path was invisible to the FSM-shared tests). */
                ctx->line.rx_eoi = false;
            } else {
                finish_action(ctx, GPIB_TERM_COMPLETED, true);
                return;
            }
        }
        if (ctx->line.stop_char_hit) {
            struct gpib_txn_record *trx = &ctx->txn[a->txn_slot];
            if (trx->kind == GPIB_REQ_ALLSPOLL ||
                trx->kind == GPIB_REQ_FINDRQS) {
                /* Poll passes carry STBs, not messages: a stray match must
                 * not terminate the pass. */
                ctx->line.stop_char_hit = false;
            } else {
                a->char_terminated = true;
                finish_action(ctx, GPIB_TERM_COMPLETED, false);
                return;
            }
        }
        if (a->committed >= a->limit) {
            gpib_request_kind_t k = tr_kind(ctx);
            if (k == GPIB_REQ_ALLSPOLL || k == GPIB_REQ_FINDRQS) {
                /* Poll mid-pass: acknowledge this STB to the source and
                 * advance. FINDRQS stops at the first RQS (bit 6). */
                memset(&ctx->req, 0, sizeof(ctx->req));
                ctx->req.nrfd = !out.rfd_true;
                ctx->req.ndac = !out.dac_true;
                gpib_lineexec_drive_out(ctx, &ctx->req);
                if (k == GPIB_REQ_FINDRQS &&
                    (ctx->txn[a->txn_slot].poll_results
                         [ctx->txn[a->txn_slot].poll_index] &
                     0x40u) != 0u) {
                    /* Requesting device found: close the poll (SPD, UNT). */
                    ctx->txn[a->txn_slot].find_index =
                        ctx->txn[a->txn_slot].poll_index;
                    ctx->line.seq = (uint8_t)GPIB_LSEQ_EPILOGUE;
                    ctx->line.post_pos = 0;
                    ctx->line.we_drive_atn = true;
                    sh_reset(ctx);
                    return;
                }
                ctx->txn[a->txn_slot].poll_index++;
                if (ctx->txn[a->txn_slot].poll_index >=
                    ctx->txn[a->txn_slot].poll_count) {
                    /* List exhausted: epilogue from POLL_ADDR. */
                    ctx->line.seq = (uint8_t)GPIB_LSEQ_POLL_ADDR;
                    ctx->line.we_drive_atn = true;
                    sh_reset(ctx);
                    return;
                }
                ctx->line.seq = (uint8_t)GPIB_LSEQ_POLL_ADDR;
                ctx->line.cmd_pos = 0;
                ctx->line.we_drive_atn = true;
                sh_reset(ctx);
                return;
            }
            if (ctx->line.n_post > 0) {
                /* Serial poll: the STB is in; acknowledge it to the source
                 * (release NDAC) BEFORE the epilogue re-asserts ATN --- the
                 * instrument's SH must see DAC to finish its byte. */
                {
                    req.nrfd = !out.rfd_true;
                    req.ndac = !out.dac_true;
                    gpib_lineexec_drive_out(ctx, &req);
                }
                ctx->line.seq = (uint8_t)GPIB_LSEQ_EPILOGUE;
                ctx->line.post_pos = 0;
                ctx->line.we_drive_atn = true;
                sh_reset(ctx);
                return;
            }
            finish_action(ctx, GPIB_TERM_COMPLETED, false);
            return;
        }
    } else if (!listen_cmds && ctx->line.rx_holdoff > 0) {
        ctx->line.rx_holdoff--;
    }
    if (listen_cmds && !snap->atn) {
        /* The external controller released ATN: bus is ours again. */
        ctx->line.seq = (uint8_t)GPIB_LSEQ_IDLE;
        gpib_ah_init(&ctx->line.ah);
        apply_release(ctx);
        return;
    }
    memset(&req, 0, sizeof(req));
    req.nrfd = !out.rfd_true;
    req.ndac = !out.dac_true;
    req.ifc = ctx->line.sc_ifc;
    req.ren = ctx->line.sc_ren;
    gpib_lineexec_drive_out(ctx, &req);
}

/* Push one device-function event (DCAS/DTAS/RL change). Overflow is a
 * mandatory-delivery VIOLATION (RL/DC/DT extract 4.5: never silently
 * drop): latch a fault, keep counting for the snapshot evidence. */
static void df_push(struct gpib_context *ctx, gpib_df_event_kind_t kind,
    uint8_t origin, uint8_t rl_state, uint64_t now_us) {
    ctx->df_evt[ctx->df_evt_head].kind = (uint8_t)kind;
    ctx->df_evt[ctx->df_evt_head].origin = origin;
    ctx->df_evt[ctx->df_evt_head].rl_state = rl_state;
    ctx->df_evt_head = (ctx->df_evt_head + 1u) % GPIB_CFG_DF_EVENTS;
    if (ctx->df_evt_count < GPIB_CFG_DF_EVENTS) {
        ctx->df_evt_count++;
        return;
    }
    ctx->df_evt_dropped++;
    if (!ctx->fault_latched) {
        ctx->fault_latched = true;
        ctx->fault_code = GPIB_FAULT_DF_EVENT_LOST;
        ctx->first_fault_us = now_us;
        ctx->last_update_us = now_us;
    }
}

/* Feed the RL/DC/DT functions with the current bus facts. Called with the
 * accepted command byte (cmd_valid) and once per idle/data round with
 * cmd_valid == false (ACDS window closed, REN observation continues). */
static void df_step(struct gpib_context *ctx, const gpib_line_snap_t *snap,
    bool cmd_valid, gpib_ifcmd_t cmd, bool lads) {
    gpib_rl_in_t rin;
    gpib_rl_out_t rout;
    gpib_dc_in_t din;
    gpib_dc_out_t dout;
    gpib_dt_in_t tin;
    gpib_dt_out_t tout;

    memset(&rin, 0, sizeof(rin));
    rin.ren = snap->ren;
    rin.rtl = ctx->rtl_pulse;
    rin.lads = lads;
    rin.cmd_valid = cmd_valid;
    rin.cmd = cmd;
    ctx->rtl_pulse = false; /* one-shot consumption */
    gpib_rl_step(&ctx->rl, &rin, &rout);
    if (rout.changed) {
        df_push(ctx, GPIB_DF_REMOTE_STATE, 0, (uint8_t)rout.entered,
            snap->now_us);
    }

    memset(&din, 0, sizeof(din));
    din.acds = cmd_valid;
    din.dcl = cmd_valid && cmd.cls == GPIB_CMD_DCL;
    din.sdc = cmd_valid && cmd.cls == GPIB_CMD_SDC;
    din.lads = lads;
    gpib_dc_step(&ctx->dc, &din, &dout);
    if (dout.clear_pulse) {
        df_push(ctx, GPIB_DF_CLEAR,
            din.dcl ? 0x14u : 0x04u, 0, snap->now_us);
    }

    memset(&tin, 0, sizeof(tin));
    tin.get = cmd_valid && cmd.cls == GPIB_CMD_GET;
    tin.lads = lads;
    tin.acds = cmd_valid;
    gpib_dt_step(&ctx->dt, &tin, &tout);
    if (tout.trigger_pulse) {
        df_push(ctx, GPIB_DF_TRIGGER, 0, 0, snap->now_us);
    }
}

/* Controller function groups: stepped once per bus exchange with the
 * observed facts; TCT reception pulses CIDS -> CADS on a controller-role
 * core whose talker is addressed ([P] wiring of the C FSM). An active
 * controller with nothing dispatched holds ATN (its bus, per CACS). */
static void ctrl_fsm_step(struct gpib_context *ctx,
    const gpib_line_snap_t *snap) {
    gpib_c_in_t in;
    gpib_c_out_t out;
    memset(&in, 0, sizeof(in));
    in.atn = snap->atn;
    in.ifc = snap->ifc;
    in.srq = snap->srq;
    in.tads = ctx->line.addr.tads || ctx->line.tk.state == GPIB_T_TADS ||
              ctx->line.tk.state == GPIB_T_TACS;
    in.anrs = ctx->line.ah.state == GPIB_AH_ANRS;
    in.sdys = ctx->line.sh.state == GPIB_SH_SDYS;
    in.strs = ctx->line.sh.state == GPIB_SH_STRS;
    /* F03: send_tct is a LOCAL REQUEST for THIS byte, not for the whole
     * operation -- gating on the last prologue byte keeps the C model in
     * CACS while the MTA(target) half is still on the wire, so CIC is
     * handed over only when the TCT byte itself is being sourced. */
    in.send_tct = ctx->line.send_tct &&
                  ctx->line.cmd_pos + 1u == ctx->line.n_cmds;
    in.now_us = snap->now_us;
    in.t6_us = 2u;
    in.t7_us = 2u;
    in.t8_us = GPIB_CFG_T8_STRICT_US; /* strictly > 100 us (F14) */
    in.t9_us = 2u;
    in.t10_us = 2u;
    /* F02: local control requests derived from the SEQUENCER PHASE, so
     * the C model tracks the wire the sequencer already drives.
     * gts: a data phase released ATN (CACS -> CSBS standby).
     * tca: we re-assert ATN (standby -> CSWS -> CAWS -> CACS walk; the
     *      synchronous tcs form needs an AH acceptor sync the host
     *      sequencer does not run between sequences).
     * rpp: the parallel poll IDY window (CACS -> CPWS -> CPPS). */
    in.gts = (ctx->line.seq == (uint8_t)GPIB_LSEQ_DATA_TX ||
              ctx->line.seq == (uint8_t)GPIB_LSEQ_DATA_RX) &&
             !ctx->line.we_drive_atn;
    in.tca = ctx->line.we_drive_atn;
    in.rpp = ctx->line.seq == (uint8_t)GPIB_LSEQ_PPOLL_IDY;
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    in.tct_received = ctx->line.ctrl_tct_pulse;
#endif
    ctx->line.ctrl_tct_pulse = false;
    /* System controller authority (C1) is static; the IFC/REN driver
     * groups take their local requests from the API surfaces. */
    in.rsc = ctx->system_controller;
    in.sic = ctx->ifc_request;
    in.sre = ctx->ren_request;
    gpib_controller_step(&ctx->line.ctrl, &in, &out);
    if (out.cic) {
        ctx->line.cic_once = true; /* ever held CIC: TCT-away is sticky */
    }
    if (ctx->line.ctrl.ifc == GPIB_C_SIAS) {
        /* One-shot pulse request: consumed once the driver group entered
         * SIAS; the mandatory T8 hold is the FSM's own timer. */
        ctx->ifc_request = false;
    }
    /* Publish the SC line state for the sequencer's drive calls to OR in;
     * the CIC ATN-hold also merges here when idle. */
    ctx->line.sc_ifc = out.ifc_assert;
    ctx->line.sc_ren = out.ren_assert;
}

/* Forward: the idle poll reuses the device observation step. */
static void dev_wait_step(struct gpib_context *ctx,
    const gpib_line_snap_t *snap);

static bool external_ifc_reset(struct gpib_context *ctx,
    const gpib_line_snap_t *snap);

/* SR service request evaluation (F18): runs on EVERY bus round -- idle
 * observation, command phases and active data exchanges alike. rsv is
 * the level input from the 488.2 status model (or the application);
 * SPAS comes from the talker pair. One call per round, never more. */
static void sr_round_eval(struct gpib_context *ctx) {
    gpib_sr_in_t sin;
    gpib_sr_out_t sout;
    sin.rsv = ctx->device_rsv;
    sin.spas = (ctx->line.tk.state == GPIB_T_SPAS);
    gpib_sr_step(&ctx->sr, &sin, &sout);
    (void)sout;
}

/* Device PP1 wire output (F04): in PPAS with ist == S drive the assigned
 * PPR line onto DIO; when the window closes, release OUR drivers again
 * (never touch DIO we do not own -- the talker/SPAS paths drive it). */
static void pp_drive_apply(struct gpib_context *ctx) {
    bool want = ctx->pp.capable && ctx->pp.state == GPIB_PP_PPAS &&
                ctx->pp_ist == ctx->pp.sense;
    if (want) {
        uint8_t ppr = (uint8_t)(1u << ctx->pp.dio_line);
        ctx->port->drive_data(ctx->port->user, &ppr);
        ctx->line.pp_drove_dio = true;
    } else if (ctx->line.pp_drove_dio) {
        ctx->port->drive_data(ctx->port->user, NULL);
        ctx->line.pp_drove_dio = false;
    }
}

/* Device PP1 round evaluation (F04, 2.9 Fig. 11): PPSS -> PPAS on
 * IDY (ATN&&EOI), PPAS -> PPSS when either releases. Command bytes are
 * fed separately at ACDS (byte_valid) with the configure arcs. */
static void pp_round_eval(struct gpib_context *ctx,
    const gpib_line_snap_t *snap) {
    if (!ctx->pp.capable) {
        return;
    }
    if (snap->atn || ctx->pp.state == GPIB_PP_PPAS) {
        gpib_pp_in_t pin;
        gpib_pp_out_t pout;
        memset(&pin, 0, sizeof(pin));
        pin.atn = snap->atn;
        pin.eoi = snap->eoi;
        pin.ist = ctx->pp_ist;
        pin.lads = (ctx->line.ln.state == GPIB_L_LADS ||
                    ctx->line.ln.state == GPIB_L_LACS);
        gpib_pp_step(&ctx->pp, &pin, &pout);
        (void)pout;
    }
}

void gpib_lineexec_poll_idle(struct gpib_context *ctx) {
    gpib_line_snap_t snap;
    if (!ctx->port->read_lines(ctx->port->user, &snap)) {
        return;
    }
    ctx->line.srq_seen = snap.srq;
    /* F01: an idle device restarts its interface functions on an
     * external IFC too; the round continues in the reset state (AH in
     * AIDS drives passive lines, exactly the post-IFC wire picture). */
    (void)external_ifc_reset(ctx, &snap);
    sr_round_eval(ctx); /* F18: SR advances on idle rounds too */
    if (ctx->line.seq == (uint8_t)GPIB_LSEQ_IDLE ||
        ctx->line.seq == (uint8_t)GPIB_LSEQ_DEV_WAIT) {
        /* Idle device observation: handshake ATN command bytes (service
         * channel continuity) even with no pending exchange action. */
        dev_wait_step(ctx, &snap);
    }
    ctrl_fsm_step(ctx, &snap);
}

/* Device role: wait for the external controller. ATN command bytes are
 * handshaken via AH (T3), decoded and fed into the addressing engine and
 * the device T/L FSMs; device commands go to the service channel. When
 * ATN releases and our talker/listener is addressed, the pending action
 * enters the regular DATA_TX/DATA_RX phases. */
static void dev_wait_step(struct gpib_context *ctx,
    const gpib_line_snap_t *snap) {
    struct gpib_action *a = &ctx->action;
    gpib_ah_in_t in;
    gpib_ah_out_t out;
    gpib_line_drive_t req;
    memset(&in, 0, sizeof(in));
    pp_round_eval(ctx, snap); /* PP1 IDY observation (F04) */
    if (ctx->spas_tx_active && snap->atn) {
        /* D01 (final audit): an ATN steal aborts any in-flight STB --
         * the SPAS answer branch below returns before the talker can
         * leave on its own (its SH feed hardwires atn=false), so the
         * cancel MUST run ahead of the command-listening branch or the
         * latch survives the steal and injects the stale STB into the
         * next data supply. */
        ctx->spas_tx_active = false;
        ctx->port->drive_data(ctx->port->user, NULL);
        sh_reset(ctx);
    }
    in.atn = snap->atn;
    in.dav = snap->dav;
    in.lads = ctx->line.addr.lads || ctx->line.ln.state != GPIB_L_LIDS;
    in.lacs = in.lads;
    in.rdy = true; /* idle device: always ready for interface commands */
    in.now_us = snap->now_us;
    in.t3_us = LNX_T3_US;
    if (snap->atn) {
        gpib_ah_step(&ctx->line.ah, &in, &out);
        if (!out.byte_valid) {
            /* AUD-09 (2.10.3.2/2.11.3.2): between accepted command bytes
             * the ACDS window is CLOSED -- the DC/DT unit FSMs must see
             * that false level to exit DCAS/DTAS, or a second GET/DCL
             * in the same command phase is swallowed (one event instead
             * of two). Feed the no-command round to the DF trio. */
            gpib_ifcmd_t none;
            memset(&none, 0, sizeof(none));
            df_step(ctx, snap, false, none,
                    ctx->line.ln.state == GPIB_L_LADS ||
                        ctx->line.ln.state == GPIB_L_LACS);
        }
        if (out.byte_valid) {
            gpib_ifcmd_t cmd = gpib_decode_atn_byte(snap->dio);
            gpib_addr_step(&ctx->line.addr, &cmd);
            {
                /* Feed the device T/L FSMs with the accepted command. */
                gpib_t_in_t tin;
                gpib_t_out_t tout;
                gpib_l_in_t lin;
                gpib_l_out_t lout;
                memset(&tin, 0, sizeof(tin));
                tin.atn = true;
                tin.cmd_valid = true;
                tin.cmd = cmd;
                gpib_talker_step(&ctx->line.tk, &tin, &tout);
                (void)tout;
                memset(&lin, 0, sizeof(lin));
                lin.atn = true;
                lin.cmd_valid = true;
                lin.cmd = cmd;
                gpib_listener_step(&ctx->line.ln, &lin, &lout);
                (void)lout;
                /* Device PP1 (F04): every accepted ATN byte feeds the
                 * configure/response groups (2.9 Fig. 11). The lads
                 * fact is the normative listener state (LADS/LACS,
                 * LE-complete after MSA(own)) -- same source as df. */
                {
                    gpib_pp_in_t pin;
                    gpib_pp_out_t pout;
                    memset(&pin, 0, sizeof(pin));
                    pin.cmd_valid = true;
                    pin.cmd = cmd;
                    pin.atn = true;
                    pin.eoi = false;
                    pin.ist = ctx->pp_ist;
                    pin.lads = (ctx->line.ln.state == GPIB_L_LADS ||
                                ctx->line.ln.state == GPIB_L_LACS);
                    gpib_pp_step(&ctx->pp, &pin, &pout);
                    (void)pout;
                }
            }
            if (cmd.cls != GPIB_CMD_MLA && cmd.cls != GPIB_CMD_MTA &&
                cmd.cls != GPIB_CMD_UNL && cmd.cls != GPIB_CMD_UNT &&
                cmd.cls != GPIB_CMD_SECONDARY &&
                cmd.cls != GPIB_CMD_UNDEFINED) {
                /* Device command (GTL/SDC/PPC/GET/TCT/LLO/DCL/PPU/SPE/SPD):
                 * service channel + trace. */
                ctx->dev_evt[ctx->dev_evt_head] = snap->dio;
                ctx->dev_evt_head =
                    (ctx->dev_evt_head + 1u) % GPIB_CFG_DEVICE_EVENTS;
                if (ctx->dev_evt_count < GPIB_CFG_DEVICE_EVENTS) {
                    ctx->dev_evt_count++;
                } else {
                    ctx->dev_evt_dropped++;
                }
                GPIB_TRACE(ctx, GPIB_TRC_DEVICE_CMD, 0, 0, snap->dio);
            }
            if (cmd.cls == GPIB_CMD_TCT &&
                (ctx->line.tk.state == GPIB_T_TADS ||
                 ctx->line.tk.state == GPIB_T_TACS)) {
                /* Another controller passes control to us: legitimate
                 * ONLY when OUR talker was addressed (MTA(own) preceded
                 * TCT) -- a stray 0x09 must not hand us the bus (F16). */
                ctx->line.ctrl_tct_pulse = true;
            }
            /* RL/DC/DT qualification of the accepted byte (ACDS window).
             * F10: the lads fact comes from the NORMATIVE listener FSM
             * only (LADS/LACS, including the LE pair after MSA(own));
             * the primary-only addr mirror used to fire SDC/GET/GTL at
             * devices whose secondary addressing was not complete.
             * AUD-20 (2.8.3.1): the RL remote-enable trigger is the
             * acceptance of the device's OWN MLA -- for an LE device
             * that is the primary MLA(own) itself (the listener FSM
             * only reaches LADS after the SAD completes, one byte later,
             * by which time the command class is SECONDARY and the
             * window is gone). Feed the own-MLA acceptance as an
             * equivalent lads fact to RL. */
            {
                bool lads_fact =
                    ctx->line.ln.state == GPIB_L_LADS ||
                    ctx->line.ln.state == GPIB_L_LACS ||
                    (cmd.cls == GPIB_CMD_MLA &&
                     cmd.arg == ctx->own_address.primary);
                df_step(ctx, snap, true, cmd, lads_fact);
            }
        }
        memset(&req, 0, sizeof(req));
        req.nrfd = !out.rfd_true;
        req.ndac = !out.dac_true;
        req.srq = (ctx->sr.state == GPIB_SR_SRQS);
        req.ifc = ctx->line.sc_ifc;
        req.ren = ctx->line.sc_ren;
        pp_drive_apply(ctx); /* PPR answer rides alongside (F04) */
        /* A CIC controller HOLDS ATN even while accepting commands (its
         * own ATN reads back TRUE on the wired-OR bus - without this the
         * drive would oscillate between the command and idle branches).
         * we_drive_atn keeps the hold through a takeover march (the
         * stop-path ATN assert walking CSBS->CAWS->CACS, N11-04). */
        if (ctx->line.ctrl.main == GPIB_C_CACS ||
            ctx->line.ctrl.main == GPIB_C_CTRS ||
            ctx->line.we_drive_atn) {
            req.atn = true;
        }
        gpib_lineexec_drive_out(ctx, &req);
        return;
    }
    /* Data-mode round: refresh the T/L view, keep DCAS/DTAS windows closed
     * and REN observation running, then check transitions. */
    {
        gpib_ifcmd_t none;
        memset(&none, 0, sizeof(none));
        df_step(ctx, snap, false, none, false);
    }
    /* SR function: rsv from the 488.2 status model; SPAS from the talker.
     * After the poll reads us out (STB accepted in SPAS), a single-cause
     * request self-clears ([P]: the module re-asserts if still armed). */
    {
        /* (moved) SR stepping is central now: see sr_round_eval. */
    }
    /* Serial poll answer: when the talker is in SPAS (SPE armed us) the
     * next byte we source is the STB with RQS --- before any user supply. */
    if (!snap->atn && ctx->line.tk.state == GPIB_T_SPAS &&
        !ctx->spas_tx_active) {
        ctx->spas_tx_active = true;
        sh_reset(ctx);
    }
    if (ctx->spas_tx_active &&
        (ctx->line.tk.state != GPIB_T_SPAS || snap->atn)) {
        /* D01 (final audit): the poll was INTERRUPTED mid-STB (ATN steal,
         * IFC, UNT or takeover -- the talker left SPAS without the byte
         * being accepted, or ATN is asserted and the SPAS branch below
         * would keep stepping the SH with a hardwired atn=false, so the
         * talker can never leave on its own). The old code kept the
         * latch alive: the stale STB was later injected as the FIRST
         * byte of the next data supply (a controller that times out a
         * hung poll and then reads data got 0x62 'D' 'A' 'T' 'A' '!'
         * instead of "DATA!"), and with no repeat poll the TX action
         * could never start. Cancel the phantom: drop the latch and the
         * DIO drivers. */
        ctx->spas_tx_active = false;
        ctx->port->drive_data(ctx->port->user, NULL);
        sh_reset(ctx);
    }
    if (ctx->spas_tx_active) {
        gpib_sh_in_t sin;
        gpib_sh_out_t sout;
        uint8_t stb = (uint8_t)(ctx->device_stb |
            ((ctx->sr.state == GPIB_SR_APRS) ? 0x40u : 0x00u));
        memset(&sin, 0, sizeof(sin));
        sin.atn = false;
        sin.tacs = !snap->atn;
        sin.nba = (ctx->line.sh.state == GPIB_SH_SGNS);
        sin.rfd = !snap->nrfd;
        sin.dac = !snap->ndac;
        sin.now_us = snap->now_us;
        sin.t1_us = 2u;
        gpib_sh_step(&ctx->line.sh, &sin, &sout);
        if (sout.byte_accepted) {
            ctx->spas_tx_active = false; /* one STB per poll */
            /* Release the data drivers: the next controller command phase
             * drives DIO; a stale STB byte is a two-driver contention. */
            ctx->port->drive_data(ctx->port->user, NULL);
            if (ctx->sr.state == GPIB_SR_APRS) {
                ctx->device_rsv = false; /* serviced */
            }
        }
        {
            gpib_line_drive_t sreq;
            memset(&sreq, 0, sizeof(sreq));
            sreq.dav = sout.dav_requested;
            sreq.srq = (ctx->sr.state == GPIB_SR_SRQS);
            gpib_lineexec_drive_out(ctx, &sreq);
            if (ctx->line.sh.state == GPIB_SH_SGNS) {
                ctx->port->drive_data(ctx->port->user, &stb);
            }
            if (ctx->line.sh.state == GPIB_SH_SIDS ||
                ctx->line.sh.state == GPIB_SH_SIWS) {
                ctx->port->drive_data(ctx->port->user, NULL);
            }
        }
        return;
    }
    {
        gpib_t_in_t tin;
        gpib_t_out_t tout;
        gpib_l_in_t lin;
        gpib_l_out_t lout;
        memset(&tin, 0, sizeof(tin));
        tin.atn = false;
        gpib_talker_step(&ctx->line.tk, &tin, &tout);
        (void)tout;
        memset(&lin, 0, sizeof(lin));
        lin.atn = false;
        gpib_listener_step(&ctx->line.ln, &lin, &lout);
        (void)lout;
    }
    bool talk_ready = (ctx->line.tk.state == GPIB_T_TADS ||
                       ctx->line.tk.state == GPIB_T_TACS);
    bool listen_ready = (ctx->line.ln.state == GPIB_L_LADS ||
                         ctx->line.ln.state == GPIB_L_LACS);
    if (a->state == (uint8_t)GPIB_ACTION_ACTIVE) {
        if (a->kind == (uint8_t)GPIB_ACTION_TX_BLOCK && talk_ready) {
            ctx->line.seq = (uint8_t)GPIB_LSEQ_DATA_TX;
            sh_reset(ctx);
            ctx->line.tx_pos = 0;
            return;
        }
        if (a->kind == (uint8_t)GPIB_ACTION_RX_BLOCK && listen_ready) {
            ctx->line.seq = (uint8_t)GPIB_LSEQ_DATA_RX;
            return;
        }
    }
    /* Idle lines: unaddressed acceptor is passive; SRQ reflects SR. */
    memset(&req, 0, sizeof(req));
    req.srq = (ctx->sr.state == GPIB_SR_SRQS);
    req.ifc = ctx->line.sc_ifc;
    req.ren = ctx->line.sc_ren;
    if ((ctx->line.rx_stop_hold || ctx->line.rx_session_open) &&
        !snap->atn) {
        /* F13 (488.2 16.2.6): RECEIVE stop state -- hold NRFD/NDAC TRUE
         * until a control sequence (ATN) or a new action takes over.
         * F05: the same hold expresses READ backpressure between blocks
         * of an OPEN stream: the remote talker waits in SDYS while the
         * application stages the next buffer.
         * AUD-05 (488.1 2.4.3.5, AWNS): after the LAST accepted byte the
         * released NDAC must STAY released until the source drops DAV --
         * NRFD held (not ready) meanwhile. Only then the full stop-state
         * hold (both lines) begins. The accept pulse width is the
         * source's to observe, not ours to cut short. */
        req.nrfd = true;
        req.ndac = !snap->dav;
    } else {
        ctx->line.rx_stop_hold = false; /* ATN took the bus: hold ends */
        ctx->line.rx_session_open = false; /* F05: ATN kills the session */
        if (!snap->atn &&
            (ctx->line.ln.state == GPIB_L_LADS ||
             ctx->line.ln.state == GPIB_L_LACS)) {
            /* AUD-08: an ADDRESSED listener with no RX buffer staged is
             * in ANRS (not ready) -- it must hold NRFD/NDAC TRUE: the
             * old passive idle let the CIC's DAV handshakes run away
             * and bytes were lost on the wire. The bus stalls until
             * the application supplies a sink (or is unaddressed). */
            req.nrfd = true;
            req.ndac = true;
        }
    }
    if (ctx->line.ctrl.main != GPIB_C_CIDS) {
        /* H1 (final audit): while the C model is ACTIVE (CACS/CTRS) the
         * idle drive holds ATN on the wire -- 16.2.1/16.2.9: ATN stays
         * asserted until another sequence changes it. The old
         * `action.state == EMPTY` gate released ATN right after every
         * action (model said CACS, wire said 0). After a data phase the
         * model sits in CSBS (gts) and no ATN is held -- correct for an
         * open RECEIVE session (F05). we_drive_atn extends the hold
         * through a takeover march (stop-path ATN assert, N11-04). */
        req.atn = (ctx->line.ctrl.main == GPIB_C_CACS ||
                   ctx->line.ctrl.main == GPIB_C_CTRS) ||
                  ctx->line.we_drive_atn;
    }
    pp_drive_apply(ctx); /* release our PPR drivers if the window closed */
    gpib_lineexec_drive_out(ctx, &req);
}

/* FINDLSTN probe sub-phases staged in ctx->line.cmd_pos (normative
 * 488.2 17.6.5: NO data byte is ever transferred). */
enum {
    FL_UNL = 0, /* unlisten the previous candidate */
    FL_MLA,     /* address the current candidate */
    FL_SETTLE,  /* ATN released, wait >= 1.5 ms */
    FL_SAMPLE   /* read the NDAC level: asserted -> a listener exists */
};

/* External IFC observed on the wire (F01): reset the device-side
 * interface functions within t4 (488.1): talker/listener idle with the
 * pon own-address configuration, serial-poll pair disarmed, handshake
 * functions and the addressing mirror reinitialized, SR back to NPRS
 * (a live rsv re-raises SRQ on the next rounds). Our OWN SC pulse
 * (ctx->line.sc_ifc) reads back TRUE on the wired-OR bus and must not
 * reset ourselves. Returns true when the snapshot was an IFC round. */
static bool external_ifc_reset(struct gpib_context *ctx,
    const gpib_line_snap_t *snap) {
    /* Our own pulse -- driven NOW or in the PREVIOUS round: the released
     * line still reads TRUE for one exchange on the wired-OR bus, and
     * the C FSM lowers sc_ifc before the drive call catches up. */
    bool own_pulse = ctx->line.sc_ifc || ctx->line.sc_ifc_last;
    ctx->line.sc_ifc_last = ctx->line.sc_ifc;
    if (!snap->ifc || own_pulse) {
        return false;
    }
    sh_reset(ctx);
    gpib_ah_init(&ctx->line.ah);
    gpib_apply_own_address(ctx);
    /* M1 (final audit): IFC does NOT touch SR in 488.1 (only pon does);
     * resetting SR here resurrected a serviced request's SRQ because a
     * live rsv re-asserts after NPRS. The rsv input alone drives SRQS.
     * AUD-13: the same holds for the PP configure/response groups --
     * Fig. 11 has NO IFC arc; only pon, PPD and PPU disarm a
     * configuration. The old blanket pon used to kill a remotely
     * configured PPE at the first IFC. */
    ctx->line.rx_session_open = false; /* F05: IFC kills the session */
    ctx->line.rx_stop_hold = false;
    ctx->spas_tx_active = false; /* D01: IFC kills a phantom STB too */
    ctx->port->drive_data(ctx->port->user, NULL);
    return true;
}

void gpib_lineexec_external_ifc(struct gpib_context *ctx) {
    /* A02: the shared reset body for a QUEUED untagged IFC fact -- the
     * line-level variant (external_ifc_reset) needs a snapshot; this
     * wrapper applies the same interface-function reset directly.
     * Byte-pump ports have no electrical lines -- nothing to release. */
    sh_reset(ctx);
    gpib_ah_init(&ctx->line.ah);
    gpib_apply_own_address(ctx);
    /* M1/AUD-13: SR and PP are NOT touched by IFC (only pon is). */
    ctx->line.rx_session_open = false;
    ctx->line.rx_stop_hold = false;
    ctx->spas_tx_active = false;
    if (gpib_line_mode(ctx)) {
        ctx->port->drive_data(ctx->port->user, NULL);
    }
}

void gpib_lineexec_step(struct gpib_context *ctx) {
    struct gpib_action *a = &ctx->action;
    gpib_line_snap_t snap;
    if (!ctx->port->read_lines(ctx->port->user, &snap)) {
        return; /* port could not sample: no progress this step */
    }
    ctx->line.srq_seen = snap.srq;
    /* F01: external IFC has priority over every data/command logic: the
     * in-flight exchange is aborted with the IFC terminal cause (the
     * last byte fate is unproven) and the functions return to pon. */
    /* F01: external IFC has priority over every data/command logic: the
     * in-flight exchange is aborted with the IFC terminal cause (the
     * last byte fate is unproven) and the functions return to pon. The
     * own-pulse guard (including the trailing-edge round) lives INSIDE
     * external_ifc_reset: only its verdict may terminate the action. */
    {
        /* A11 (independent audit): in_flight only latches on the
         * role-lost transition -- a byte sitting in STRS (DAV up, DAC
         * not yet seen) is ALSO unproven at an IFC abort: without a
         * common DAC we cannot know whether ANY listener already took
         * it. Capture the STRS condition BEFORE the reset wipes the SH. */
        bool byte_unproven = ctx->line.sh.in_flight ||
                             ctx->line.sh.state == GPIB_SH_STRS ||
                             ctx->line.sh.state == GPIB_SH_SWNS ||
                             ctx->line.sh.state == GPIB_SH_SDYS;
        if (external_ifc_reset(ctx, &snap)) {
            if (a->state == (uint8_t)GPIB_ACTION_ACTIVE) {
                a->last_byte_uncertain = byte_unproven;
                finish_action(ctx, GPIB_TERM_IFC, false);
                return;
            }
        }
    }
    sr_round_eval(ctx); /* F18: SR advances during active rounds too */
    {
        /* A04 (488.1 2.8.3.3-4): RL must observe the CONTINUOUS REN
         * level on every active bus round -- the old code fed it only
         * at command bytes, so dropping REN mid-exchange left a device
         * stuck in REMS forever (local controls logically locked).
         * The one-shot command bytes (GTL/LLO) still arrive via the
         * ACDS feed in dev_wait; this is the continuous-level feed. */
        gpib_ifcmd_t none;
        memset(&none, 0, sizeof(none));
        df_step(ctx, &snap, false, none,
                ctx->line.ln.state == GPIB_L_LADS ||
                    ctx->line.ln.state == GPIB_L_LACS);
    }
    /* Stop latch / deadline already latched a cause: end at this boundary. */
    if (a->state == (uint8_t)GPIB_ACTION_ACTIVE &&
        a->stop_cause != (uint8_t)GPIB_TERM_NONE &&
        ctx->line.seq != (uint8_t)GPIB_LSEQ_IDLE) {
        gpib_term_reason_t cause = (gpib_term_reason_t)a->stop_cause;
        finish_action(ctx, cause, false);
        return;
    }
    switch (ctx->line.seq) {
    case (uint8_t)GPIB_LSEQ_CMD:
        sh_phase(ctx, &snap, true, ctx->line.cmds, ctx->line.n_cmds,
            &ctx->line.cmd_pos, false);
        break;
    case (uint8_t)GPIB_LSEQ_EPILOGUE:
        sh_phase(ctx, &snap, true, ctx->line.post, ctx->line.n_post,
            &ctx->line.post_pos, false);
        break;
    case (uint8_t)GPIB_LSEQ_DATA_TX_PENDING: {
        /* RESET level 3: MLA(addr) is staged in cmds; send it, then run
         * the *RST data phase; when done, advance to the next address. */
        sh_phase(ctx, &snap, true, ctx->line.cmds, ctx->line.n_cmds,
            &ctx->line.cmd_pos, false);
        if (ctx->line.cmd_pos == ctx->line.n_cmds &&
            ctx->line.sh.state == GPIB_SH_SWNS) {
            ctx->line.seq = (uint8_t)GPIB_LSEQ_DATA_TX;
            ctx->line.we_drive_atn = false;
            apply_release(ctx);
            sh_reset(ctx);
            ctx->line.tx_pos = 0;
        }
        break;
    }
    case (uint8_t)GPIB_LSEQ_DATA_TX:
        if (snap.atn && !ctx->line.we_drive_atn) {
            /* External controller stole the bus mid-data: P1 yield with
             * the confirmed prefix; continue listening for commands. */
            bool dev = ctx->txn[a->txn_slot].kind == GPIB_REQ_DEV_TX ||
                       ctx->txn[a->txn_slot].kind == GPIB_REQ_DEV_RX;
            if (!dev) {
                gpib_ah_init(&ctx->line.ah);
            }
            finish_action(ctx, GPIB_TERM_PHASE_INTERRUPTED, false);
            /* F15: finish_action resets seq to IDLE -- re-establish the
             * observer state AFTER it, or the steal leaves the sequencer
             * deaf to the usurper's command stream. */
            ctx->line.seq = (uint8_t)(dev ? GPIB_LSEQ_DEV_WAIT
                                          : GPIB_LSEQ_LISTEN_CMDS);
            return;
        }
        sh_phase(ctx, &snap, false, a->data, a->len, &ctx->line.tx_pos, true);
        if (ctx->line.sh.state == GPIB_SH_SIWS) {
            /* SH lost its role asynchronously (ATN seen by the FSM):
             * the in-flight byte fate is unproven (PHASE_INTERRUPTED). */
            bool in_flight = ctx->line.sh.in_flight;
            bool dev = ctx->txn[a->txn_slot].kind == GPIB_REQ_DEV_TX ||
                       ctx->txn[a->txn_slot].kind == GPIB_REQ_DEV_RX;
            if (!dev) {
                gpib_ah_init(&ctx->line.ah);
            }
            if (in_flight) {
                a->last_byte_uncertain = true;
            }
            finish_action(ctx, GPIB_TERM_PHASE_INTERRUPTED, false);
            /* F15: same re-establishment after the finish overwrite. */
            ctx->line.seq = (uint8_t)(dev ? GPIB_LSEQ_DEV_WAIT
                                          : GPIB_LSEQ_LISTEN_CMDS);
        }
        break;
    case (uint8_t)GPIB_LSEQ_PPOLL_IDY: {
        /* PERFORM PARALLEL POLL (16.15): assert ATN+EOI, hold for T6,
         * then read the PPR byte and release. AUD-14: T6 is a TIME
         * bound (Table 39, >= 2 us), not a round counter -- a frozen
         * clock must stall the poll (no result), a fast clock must not
         * shorten the settle. The rounds cap only guards a runaway. */
        if (ctx->line.idy_rounds == 0) {
            /* First IDY round: stamp the physical start. */
            ctx->line.idy_start_us = snap.now_us;
        }
        if ((snap.now_us - ctx->line.idy_start_us) <
                (uint64_t)GPIB_CFG_T6_SETTLE_US) {
            if (ctx->line.idy_rounds >= 100000u) {
                /* A07 (independent audit): the round cap guards a
                 * runaway, but a count overflow is NOT proof of a
                 * settled T6 -- a frozen or dead port clock must fail
                 * the poll, never fabricate a successful PPR read. */
                ctx->line.idy_rounds = 0;
                apply_release(ctx);
                finish_action(ctx, GPIB_TERM_FAULT, false);
                break;
            }
            gpib_line_drive_t sreq;
            memset(&sreq, 0, sizeof(sreq));
            sreq.atn = true;
            sreq.eoi = true;
            sreq.ren = ctx->line.sc_ren;
            gpib_lineexec_drive_out(ctx, &sreq);
            ctx->line.idy_rounds++;
            break;
        }
        /* Read the PPR byte: the DIO lines carry the wired-OR of all
         * configured PP devices that matched their sense bit. */
        ctx->txn[a->txn_slot].ppr_byte = snap.dio;
        ctx->line.idy_rounds = 0;
        apply_release(ctx);
        finish_action(ctx, GPIB_TERM_COMPLETED, false);
        break;
    }
    case (uint8_t)GPIB_LSEQ_FINDLSTN_PROBE: {
        /* FINDLSTN (488.2 17.6.5, NORMATIVE): per candidate, UNL +
         * MLA(addr) with ATN, then ATN FALSE, a settle of at least
         * 1.5 ms, and a READ of NDAC: an addressed listener's acceptor
         * holds NDAC asserted (ANRS); nobody addressed -> every AH is
         * in AIDS with passive lines. NO DAB is ever sourced: the old
         * probe byte opened a two-talker DIO conflict whenever a
         * previous talker still had an unconsumed reply (final-audit
         * blocker, reproduced by the chaos stress). The sub-phase is
         * staged in ctx->line.cmd_pos (FL_*); the epilogue
         * (probe_index >= probe_count) sends a final UNL. */
        struct gpib_txn_record *tr = &ctx->txn[a->txn_slot];
        uint8_t byte = 0;
        size_t pos = 0;
        bool atn_phase;
        if (tr->probe_index >= tr->probe_count) {
            /* Epilogue: UNL so the last-present listener does not stay
             * addressed after the scan. */
            byte = GPIB_UNL_BYTE; /* UNL */
            atn_phase = true;
            ctx->line.we_drive_atn = true;
            sh_phase(ctx, &snap, atn_phase, &byte, 1, &pos, false);
            if (pos == 1 && ctx->line.sh.state == GPIB_SH_SWNS) {
                finish_action(ctx, GPIB_TERM_COMPLETED, false);
            }
            break;
        }
        if (ctx->line.cmd_pos == (uint8_t)FL_UNL) {
            byte = GPIB_UNL_BYTE;
            atn_phase = true;
            ctx->line.we_drive_atn = true;
            sh_phase(ctx, &snap, atn_phase, &byte, 1, &pos, false);
            if (pos == 1 && ctx->line.sh.state == GPIB_SH_SWNS) {
                ctx->line.cmd_pos = (uint8_t)FL_MLA;
                sh_reset(ctx);
            }
            break;
        }
        if (ctx->line.cmd_pos == (uint8_t)FL_MLA) {
            byte = gpib_mk_mla(tr->probe_addrs[tr->probe_index]);
            atn_phase = true;
            ctx->line.we_drive_atn = true;
            sh_phase(ctx, &snap, atn_phase, &byte, 1, &pos, false);
            if (pos == 1 && ctx->line.sh.state == GPIB_SH_SWNS) {
                /* MLA done (17.6.5): release ATN -- the settle window
                 * runs with ATN FALSE and the controller sources
                 * NOTHING on DIO (the old DAB probe opened a two-talker
                 * conflict against a stale talker's reply). AUD-21: the
                 * deadline is stored as (start, duration) -- wrap-safe. */
                ctx->line.cmd_pos = (uint8_t)FL_SETTLE;
                ctx->line.probe_settle_deadline = snap.now_us;
                apply_release(ctx);
            }
            break;
        }
        if (ctx->line.cmd_pos == (uint8_t)FL_SETTLE) {
            /* ATN FALSE for >= 1.5 ms: an addressed listener's AH
             * settles into ANRS (NDAC asserted); unaddressed devices
             * are in AIDS with passive lines. */
            ctx->line.we_drive_atn = false;
            if ((uint64_t)(snap.now_us - ctx->line.probe_settle_deadline) >=
                (uint64_t)GPIB_CFG_FINDLSTN_SETTLE_US) {
                ctx->line.cmd_pos = (uint8_t)FL_SAMPLE;
            }
            break;
        }
        /* FL_SAMPLE: read the wired-OR NDAC level -- asserted means an
         * acceptor is active at the probed address (present). */
        ctx->line.we_drive_atn = false;
        tr->probe_results[tr->probe_index] = snap.ndac;
        tr->probe_index++;
        ctx->line.cmd_pos = (uint8_t)FL_UNL;
        sh_reset(ctx);
        break;
    }
    case (uint8_t)GPIB_LSEQ_POLL_ADDR: {
        /* ALLSPOLL/FINDRQS: with SPE armed, address the next device as
         * talker (ATN TRUE), then switch to a one-byte DATA_RX for its
         * STB. An exhausted list falls through to the SPD/UNT epilogue. */
        struct gpib_txn_record *tr = &ctx->txn[a->txn_slot];
        if (tr->poll_index >= tr->poll_count) {
            ctx->line.seq = (uint8_t)GPIB_LSEQ_EPILOGUE;
            ctx->line.post_pos = 0;
            ctx->line.we_drive_atn = true;
            sh_reset(ctx);
            break;
        }
        uint8_t mta = gpib_mk_mta(tr->poll_addrs[tr->poll_index]);
        ctx->line.we_drive_atn = true;
        sh_phase(ctx, &snap, true, &mta, 1, &ctx->line.cmd_pos, false);
        if (ctx->line.cmd_pos == 1 &&
            ctx->line.sh.state == GPIB_SH_SWNS) {
            /* MTA done: one-byte read of this device's STB. */
            a->data = &tr->poll_results[tr->poll_index];
            a->len = 1;
            a->limit = 1;
            a->committed = 0;
            ctx->line.rx_captured = 0;
            ctx->line.rx_eoi = false;
            ctx->line.rx_holdoff = 0;
            gpib_ah_init(&ctx->line.ah);
            ctx->line.seq = (uint8_t)GPIB_LSEQ_DATA_RX;
            ctx->line.we_drive_atn = false;
            apply_release(ctx);
        }
        break;
    }
    case (uint8_t)GPIB_LSEQ_DATA_RX:
        if (snap.atn && !ctx->line.we_drive_atn) {
            bool dev = ctx->txn[a->txn_slot].kind == GPIB_REQ_DEV_TX ||
                       ctx->txn[a->txn_slot].kind == GPIB_REQ_DEV_RX;
            ctx->line.seq = (uint8_t)(dev ? GPIB_LSEQ_DEV_WAIT
                                          : GPIB_LSEQ_LISTEN_CMDS);
            if (!dev) {
                gpib_ah_init(&ctx->line.ah);
            }
            /* External controller stole the bus mid-data: P1 yield with
             * the confirmed prefix; continue listening for commands. */
            finish_action(ctx, GPIB_TERM_PHASE_INTERRUPTED, false);
            return;
        }
        ah_phase(ctx, &snap, false);
        break;
    case (uint8_t)GPIB_LSEQ_LISTEN_CMDS:
        ah_phase(ctx, &snap, true);
        break;
    case (uint8_t)GPIB_LSEQ_DEV_WAIT:
        dev_wait_step(ctx, &snap);
        break;
    default:
        break;
    }
    /* Device-role actions return to the addressing wait after finishing,
     * keeping the service channel and re-addressing alive. */
    if (ctx->line.seq == (uint8_t)GPIB_LSEQ_IDLE &&
        (a->state == (uint8_t)GPIB_ACTION_QUIESCED ||
         a->state == (uint8_t)GPIB_ACTION_RESULT_ACCOUNTED)) {
        gpib_request_kind_t k = ctx->txn[a->txn_slot].kind;
        if (k == GPIB_REQ_DEV_TX || k == GPIB_REQ_DEV_RX) {
            ctx->line.seq = (uint8_t)GPIB_LSEQ_DEV_WAIT;
        }
    }
    ctrl_fsm_step(ctx, &snap);
}
