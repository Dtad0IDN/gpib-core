#include <string.h>

#include "gpib_internal.h"

/* REQ-078/079: this build has no TE/LE interface functions, so a present
 * secondary must fail loudly before any bus action or ownership capture.
 * Value ranges await the normative baseline (OPEN-01/02); the Keysight
 * 0..30 documentation is deliberately not treated as a norm. */
bool gpib_address_supported(const gpib_address_t *a) {
    /* Primary address space is 0..30 (F12): 31 is reserved -- its
     * encodings are UNL (0x3F) and UNT (0x5F) -- and larger values alias
     * into other command groups through the 5-bit address field.
     * TE/LE is implemented: secondary addresses are supported when the
     * value is 0..31 (the SAD encoding space; SAD(n) = 0x60|n). The
     * REQ-078/079 reservation is honoured: absence only via the flag. */
    if (a->primary > 30u) {
        return false;
    }
    if (a->secondary_present && a->secondary > 31u) {
        return false;
    }
    return true;
}

static gpib_status_t busy(struct gpib_context *ctx, gpib_busy_reason_t reason) {
    ctx->last_busy_reason = reason;
    return GPIB_BUSY_RESOURCES;
}

void gpib_fault_latch(struct gpib_context *ctx, gpib_fault_code_t code) {
    uint64_t now = ctx->port->now_us(ctx->port->user);
    if (!ctx->fault_latched) {
        ctx->fault_latched = true;
        ctx->fault_code = code;
        ctx->first_fault_us = now;
    }
    ctx->last_update_us = now;
    GPIB_TRACE(ctx, GPIB_TRC_FAULT, 0, 0, (uint32_t)code);
}

/* Wipe keeps the storage fingerprint so the same placement stays
 * re-initializable; old handles die via the epoch manager, never via reuse. */
static void wipe(struct gpib_context *ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->storage_fingerprint = GPIB_STORAGE_FINGERPRINT;
}

/* (Re)configure the addressing functions from ctx->own_address (F11):
 * every automaton that keys on the own address must move together --
 * a stale mirror would keep answering on the OLD address. Also the
 * IFC reset path (F01): the interface functions return to their pon
 * configuration, which includes the own-address setup. */
void gpib_apply_own_address(struct gpib_context *ctx) {
    bool has_sec = ctx->own_address.secondary_present &&
                   ctx->own_address.secondary <= 31u;
    gpib_addr_init(&ctx->line.addr, ctx->own_address.primary);
    gpib_talker_init_ex(&ctx->line.tk,
        ctx->own_address.primary, true, false,
        has_sec, ctx->own_address.secondary);
    gpib_listener_init_ex(&ctx->line.ln,
        ctx->own_address.primary, true,
        has_sec, ctx->own_address.secondary);
}

gpib_status_t gpib_init(gpib_context_t *ctx, const gpib_init_params_t *params) {
    if (ctx == NULL || params == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle != GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_STATE; /* repeated init on a live instance */
    }
    if (ctx->storage_fingerprint != GPIB_STORAGE_FINGERPRINT) {
        return GPIB_INVALID_ARGUMENT; /* storage/core profile mismatch */
    }
    if (!gpib_address_supported(&params->own_address)) {
        return GPIB_UNSUPPORTED_ADDRESSING; /* config unchanged */
    }
    uint64_t epoch = 0;
    gpib_status_t st = gpib_epoch_next(&epoch);
    if (st != GPIB_OK) {
        return st;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->storage_fingerprint = GPIB_STORAGE_FINGERPRINT;
    ctx->lifecycle = GPIB_LC_INITIALIZED; /* no protocol exchange (LIF-01) */
    ctx->epoch = epoch;
    ctx->system_controller = params->system_controller;
    ctx->own_address = params->own_address;
    ctx->port = (params->port != NULL) ? params->port : &gpib_null_port;
    /* Device-role function FSMs: T2-style talker (serial poll capable),
     * L4-style listener (unaddress on own MTA). When the own address
     * carries a secondary, the TE/LE extended pairs are armed. */
    gpib_apply_own_address(ctx);
    gpib_controller_init(&ctx->line.ctrl);
    ctx->line.ctrl_tct_pulse = false;
    ctx->line.send_tct = false;
    /* Device-function FSMs: RL1 (full lockout), DC1 (DCL+SDC), DT1. */
    gpib_rl_init(&ctx->rl, true);
    gpib_dc_init(&ctx->dc, true);
    gpib_dt_init(&ctx->dt);
    ctx->rtl_pulse = false;
    gpib_sr_init(&ctx->sr);
    ctx->device_stb = 0;
    ctx->device_rsv = false;
    /* Device PP1 (F04): armed at init only -- the subset is a static
     * device capability (488.1 Table 27), not a runtime toggle. */
    gpib_pp_init(&ctx->pp, params->pp_capable);
    ctx->pp_ist = false;
    GPIB_TRACE(ctx, GPIB_TRC_INIT, 0, 0, (uint32_t)epoch);
    return GPIB_OK;
}

gpib_status_t gpib_start(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle != GPIB_LC_INITIALIZED) {
        return GPIB_INVALID_STATE; /* default-deny: repeated start (LIF-01) */
    }
    if (!ctx->port->arm(ctx->port->user)) {
        /* LIF-02: the bounded port arm failed; the instance stays
         * INITIALIZED with no persistent port changes and start may be
         * retried after the port condition is fixed. */
        return GPIB_PORT_ERROR;
    }
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    if (ctx->system_controller) {
        /* 488.2 --16.2.8: "At power-on, the controller shall execute the
         * SEND IFC control sequence." Arming the IFC driver here is the
         * System Controller's LEGITIMATE path to CIC: the pulse (T8 hold)
         * walks CIDS -> CADS -> CACS via SIAS during the first process
         * rounds; actions dispatch only once CIC is held. */
        ctx->ifc_request = true;
    }
#endif
    ctx->lifecycle = GPIB_LC_RUNNING;
    GPIB_TRACE(ctx, GPIB_TRC_START, 0, 0, 0);
    return GPIB_OK;
}

gpib_status_t gpib_poll_device_event(gpib_context_t *ctx, uint8_t *cmd_byte) {
    if (ctx == NULL || cmd_byte == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->dev_evt_count == 0) {
        return GPIB_NO_RESULT;
    }
    size_t oldest = (ctx->dev_evt_head + GPIB_CFG_DEVICE_EVENTS -
                     ctx->dev_evt_count) % GPIB_CFG_DEVICE_EVENTS;
    *cmd_byte = ctx->dev_evt[oldest];
    ctx->dev_evt_count--;
    return GPIB_OK;
}

gpib_status_t gpib_request_shutdown(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    switch (ctx->lifecycle) {
    case GPIB_LC_UNINITIALIZED:
        return GPIB_INVALID_HANDLE;
    case GPIB_LC_STOPPING:
        return GPIB_ALREADY_STOPPING; /* idempotent, no second stop (LIF-06) */
    case GPIB_LC_DRAINING:
        return GPIB_ALREADY_STOPPED;
    default:
        ctx->lifecycle = GPIB_LC_STOPPING;
        GPIB_TRACE(ctx, GPIB_TRC_SHUTDOWN, 0, 0, 0);
        return GPIB_OK;
    }
}

gpib_status_t gpib_process(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    switch (ctx->lifecycle) {
    case GPIB_LC_UNINITIALIZED:
        return GPIB_INVALID_HANDLE;
    case GPIB_LC_STOPPING:
        break;
    case GPIB_LC_INITIALIZED:
    case GPIB_LC_DRAINING:
        /* Bounded servicing only; no GPIB I/O. */
        return GPIB_OK;
    case GPIB_LC_RUNNING:
        /* One bounded owner slice: account -> publish -> step (PRG-02). */
        for (int budget = GPIB_CFG_PROCESS_BUDGET; budget > 0; budget--) {
            /* AUD-04: deadlines bind ACCEPTED TRANSACTIONS, not only the
             * dispatched action. An OPEN stream waiting for its next RX
             * buffer or queued behind the session/CIC gates used to run
             * forever although the application set a deadline. Line mode
             * only: byte-pump actions dispatch immediately and their own
             * deadline latch in executor_step covers them. */
            if (gpib_line_mode(ctx)) {
                uint64_t now = ctx->port->now_us(ctx->port->user);
                for (int i = 0; i < GPIB_CFG_MAX_OPERATIONS; i++) {
                    struct gpib_txn_record *tr = &ctx->txn[i];
                    if (tr->state != TXN_OPEN || tr->deadline_us == 0) {
                        continue;
                    }
                    /* A dispatched action carries the same deadline and
                     * ends through the executor's stop path (which also
                     * cuts the in-flight byte safely); do not race it. */
                    if ((ctx->action.state ==
                             (uint8_t)GPIB_ACTION_ACTIVE ||
                         ctx->action.state ==
                             (uint8_t)GPIB_ACTION_PUBLISHED) &&
                        ctx->action.txn_slot == (uint16_t)i) {
                        continue;
                    }
                    if (now < tr->deadline_us) {
                        continue;
                    }
                    if (ctx->line.rx_session_open &&
                        ctx->line.rx_session_slot == (uint16_t)i &&
                        ctx->line.rx_session_gen == tr->generation) {
                        ctx->line.rx_session_open = false;
                    }
                    tr->state = TXN_FINISHING;
                    gpib_ledger_terminalize_txn_blocks(ctx, (uint16_t)i);
                    gpib_ledger_try_publish_txn(ctx, (uint16_t)i, GPIB_OK,
                        GPIB_TERM_TIMEOUT, GPIB_END_NONE);
                }
            }
            if (ctx->action.state == (uint8_t)GPIB_ACTION_QUIESCED) {
                gpib_executor_account(ctx);
            }
            if (ctx->action.state == (uint8_t)GPIB_ACTION_EMPTY ||
                ctx->action.state == (uint8_t)GPIB_ACTION_RESULT_ACCOUNTED) {
                if (!gpib_executor_try_publish(ctx)) {
                    if (ctx->line.seq != (uint8_t)GPIB_LSEQ_IDLE) {
                        /* Sequencer still busy (e.g. LISTEN_CMDS after an
                         * external ATN steal): keep exchanging on the bus. */
                        gpib_lineexec_step(ctx);
                        continue;
                    }
                    if (gpib_line_mode(ctx)) {
                        /* AUD-17: drain the mandatory event queue BEFORE
                         * the idle observation -- HISTORY_LOST and
                         * untagged port facts must be processed between
                         * operations, not only with an ACTIVE action. */
                        gpib_executor_pump_events(ctx,
                            GPIB_CFG_PROCESS_BUDGET);
                        /* Idle bus observation: keep the SRQ monitor and
                         * the simulated peers progressing. */
                        gpib_lineexec_poll_idle(ctx);
                        break;
                    }
                    /* No dispatchable work: still drain one queued port fact
                     * so late IRQ evidence is counted, not silently lost. */
                    gpib_executor_step(ctx);
                    break;
                }
            }
            gpib_executor_step(ctx);
        }
        return GPIB_OK;
    default:
        return GPIB_INVALID_STATE;
    }
    /* STOPPING: force the global stop latch, drain the action, then finish
     * every transaction and apply the DRAINING guard. */
    if (gpib_action_busy(ctx)) {
        if (!ctx->stop_request.active) {
            ctx->stop_request.active = true;
            ctx->stop_request.cause = (uint8_t)GPIB_TERM_SHUTDOWN;
            ctx->stop_request.txn_slot = 0xFFFFu;
        }
        for (int budget = GPIB_CFG_PROCESS_BUDGET; budget > 0; budget--) {
            gpib_executor_step(ctx);
            if (ctx->action.state == (uint8_t)GPIB_ACTION_QUIESCED) {
                break;
            }
        }
    }
    if (ctx->action.state == (uint8_t)GPIB_ACTION_QUIESCED) {
        gpib_executor_account(ctx);
    }
    gpib_term_reason_t cause = ctx->stop_request.active
        ? (gpib_term_reason_t)ctx->stop_request.cause
        : GPIB_TERM_SHUTDOWN;
    for (int i = 0; i < GPIB_CFG_MAX_OPERATIONS; i++) {
        struct gpib_txn_record *t = &ctx->txn[i];
        if (t->state == TXN_OPEN) {
            t->state = TXN_FINISHING;
        }
        if (t->state == TXN_FINISHING) {
            gpib_ledger_terminalize_txn_blocks(ctx, (uint16_t)i);
            gpib_ledger_try_publish_txn(ctx, (uint16_t)i, GPIB_OK, cause,
                GPIB_END_NONE);
        }
    }
    /* Guard of STOPPING -> DRAINING (LIF-07): instance-wide executor
     * quiescence plus terminal records of ALL accepted blocks/transactions;
     * ACTION_QUIESCED of a single action would not be sufficient. */
    if (ctx->port->executor_quiesced(ctx->port->user) &&
        gpib_ledger_all_terminal(ctx)) {
        /* T8-01: leaving the bus is an ELECTRICAL act -- the instance
         * releases every output (SC lines included) exactly once here,
         * so an honest port's safe_detached() can certify the detach
         * without trusting a stub. */
        gpib_lineexec_detach(ctx);
        /* C2: the instance-wide stop latch is fully consumed here (its
         * targets are all terminal by the guard above). */
        ctx->stop_request.active = false;
        ctx->lifecycle = GPIB_LC_DRAINING;
        GPIB_TRACE(ctx, GPIB_TRC_DRAINING, 0, 0, 0);
    }
    return GPIB_OK;
}

gpib_status_t gpib_deinit(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    switch (ctx->lifecycle) {
    case GPIB_LC_UNINITIALIZED:
        return GPIB_INVALID_HANDLE;
    case GPIB_LC_RUNNING:
        /* Requires shutdown + quiescence + ACKs first (REQ-076). */
        return GPIB_INVALID_STATE;
    case GPIB_LC_STOPPING:
        /* Even with guards satisfied, the owner must publish DRAINING first;
         * no hidden transition inside deinit (default-deny table). */
        return busy(ctx, GPIB_BUSY_STOP_TRANSITION_PENDING);
    case GPIB_LC_DRAINING:
        if (gpib_ledger_unacked(ctx) != 0) {
            return busy(ctx, GPIB_BUSY_UNACK_RESULTS);
        }
        if (!ctx->port->safe_detached(ctx->port->user)) {
            return busy(ctx, GPIB_BUSY_PORT_TEARDOWN_PENDING);
        }
        GPIB_TRACE(ctx, GPIB_TRC_DEINIT, 0, 0, 0);
        wipe(ctx);
        return GPIB_OK;
    case GPIB_LC_INITIALIZED:
        if (!ctx->port->safe_detached(ctx->port->user)) {
            return busy(ctx, GPIB_BUSY_PORT_TEARDOWN_PENDING);
        }
        GPIB_TRACE(ctx, GPIB_TRC_DEINIT, 0, 0, 0);
        wipe(ctx);
        return GPIB_OK;
    default:
        return GPIB_INVALID_STATE;
    }
}

gpib_status_t gpib_set_address(gpib_context_t *ctx, gpib_address_t address) {    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle != GPIB_LC_INITIALIZED && ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE;
    }
    if (!gpib_address_supported(&address)) {
        return GPIB_UNSUPPORTED_ADDRESSING; /* active config untouched */
    }
    if (ctx->lifecycle == GPIB_LC_RUNNING &&
        (ctx->action.state != (uint8_t)GPIB_ACTION_EMPTY &&
         ctx->action.state != (uint8_t)GPIB_ACTION_RESULT_ACCOUNTED)) {
        /* F11: never move the addressing functions mid-exchange -- the
         * in-flight handshake references the current address state. */
        return GPIB_BUSY_RESOURCES;
    }
    if (ctx->line.rx_session_open) {
        /* R6-03: an OPEN READ stream holds the addressing latched in the
         * remote talker/listener pair -- re-addressing ourselves between
         * blocks would break the one-RECEIVE-per-response invariant. */
        return GPIB_BUSY_RESOURCES;
    }
    ctx->own_address = address;
    gpib_apply_own_address(ctx); /* every own-address automaton moves with it */
    return GPIB_OK;
}

gpib_status_t gpib_pp_set_ist(gpib_context_t *ctx, bool ist) {
    /* Device PP1 individual status (F04): the application's device-side
     * fact sampled at each IDY; ist == S drives the assigned PPR line
     * true, otherwise passive false (Table 25). */
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (!ctx->pp.capable) {
        return GPIB_UNSUPPORTED; /* PP0: no parallel poll capability */
    }
    ctx->pp_ist = ist;
    return GPIB_OK;
}

gpib_status_t gpib_poll_df_event(gpib_context_t *ctx, gpib_df_event_t *ev) {
    if (ctx == NULL || ev == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->df_evt_count == 0) {
        return GPIB_NO_RESULT;
    }
    size_t oldest = (ctx->df_evt_head + GPIB_CFG_DF_EVENTS -
                     ctx->df_evt_count) % GPIB_CFG_DF_EVENTS;
    ev->kind = (gpib_df_event_kind_t)ctx->df_evt[oldest].kind;
    ev->origin = ctx->df_evt[oldest].origin;
    ev->rl_state = (gpib_rl_state_t)ctx->df_evt[oldest].rl_state;
    ctx->df_evt_count--;
    return GPIB_OK;
}

gpib_status_t gpib_device_local(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle != GPIB_LC_INITIALIZED && ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE;
    }
    /* --2.8.5: rtl must be a non-steady request; the pulse is consumed by
     * the next RL step (ignored under local lockout by the FSM itself). */
    ctx->rtl_pulse = true;
    return GPIB_OK;
}

gpib_status_t gpib_device_set_stb(gpib_context_t *ctx, uint8_t stb) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    ctx->device_stb = (uint8_t)(stb & 0xBFu); /* B6 belongs to RQS */
    return GPIB_OK;
}

gpib_status_t gpib_device_service_request(gpib_context_t *ctx, bool rsv) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    ctx->device_rsv = rsv;
    return GPIB_OK;
}

gpib_status_t gpib_interface_clear(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (!ctx->system_controller) {
        return GPIB_UNSUPPORTED; /* C2: system controller only */
    }
    if (ctx->lifecycle != GPIB_LC_INITIALIZED &&
        ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE;
    }
    /* Arm the IFC driver group: SACS via rsc, SIAS via sic. The pulse is
     * applied on the next bus exchange and held for T8 by the FSM. */
    ctx->ifc_request = true;
    return GPIB_OK;
}

gpib_status_t gpib_remote_enable(gpib_context_t *ctx, bool enable) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (!ctx->system_controller) {
        return GPIB_UNSUPPORTED; /* C3: system controller only */
    }
    if (ctx->lifecycle != GPIB_LC_INITIALIZED &&
        ctx->lifecycle != GPIB_LC_RUNNING) {
        /* D04 (independent audit): default-deny -- a STOPPING/DRAINING
         * instance must not accept protocol requests; the old code
         * returned OK and mutated ren_request with no electrical
         * effect, misleading the diagnostics. */
        return GPIB_INVALID_STATE;
    }
    ctx->ren_request = enable;
    return GPIB_OK;
}

gpib_status_t gpib_query_status(gpib_context_t *ctx, gpib_status_info_t *out) {    if (ctx == NULL || out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    out->lifecycle = ctx->lifecycle;
    out->fault_latched = ctx->fault_latched;
    out->instance_epoch = ctx->epoch;
    out->last_busy_reason = ctx->last_busy_reason;
    out->open_transactions = 0;
    out->accepted_blocks = 0;
    out->unacked_results = gpib_ledger_unacked(ctx);
    out->srq_active = ctx->line.srq_seen;
    for (int i = 0; i < GPIB_CFG_MAX_OPERATIONS; i++) {
        gpib_txn_state_t st = ctx->txn[i].state;
        if (st == TXN_RESERVED || st == TXN_OPEN || st == TXN_FINISHING) {
            out->open_transactions++;
        }
    }
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        if (ctx->block[i].state != BLOCK_APP_OWNED) {
            out->accepted_blocks++;
        }
    }
    return GPIB_OK;
}

gpib_status_t gpib_recover(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    switch (ctx->lifecycle) {
    case GPIB_LC_UNINITIALIZED:
        return GPIB_INVALID_HANDLE;
    case GPIB_LC_RUNNING:
        /* Recovery guard (RECOVERY_CONTRACT --3): only with a provably closed
         * old action, no latched fault and nothing quarantined. The content
         * of local recovery policies stays [O]. */
        if (ctx->fault_latched || ctx->quarantined_blocks != 0 ||
            gpib_action_busy(ctx)) {
            return GPIB_RECOVERY_NOT_ALLOWED;
        }
        GPIB_TRACE(ctx, GPIB_TRC_RECOVER, 0, 0, 0);
        return GPIB_OK;
    case GPIB_LC_INITIALIZED:
        return GPIB_INVALID_STATE; /* default-deny: nothing runs yet */
    default:
        return GPIB_RECOVERY_NOT_ALLOWED; /* STOPPING/DRAINING */
    }
}

gpib_status_t gpib_emergency_stop(gpib_context_t *ctx) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    switch (ctx->lifecycle) {
    case GPIB_LC_UNINITIALIZED:
        return GPIB_INVALID_HANDLE;
    case GPIB_LC_STOPPING:
        return GPIB_ALREADY_STOPPING; /* idempotent, no second inhibit */
    case GPIB_LC_DRAINING:
        return GPIB_ALREADY_STOPPED;  /* already quiesced, no new IO */
    default:
        break;
    }
    /* Electrical release of own outputs only: never an implicit IFC/DCL
     * (REQ-042, RECOVERY_CONTRACT --3); results carry uncertainty flags. */
    gpib_fault_latch(ctx, GPIB_FAULT_EMERGENCY_STOP);
    ctx->stop_request.active = true;
    ctx->stop_request.cause = (uint8_t)GPIB_TERM_FAULT;
    ctx->stop_request.txn_slot = 0xFFFFu;
    ctx->port->emergency_inhibit(ctx->port->user);
    ctx->lifecycle = GPIB_LC_STOPPING;
    GPIB_TRACE(ctx, GPIB_TRC_EMERGENCY, 0, 0, 0);
    return GPIB_OK;
}

gpib_status_t gpib_get_fault_snapshot(gpib_context_t *ctx,
    gpib_fault_snapshot_t *out) {
    if (ctx == NULL || out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    /* Fixed zombie diagnostic per RECOVERY_CONTRACT --4; works without
     * dynamic allocation, bus IO, or waiting. */
    memset(out, 0, sizeof(*out));
    out->instance_epoch = ctx->epoch;
    out->lifecycle = ctx->lifecycle;
    out->fault_latched = ctx->fault_latched;
    out->fault_code = ctx->fault_code;
    out->first_fault_us = ctx->first_fault_us;
    out->last_update_us = ctx->last_update_us;
    out->active_action_id = ctx->action.generation;
    out->action_state = (gpib_action_state_t)ctx->action.state;
    out->action_kind = (gpib_action_kind_t)ctx->action.kind;
    out->stop_reason = (gpib_term_reason_t)ctx->action.stop_cause;
    out->action_quiesced = !gpib_action_busy(ctx);
    out->executor_quiesced = out->action_quiesced &&
        ctx->port->executor_quiesced(ctx->port->user);
    out->accepted_blocks = 0;
    out->unacked_blocks = gpib_ledger_unacked(ctx);
    out->quarantined_blocks = ctx->quarantined_blocks;
    out->srq_events = ctx->srq_events;
    out->dropped_stale_events = ctx->dropped_stale_events;
    out->device_events_dropped = ctx->dev_evt_dropped;
    out->command_valid = ctx->action.command_valid;
    out->command_byte = ctx->action.command_byte;
    out->snapshot_valid = true; /* host model data is self-consistent */
    return GPIB_OK;
}
