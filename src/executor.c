#include <stddef.h>
#include <string.h>

#include "gpib_internal.h"

#define STOP_TARGET_ALL 0xFFFFu

bool gpib_action_busy(const struct gpib_context *ctx) {
    uint8_t st = ctx->action.state;
    return st == (uint8_t)GPIB_ACTION_PUBLISHED || st == (uint8_t)GPIB_ACTION_ACTIVE;
}

bool gpib_executor_try_publish(struct gpib_context *ctx) {
    if (gpib_action_busy(ctx)) {
        return false;
    }
    if (ctx->action.state == (uint8_t)GPIB_ACTION_QUIESCED) {
        return false; /* must be accounted before the slot is reusable */
    }
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    if (gpib_line_mode(ctx) &&
        (ctx->line.ctrl.main == GPIB_C_CIDS ||
         ctx->line.ctrl.ifc == GPIB_C_SIAS)) {
        /* No CIC, or the power-on/re-seize IFC pulse is still on the wire:
         * "The controller shall not execute any other control sequences
         * until IFC has been made FALSE" (488.2 16.2.8). The action stays
         * CORE_QUEUED until the FSM holds CIC with the pulse completed
         * (byte-pump ports have no FSM stepping and are not gated). */
        return false;
    }
#endif
    for (int t = 0; t < GPIB_CFG_MAX_OPERATIONS; t++) {
        struct gpib_txn_record *tr = &ctx->txn[t];
        if (tr->state != TXN_OPEN) {
            continue;
        }
        if (ctx->stop_request.active &&
            (ctx->stop_request.txn_slot == (uint16_t)t ||
             ctx->stop_request.txn_slot == STOP_TARGET_ALL)) {
            continue; /* stopping targets this transaction: no new dispatch */
        }
        for (int b = 0; b < GPIB_CFG_MAX_BLOCKS; b++) {
            struct gpib_block_record *br = &ctx->block[b];
            if (br->state != BLOCK_CORE_QUEUED ||
                br->txn_slot != (uint16_t)t ||
                br->txn_generation != tr->generation) {
                continue;
            }
            struct gpib_action *a = &ctx->action;
            /* Generations never repeat within an instance: monotonic
             * increment, so stale tagged events cannot alias a live action
             * (MBX-07). */
            uint16_t gen = a->generation;
            memset(a, 0, sizeof(*a));
            a->generation = (uint16_t)(gen + 1u);
            a->state = (uint8_t)GPIB_ACTION_PUBLISHED;
            a->block_slot = (uint16_t)b;
            a->block_generation = br->generation;
            a->txn_slot = (uint16_t)t;
            a->kind = (tr->kind == GPIB_REQ_READ ||
                       tr->kind == GPIB_REQ_SPOLL ||
                       tr->kind == GPIB_REQ_DEV_RX)
                          ? (uint8_t)GPIB_ACTION_RX_BLOCK
                          : (uint8_t)GPIB_ACTION_TX_BLOCK;
            a->data = br->data;
            a->len = br->len;
            a->deadline_us = tr->deadline_us;
            /* Per-block capacity: stream-level completion (rx_limit/EOI) is
             * decided by the owner in txn_after_block_terminal. */
            a->limit = br->len;
            br->state = BLOCK_EXECUTOR_IN_USE; /* dispatch linearization */
            GPIB_TRACE(ctx, GPIB_TRC_PUBLISH, a->generation, (uint16_t)b, 0);
            return true;
        }
    }
    return false;
}

/* Quarantine: mandatory edge history is unrecoverable, so no safe terminal
 * result exists; the pinned block stays in BLOCK_WAIT_QUIESCENCE forever
 * (LED-04/QSC-02) and the instance escalates to STOPPING with a fault. */
static void fault_history_lost(struct gpib_context *ctx) {
    struct gpib_action *a = &ctx->action;
    /* Only a live action holds a pinned block; an idle action must NOT be
     * marked QUIESCED, otherwise owner accounting would fabricate a result
     * for a stale block_slot. */
    if (a->state == (uint8_t)GPIB_ACTION_PUBLISHED ||
        a->state == (uint8_t)GPIB_ACTION_ACTIVE) {
        struct gpib_block_record *b = &ctx->block[a->block_slot];
        if (b->state == BLOCK_EXECUTOR_IN_USE) {
            b->state = BLOCK_WAIT_QUIESCENCE;
            b->quarantined = true;
            ctx->quarantined_blocks++;
            GPIB_TRACE(ctx, GPIB_TRC_QUARANTINE, a->block_slot, a->generation, 0);
        }
        a->state = (uint8_t)GPIB_ACTION_QUIESCED; /* no further access */
    }
    gpib_fault_latch(ctx, GPIB_FAULT_HISTORY_LOST);
    if (ctx->lifecycle == GPIB_LC_RUNNING ||
        ctx->lifecycle == GPIB_LC_INITIALIZED) {
        ctx->lifecycle = GPIB_LC_STOPPING;
    }
    ctx->stop_request.active = true;
    ctx->stop_request.cause = (uint8_t)GPIB_TERM_FAULT;
    ctx->stop_request.txn_slot = STOP_TARGET_ALL;
}

static void action_quiesce(struct gpib_context *ctx) {
    ctx->action.state = (uint8_t)GPIB_ACTION_QUIESCED;
    /* Deferred host model: quiescence proof is synchronous; a real port must
     * prove it via the async fence procedure (MBX --5). */
}

static void consume_event(struct gpib_context *ctx, const gpib_port_event_t *ev) {
    struct gpib_action *a = &ctx->action;
    if (ev->type == (uint8_t)GPIB_PEVT_HISTORY_LOST) {
        fault_history_lost(ctx);
        return;
    }
    /* Untagged events (IFC from the line) apply globally; tagged events must
     * match the live action generation, else they are stale IRQ facts. */
    if (ev->tag != 0 && ev->tag != a->generation) {
        ctx->dropped_stale_events++; /* old front may not act for new action */
        GPIB_TRACE(ctx, GPIB_TRC_STALE_DROP, ev->tag, a->generation,
            (uint32_t)ctx->dropped_stale_events);
        return;
    }
    if (ev->type == (uint8_t)GPIB_PEVT_DEVICE_CMD) {
        /* Raw observed command byte for this device: latched for the
         * application, unclassified, never affecting the running action
         * (REQ-016 structural part; classification awaits OPEN-01/02). */
        ctx->dev_evt[ctx->dev_evt_head] = ev->data;
        ctx->dev_evt_head = (ctx->dev_evt_head + 1u) % GPIB_CFG_DEVICE_EVENTS;
        if (ctx->dev_evt_count < GPIB_CFG_DEVICE_EVENTS) {
            ctx->dev_evt_count++;
        } else {
            ctx->dev_evt_dropped++;
        }
        GPIB_TRACE(ctx, GPIB_TRC_DEVICE_CMD, 0, 0, ev->data);
        return;
    }
    bool live = a->state == (uint8_t)GPIB_ACTION_PUBLISHED ||
                a->state == (uint8_t)GPIB_ACTION_ACTIVE;
    if (!live) {
        /* No action is dispatched: an SRQ fact is still counted; any action
         * scoped fact (deadline/ATN/EOI/IFC aimed at a finished action) is a
         * late IRQ and may not touch retired blocks. */
        if (ev->type == (uint8_t)GPIB_PEVT_SRQ) {
            ctx->srq_events++;
        } else {
            ctx->dropped_stale_events++;
        }
        return;
    }
    switch (ev->type) {
    case (uint8_t)GPIB_PEVT_ATN_COMMAND:
        if (a->stop_cause == (uint8_t)GPIB_TERM_NONE) {
            /* P1 (R12): data phase ends here; the command byte is captured
             * for the owner and never enters the RX payload. */
            a->stop_cause = (uint8_t)GPIB_TERM_PHASE_INTERRUPTED;
            a->command_valid = true;
            a->command_byte = ev->data;
        }
        break;
    case (uint8_t)GPIB_PEVT_IFC:
        if (a->stop_cause == (uint8_t)GPIB_TERM_NONE) {
            a->stop_cause = (uint8_t)GPIB_TERM_IFC;
        }
        break;
    case (uint8_t)GPIB_PEVT_SRQ:
        ctx->srq_events++; /* sideband latch: must not terminate the block */
        GPIB_TRACE(ctx, GPIB_TRC_SRQ, 0, 0, (uint32_t)ctx->srq_events);
        break;
    case (uint8_t)GPIB_PEVT_DEADLINE:
        if (a->stop_cause == (uint8_t)GPIB_TERM_NONE) {
            a->stop_cause = (uint8_t)GPIB_TERM_TIMEOUT;
        }
        break;
    case (uint8_t)GPIB_PEVT_EOI:
        if (a->stop_cause == (uint8_t)GPIB_TERM_NONE &&
            a->kind == (uint8_t)GPIB_ACTION_RX_BLOCK) {
            a->stop_cause = (uint8_t)GPIB_TERM_COMPLETED;
            a->eoi_terminated = true;
        }
        break;
    default:
        break;
    }
}

bool gpib_executor_step(struct gpib_context *ctx) {
    struct gpib_action *a = &ctx->action;
    const struct gpib_port *port = ctx->port;

    gpib_port_event_t ev;
    if (port->poll_event(port->user, &ev)) {
        consume_event(ctx, &ev);
        if (a->state == (uint8_t)GPIB_ACTION_QUIESCED) {
            return false; /* HISTORY_LOST already resolved the action */
        }
    }
    /* Instance stop latch has priority over new execution (MBX-01). */
    if (ctx->stop_request.active &&
        (ctx->stop_request.txn_slot == a->txn_slot ||
         ctx->stop_request.txn_slot == STOP_TARGET_ALL ||
         ctx->stop_request.cause != (uint8_t)GPIB_TERM_CANCELED) &&
        a->stop_cause == (uint8_t)GPIB_TERM_NONE) {
        a->stop_cause = ctx->stop_request.cause;
    }
    /* Structural deadline (REQ-025): an expired deadline ends the action
     * even when the owner services it late; success is never fabricated
     * from a missed check (PROGRESS_CONTRACT --6). Numeric norms stay [O]. */
    if (a->stop_cause == (uint8_t)GPIB_TERM_NONE && a->deadline_us != 0 &&
        port->now_us(port->user) >= a->deadline_us) {
        a->stop_cause = (uint8_t)GPIB_TERM_TIMEOUT;
        GPIB_TRACE(ctx, GPIB_TRC_DEADLINE_EXPIRE, a->generation, 0,
            (uint32_t)a->committed);
    }

    if (a->state == (uint8_t)GPIB_ACTION_PUBLISHED) {
        if (a->stop_cause != (uint8_t)GPIB_TERM_NONE) {
            /* Stop won before acceptance: the action never becomes ACTIVE
             * and still produces exactly one completion (MBX-01). */
            action_quiesce(ctx);
            gpib_lineexec_release(ctx);
            return false;
        }
        if (gpib_line_mode(ctx)) {
            gpib_lineexec_begin(ctx);
        }
        a->state = (uint8_t)GPIB_ACTION_ACTIVE;
        GPIB_TRACE(ctx, GPIB_TRC_ACCEPT, a->generation, 0, 0);
        return true;
    }
    if (a->state != (uint8_t)GPIB_ACTION_ACTIVE) {
        return false;
    }
    if (a->stop_cause != (uint8_t)GPIB_TERM_NONE) {
        a->state = (uint8_t)GPIB_ACTION_STOP_REQUESTED;
        gpib_lineexec_release(ctx);
        action_quiesce(ctx);
        return false;
    }
    if (gpib_line_mode(ctx)) {
        gpib_lineexec_step(ctx);
        return a->state == (uint8_t)GPIB_ACTION_ACTIVE;
    }
    /* Bounded byte step of the host model; `committed` counts only bytes
     * with completed (simulated) handshake, never observed-only data. */
    uint8_t *buf = (uint8_t *)(uintptr_t)a->data + a->committed;
    size_t cap = a->len - a->committed;
    gpib_dir_t dir = (a->kind == (uint8_t)GPIB_ACTION_TX_BLOCK) ? GPIB_DIR_TX
                                                                : GPIB_DIR_RX;
    size_t moved = port->transfer(port->user, dir, buf, cap);
    if (moved > cap) {
        moved = cap;
    }
    a->committed += moved;
    if (a->limit != 0 && a->committed >= a->limit) {
        a->stop_cause = (uint8_t)GPIB_TERM_COMPLETED;
        a->state = (uint8_t)GPIB_ACTION_STOP_REQUESTED;
        GPIB_TRACE(ctx, GPIB_TRC_COMPLETE, a->generation, 0,
            (uint32_t)a->committed);
        action_quiesce(ctx);
        return false;
    }
    return true;
}

void gpib_executor_account(struct gpib_context *ctx) {
    struct gpib_action *a = &ctx->action;
    struct gpib_block_record *b = &ctx->block[a->block_slot];
    if (a->state != (uint8_t)GPIB_ACTION_QUIESCED) {
        return;
    }
    gpib_term_reason_t cause = (gpib_term_reason_t)a->stop_cause;
    if (cause == GPIB_TERM_NONE) {
        cause = GPIB_TERM_COMPLETED; /* defensive: quiesced implies a cause */
    }
    a->state = (uint8_t)GPIB_ACTION_RESULT_ACCOUNTED;
    GPIB_TRACE(ctx, GPIB_TRC_ACCOUNT, a->block_slot, a->generation,
        (uint32_t)cause);
    if (b->quarantined) {
        return; /* no safe result exists; block stays pinned (LED-04) */
    }
    b->state = BLOCK_WAIT_QUIESCENCE;
    /* ACTION_QUIESCED proven synchronously in this host model: */
    b->state = BLOCK_ACCOUNTABLE;
    b->result.status = GPIB_OK;
    b->result.reason = cause;
    if (cause == GPIB_TERM_COMPLETED) {
        if (a->eoi_terminated) {
            b->result.end_cause = GPIB_END_EOI;
        } else if (a->char_terminated) {
            b->result.end_cause = GPIB_END_CHAR;
        } else {
            b->result.end_cause = GPIB_END_LENGTH;
        }
    } else {
        b->result.end_cause = GPIB_END_NONE;
    }
    b->result.bytes_confirmed = a->committed;
    b->result.last_byte_uncertain =
        (cause == GPIB_TERM_FAULT) || a->last_byte_uncertain;
    if (ctx->txn[a->txn_slot].kind == GPIB_REQ_SPOLL) {
        /* The staged byte IS the status byte of the serial poll. */
        b->result.stb = ctx->txn[a->txn_slot].spoll_stb;
    }
    b->state = BLOCK_RESULT_READY;
    ctx->txn[a->txn_slot].blocks_terminal++;
    ctx->txn[a->txn_slot].stream_bytes += a->committed;
    if (ctx->txn[a->txn_slot].kind == GPIB_REQ_ALLSPOLL ||
        ctx->txn[a->txn_slot].kind == GPIB_REQ_FINDRQS) {
        /* Pass-wide byte count: polls actually performed; FINDRQS also
         * resolves the caller's found-index output. */
        ctx->txn[a->txn_slot].stream_bytes =
            ctx->txn[a->txn_slot].poll_index + 1u;
        if (ctx->txn[a->txn_slot].poll_index >=
            ctx->txn[a->txn_slot].poll_count) {
            ctx->txn[a->txn_slot].stream_bytes =
                ctx->txn[a->txn_slot].poll_count;
        }
        if (ctx->txn[a->txn_slot].find_out != NULL) {
            *ctx->txn[a->txn_slot].find_out =
                ctx->txn[a->txn_slot].find_index;
        }
    }
    if (ctx->txn[a->txn_slot].kind == GPIB_REQ_PPOLL) {
        b->result.stb = ctx->txn[a->txn_slot].ppr_byte;
    }
    gpib_txn_after_block_terminal(ctx, a->txn_slot);
}

void gpib_txn_after_block_terminal(struct gpib_context *ctx, uint16_t slot) {
    struct gpib_txn_record *t = &ctx->txn[slot];
    if (t->state != TXN_OPEN && t->state != TXN_FINISHING) {
        return;
    }
    /* Single scan over correlated terminal records; state check matters
     * because free slots carry zeroed correlation fields. */
    gpib_term_reason_t cause = GPIB_TERM_NONE;
    gpib_end_cause_t end = GPIB_END_NONE;
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        struct gpib_block_record *b = &ctx->block[i];
        if (b->state != BLOCK_RESULT_READY || b->txn_slot != slot ||
            b->txn_generation != t->generation) {
            continue;
        }
        if (b->result.reason != GPIB_TERM_COMPLETED &&
            b->result.reason != GPIB_TERM_NOT_STARTED && cause == GPIB_TERM_NONE) {
            cause = b->result.reason;
        }
        if (b->result.end_cause == GPIB_END_EOI) {
            end = GPIB_END_EOI;
        }
        if (b->result.end_cause == GPIB_END_CHAR) {
            end = GPIB_END_CHAR;
        }
    }
    bool all_known_terminal = (t->blocks_terminal == t->block_count);
    if (cause == GPIB_TERM_NONE && t->kind == GPIB_REQ_READ) {
        /* READ streams close on the declared count, on EOI or on the
         * 16.2.6 stop character; a drained block queue without any of
         * those is controlled backpressure, not completion (REQ-026/035). */
        if (end == GPIB_END_EOI || end == GPIB_END_CHAR) {
            cause = GPIB_TERM_COMPLETED; /* end stays as matched */
        } else if (all_known_terminal && t->rx_limit != 0 &&
                   t->stream_bytes >= t->rx_limit) {
            cause = GPIB_TERM_COMPLETED;
            end = GPIB_END_LENGTH;
        } else {
            return; /* stay open: stream continues or backpressure pause */
        }
    } else if (cause == GPIB_TERM_NONE) {
        /* WRITE/WRITE_MANY/DEV_TX/DEV_RX: auto-close once every accepted
         * block is terminal (stream end semantics [P], OPEN-23); the end
         * cause inherits EOI from the terminal blocks if any. */
        if (!all_known_terminal) {
            return;
        }
        cause = GPIB_TERM_COMPLETED;
        end = GPIB_END_LENGTH;
        for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
            struct gpib_block_record *b = &ctx->block[i];
            if (b->state == BLOCK_RESULT_READY && b->txn_slot == slot &&
                b->txn_generation == t->generation &&
                b->result.end_cause == GPIB_END_EOI) {
                end = GPIB_END_EOI;
                break;
            }
        }
    }
    t->state = TXN_FINISHING;
    gpib_ledger_terminalize_txn_blocks(ctx, slot);
    gpib_ledger_try_publish_txn(ctx, slot, GPIB_OK, cause, end);
}
