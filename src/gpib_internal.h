#ifndef GPIB_INTERNAL_H
#define GPIB_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_storage.h"

/* Internal helpers. All entry points run on the single protocol-owner context
 * only (REQ-021, PROGRESS_CONTRACT --2): no cross-context mutation happens in
 * this increment; IRQ/executor paths arrive with the mailbox work (OPEN-03). */

/* Instance identity manager (R14/REQ-072). */
gpib_status_t gpib_epoch_next(uint64_t *out);

/* Free-slot search only; the caller commits both reservations atomically
 * (RUNNING submit row: txn record + slots of all offered blocks, or nothing). */
int gpib_ledger_find_free_txn(const struct gpib_context *ctx);
int gpib_ledger_find_free_block(const struct gpib_context *ctx);

/* Validated slot lookup: epoch/kind/slot-range/generation must all match.
 * A miss never touches the slot occupied by a newer generation. */
int gpib_ledger_txn_slot(const struct gpib_context *ctx, gpib_handle_t h);
int gpib_ledger_block_slot(const struct gpib_context *ctx, gpib_handle_t h);

gpib_handle_t gpib_make_handle(const struct gpib_context *ctx, uint8_t kind,
                               uint16_t slot, uint32_t generation);

/* Terminalize all still-queued correlated blocks as NOT_STARTED. Valid in
 * this increment because no executor exists: never_dispatched is provable
 * for every CORE_QUEUED block (FIX-02/REQ-064); no fake ACTION_QUIESCED. */
void gpib_ledger_terminalize_txn_blocks(struct gpib_context *ctx,
    uint16_t txn_slot);

/* Publish the single transaction terminal record; guarded by
 * blocks_terminal == block_count (TXN_FINISHING row). Block ACKs are not
 * waited for (REQ-033, api_contract --8). */
void gpib_ledger_try_publish_txn(struct gpib_context *ctx, uint16_t txn_slot,
    gpib_status_t status, gpib_term_reason_t reason, gpib_end_cause_t end);

/* STOPPING -> DRAINING guard ingredients (LIF-07). */
bool gpib_ledger_all_terminal(const struct gpib_context *ctx);
size_t gpib_ledger_unacked(const struct gpib_context *ctx);

/* REQ-078/079: address form support of this build (TE/LE absent). */
bool gpib_address_supported(const gpib_address_t *a);

/* --- mailbox host model (executor.c) ------------------------------------- */

/* Publish the next queued block of an open, non-stop-latched transaction.
 * Returns true when an action was published. Linearization: the release
 * transition EMPTY -> PUBLISHED (MBX --3). */
bool gpib_executor_try_publish(struct gpib_context *ctx);

/* One bounded executor step: consume at most one port event, honour the stop
 * latch, move at most a bounded number of bytes. Returns true while the
 * current action still has work. */
bool gpib_executor_step(struct gpib_context *ctx);

/* Owner-side accounting of a quiesced action: exactly one terminal block
 * record (unless quarantined), then RESULT_ACCOUNTED. */
void gpib_executor_account(struct gpib_context *ctx);

/* Transaction bookkeeping after a block became terminal: publishes the
 * transaction terminal record once all its accepted blocks are terminal. */
void gpib_txn_after_block_terminal(struct gpib_context *ctx, uint16_t slot);

/* Latch an instance fault with timestamps (RECOVERY_CONTRACT --4). */
void gpib_fault_latch(struct gpib_context *ctx, gpib_fault_code_t code);

/* True when the action slot holds unfinished work. */
bool gpib_action_busy(const struct gpib_context *ctx);

/* --- line-level engine (lineexec.c) --------------------------------------- */

enum gpib_lseq {
    GPIB_LSEQ_IDLE = 0,
    GPIB_LSEQ_CMD,
    GPIB_LSEQ_DATA_TX,
    GPIB_LSEQ_DATA_RX,
    GPIB_LSEQ_EPILOGUE,     /* command epilogue after the data phase */
    GPIB_LSEQ_LISTEN_CMDS,
    GPIB_LSEQ_DEV_WAIT,     /* device role: wait for addressing, exchange */
    GPIB_LSEQ_POLL_ADDR,    /* ALLSPOLL/FINDRQS: MTA(next) with SPE armed */
    GPIB_LSEQ_DATA_TX_PENDING, /* RESET: MLA staged, switch to DATA_TX */
    GPIB_LSEQ_PPOLL_IDY,    /* PPOLL: ATN+EOI asserted, wait T6, read PPR */
    GPIB_LSEQ_FINDLSTN_PROBE /* FINDLSTN: send MLA, observe NDAC */
};

/* True when the port provides the line slice. */
bool gpib_line_mode(const struct gpib_context *ctx);

/* Build the controller command prologue for the published action:
 * TX to listeners: UNL, MLA*, MTA(own); RX from device: UNL, MLA(own),
 * MTA(device); serial poll: SPE, MTA(device) with epilogue SPD, UNT.
 * Marks the action PUBLISHED -> sequencer CMD. */
/* Reconfigure every own-address automaton (init, set_address, IFC reset). */
void gpib_apply_own_address(struct gpib_context *ctx);

/* Off-bus electrical detach: releases EVERY output including the SC pair.
 * Called once at the STOPPING->DRAINING boundary (T8-01). */
void gpib_lineexec_detach(struct gpib_context *ctx);

/* AUD-10: command-action end keeps ATN held (16.2.1). */
void gpib_lineexec_release_hold_atn(struct gpib_context *ctx);

/* AUD-17: bounded drain of the port event queue from idle/queued
 * rounds. Consumes HISTORY_LOST and untagged facts; stale tagged facts
 * are discarded by generation. */
void gpib_executor_pump_events(struct gpib_context *ctx, int budget);

/* A02: reset the device-side interface functions on an UNTAGGED queued
 * IFC fact (works with and without a live action). */
void gpib_lineexec_external_ifc(struct gpib_context *ctx);

/* The drive composer (R7-01): merges the persistent SC lines into every
 * exchange-scoped drive. Exported for the executor's stop path (C3). */
void gpib_lineexec_drive_out(struct gpib_context *ctx,
    gpib_line_drive_t *req);

void gpib_lineexec_begin(struct gpib_context *ctx);
/* One bounded bus exchange: read snapshot, step SH/AH, apply drives.
 * Reports completion by setting action stop_cause and quiescing it (the
 * regular accounting path then produces the terminal record). */
void gpib_lineexec_step(struct gpib_context *ctx);

/* Idle bus observation: one snapshot exchange to keep the SRQ monitor and
 * the simulated peers progressing while no action is dispatched. */
void gpib_lineexec_poll_idle(struct gpib_context *ctx);

/* Release all lines (used when the action ends or the instance stops). */
void gpib_lineexec_release(struct gpib_context *ctx);

/* --- binary trace (trace.c) ----------------------------------------------- */

#if GPIB_CFG_TRACE_EVENTS > 0
void gpib_trace_write(struct gpib_context *ctx, uint16_t kind, uint16_t a,
    uint16_t b, uint32_t c);
#define GPIB_TRACE(ctx, kind, a, b, c) \
    gpib_trace_write(ctx, (uint16_t)(kind), (uint16_t)(a), (uint16_t)(b), \
        (uint32_t)(c))
#else
#define GPIB_TRACE(ctx, kind, a, b, c) ((void)(ctx))
#endif

#endif /* GPIB_INTERNAL_H */
