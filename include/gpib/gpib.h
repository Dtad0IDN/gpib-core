#ifndef GPIB_H
#define GPIB_H

#include <stdbool.h>
#include <stddef.h>

#include "gpib_cfg.h"
#include "gpib_proto.h"
#include "gpib_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file gpib.h
 * @brief Public semantic API of the portable level-3 GPIB core.
 *
 * Semantic names only: the final C ABI, struct layouts and numeric codes
 * remain open ([O] OPEN-05/26). All functions are non-blocking, never
 * allocate, and run on the single protocol-owner context (REQ-021); the
 * ISR-safe subset is emergency stop and the port evidence path only.
 */

/* Opaque instance context; the typed storage object for static placement is
 * defined by gpib_storage.h ([P] per configuration.md --7). */
typedef struct gpib_context gpib_context_t;

struct gpib_port; /* level-2 adapter contract, see gpib_port.h */

/** Immutable instance parameters (REQ-003). */
typedef struct gpib_init_params {
    bool system_controller;     /**< immutable after init */
    gpib_address_t own_address; /**< own address form (Device listen/talk) */
    const struct gpib_port *port; /**< NULL selects the host null port */
    bool pp_capable; /**< device-role PP1 responder armed at init (F04);
                          false = PP0: never answers a parallel poll */
} gpib_init_params_t;

/** One externally submitted operation request. */
typedef struct gpib_request {
    gpib_request_kind_t kind;
    gpib_address_t address;         /* remote address for WRITE/READ */
    const gpib_address_t *listeners; /* WRITE_MANY target list */
    size_t listener_count;
    const void *tx_data;            /* first zero-copy TX block (borrowed) */
    size_t tx_len;                  /* zero-length blocks: policy [O] OPEN-23 */
    void *rx_data;                  /* first zero-copy RX block (borrowed) */
    size_t rx_cap;
    size_t rx_min_bytes;            /* stream target for READ (REQ-035) */
    uint64_t deadline_us;           /* absolute logical us, 0 = none ([P]) */
    const uint8_t *cmd_list;        /* SEND COMMAND: ATN command bytes */
    size_t cmd_len;
    size_t *find_out;               /* FINDRQS: result index output */
    gpib_stop_cond_t stop_cond;     /* RECEIVE stop condition (16.2.6) */
    uint8_t stop_dab;               /* GPIB_STOP_DAB: the match byte */
} gpib_request_t;

/** Instance status snapshot for diagnostics. */
typedef struct gpib_status_info {
    gpib_lifecycle_t lifecycle;
    bool fault_latched;
    uint64_t instance_epoch;
    gpib_busy_reason_t last_busy_reason;
    size_t open_transactions;
    size_t accepted_blocks;
    size_t unacked_results;
    bool srq_active; /**< observed SRQ line (line ports, idle polling) */
} gpib_status_info_t;

/** Fixed zombie/fault diagnostic (RECOVERY_CONTRACT --4). */
typedef struct gpib_fault_snapshot {
    uint64_t instance_epoch;
    gpib_lifecycle_t lifecycle;
    bool fault_latched;
    gpib_fault_code_t fault_code;
    uint64_t first_fault_us;
    uint64_t last_update_us;
    uint16_t active_action_id;      /**< action generation */
    gpib_action_state_t action_state;
    gpib_action_kind_t action_kind;
    gpib_term_reason_t stop_reason;
    bool action_quiesced;
    bool executor_quiesced;
    size_t accepted_blocks;
    size_t unacked_blocks;
    size_t quarantined_blocks;
    uint64_t srq_events;
    uint64_t dropped_stale_events;
    uint64_t device_events_dropped;
    bool command_valid;   /**< ATN command captured by P1 yield */
    uint8_t command_byte; /**< never part of RX payload */
    bool snapshot_valid;
} gpib_fault_snapshot_t;

/** Binary trace ring statistics. */
typedef struct gpib_trace_info {
    size_t count;     /**< valid records currently in the ring */
    uint64_t dropped; /**< records lost to ring overflow */
} gpib_trace_info_t;

/** Device-function events derived from the interface functions
 * (DCAS/DTAS/RL state changes), delivered one-shot and loss-counted. */
typedef enum gpib_df_event_kind {
    GPIB_DF_CLEAR = 0,   /**< DCAS entered; origin byte in `origin` */
    GPIB_DF_TRIGGER,     /**< DTAS entered */
    GPIB_DF_REMOTE_STATE /**< RL state changed; state in `rl_state` */
} gpib_df_event_kind_t;

typedef struct gpib_df_event {
    gpib_df_event_kind_t kind;
    uint8_t origin;        /**< DCL (0x14) or SDC (0x04) for CLEAR */
    gpib_rl_state_t rl_state; /**< new RL state for REMOTE_STATE */
} gpib_df_event_t;

/* --- lifecycle ----------------------------------------------------------- */

/**
 * Initialize the instance: validates configuration, assigns identity,
 * performs no protocol exchange (LIF-01). Repeated init on a live instance
 * is rejected. @return GPIB_OK, or INVALID_ARGUMENT/UNSUPPORTED_ADDRESSING/
 * RESOURCE_EXHAUSTED on refusal (nothing is captured).
 */
gpib_status_t gpib_init(gpib_context_t *ctx, const gpib_init_params_t *params);

/**
 * Explicit INITIALIZED -> RUNNING transition; no auto-start after init.
 * On a failed bounded port arm the instance stays INITIALIZED (LIF-02).
 */
gpib_status_t gpib_start(gpib_context_t *ctx);

/** Asynchronous shutdown request; idempotent in STOPPING (LIF-06). */
gpib_status_t gpib_request_shutdown(gpib_context_t *ctx);

/** Bounded owner-side advance of internal work (PROGRESS_CONTRACT). */
gpib_status_t gpib_process(gpib_context_t *ctx);

/**
 * Non-blocking teardown; BUSY_RESOURCES until the ledger is drained and the
 * port is provably detached (REQ-076, R16).
 */
gpib_status_t gpib_deinit(gpib_context_t *ctx);

/**
 * Emergency stop: idempotent electrical inhibit of own outputs plus latched
 * fault; never an implicit IFC/DCL; results report uncertainty (REQ-041/042,
 * RECOVERY_CONTRACT --3). Bounded, allocation-free: ISR-safe subset.
 */
gpib_status_t gpib_emergency_stop(gpib_context_t *ctx);

/** Fixed diagnostic snapshot; allowed in any live lifecycle state. */
gpib_status_t gpib_get_fault_snapshot(gpib_context_t *ctx,
    gpib_fault_snapshot_t *out);

/* --- operations ----------------------------------------------------------- */

/**
 * Accept a request: atomically reserves the transaction terminal record plus
 * the slot of every block offered now; rejection transfers no ownership
 * (REQ-032/033, TXN-01). Remote-addressed operations are Controller-only
 * (REQ-017).
 */
gpib_status_t gpib_submit(gpib_context_t *ctx, const gpib_request_t *req,
    gpib_handle_t *txn_out, gpib_handle_t *first_block_out);

/** Append one more stream block to an open transaction (TXN_OPEN row). */
gpib_status_t gpib_submit_block(gpib_context_t *ctx, gpib_handle_t txn,
    const void *data, size_t len, gpib_handle_t *block_out);

/**
 * Serial poll of one device (SPE, MTA(addr), read STB, SPD, UNT). The
 * status byte lands in gpib_result_t.stb of both records on completion;
 * the RQS bit (0x40) reflects the device's service request state.
 * Controller-only, asynchronous like every operation.
 */
gpib_status_t gpib_serial_poll(gpib_context_t *ctx, uint8_t address,
    gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Device role: queue a buffer to be SENT once an external controller
 * addresses this device as talker (MTA(own) and ATN released). EOI is
 * asserted with the last byte. Device-role builds only.
 */
gpib_status_t gpib_device_supply(gpib_context_t *ctx, const void *data,
    size_t len, gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Device role: provide a buffer to be FILLED once an external controller
 * addresses this device as listener (MLA(own) and data flows). Completes
 * on EOI or when the buffer is full. Device-role builds only.
 */
gpib_status_t gpib_device_sink(gpib_context_t *ctx, void *buf, size_t cap,
    gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Controller: pass bus control to the controller at `address`
 * (MTA(address), TCT). On completion the local controller function groups
 * have left CIC (CACS -> CTRS -> CIDS); the receiving core observes the
 * takeover through its own controller FSM. Controller-role builds only.
 */
gpib_status_t gpib_pass_control(gpib_context_t *ctx, uint8_t address,
    gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Controller: the SEND COMMAND control sequence (488.2 --16.2.1) --- send an
 * arbitrary list of 7-bit ATN-true command bytes (DIO8 FALSE). This is the
 * building block for DEVICE CLEAR (DCL/SDC), TRIGGER (GET), SEND LLO and
 * any non-standard sequence. Asynchronous like every operation.
 */
gpib_status_t gpib_send_commands(gpib_context_t *ctx, const uint8_t *cmds,
    size_t n, gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Controller (system controller, C2): the SEND IFC control sequence
 * (488.2 --16.2.8) --- pulse the IFC line for the T8 hold (>100 --s). All
 * talkers/listeners are unaddressed and control returns to this
 * controller. Synchronous within one process round-trip ([P] host model:
 * the pulse completes inside gpib_process while the FSM hold runs).
 */
gpib_status_t gpib_interface_clear(gpib_context_t *ctx);

/**
 * Controller (system controller, C3): ENABLE REMOTE (REN TRUE, 488.2
 * --16.2.11.2) or ENABLE LOCAL CONTROLS-all (REN FALSE, --16.2.10.2).
 * Persistent line state, not an operation; the REN stability window of
 * the REN driver group (T8) is honored by the FSM.
 */
gpib_status_t gpib_remote_enable(gpib_context_t *ctx, bool enable);

/**
 * Controller: the ALLSPOLL protocol (488.2 --17.3, mandatory) --- serial
 * poll every SR1-capable device in `addrs` with ONE armed SPE pass:
 * UNL, MLA(controller), SPE, then per address MTA(addr) + one STB read,
 * finally SPD, UNT. Results land in `results[i]` (application-owned,
 * one byte per address; RQS in bit 6). Asynchronous like every
 * operation; the address list and results are borrowed until the
 * transaction is ACKed.
 */
gpib_status_t gpib_allspoll(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, uint8_t *results, gpib_handle_t *txn_out,
    gpib_handle_t *block_out);

/**
 * Controller: the FINDRQS protocol (488.2 --17.2, optional) --- serial poll
 * the address list in order until a device answers with RQS (bit 6) set;
 * return that address and its STB. Results land in `results[i]` for the
 * addresses actually polled (unpolled entries are untouched); on success
 * `*found` receives the index into `addrs` of the requesting device, and
 * the transaction result bytes_confirmed counts the polls performed. If
 * no device requests, the action still completes; `*found` is then
 * (size_t)-1.
 */
gpib_status_t gpib_findrqs(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, uint8_t *results, size_t *found, gpib_handle_t *txn_out,
    gpib_handle_t *block_out);

/**
 * Controller: the RESET protocol (488.2 --17.1, mandatory), three levels
 * in order: bus initialization (ENABLE REMOTE all + SEND IFC), message
 * exchange initialization (DEVICE CLEAR all = DCL), device initialization
 * (SEND "*RST<PMT>" to every supplied address). The bus levels run
 * synchronously; the *RST sends are queued as one WRITE_MANY-style
 * action per address in order. System controller required.
 */
gpib_status_t gpib_system_reset(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Controller: PERFORM PARALLEL POLL (488.2 §16.15) - assert ATN and EOI
 * (IDY) for the T6 settling, then read the PPR byte from the DIO lines
 * and release. The PPR byte is returned in the transaction result's stb
 * field. Configured devices (PP1 with matching sense) drive their lines.
 */
gpib_status_t gpib_parallel_poll(gpib_context_t *ctx,
    gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Controller: CONFIGURE PARALLEL POLL for one device (488.2 §16.16) -
 * address the device as listener, send PPC then PPE(sense, line).
 * Use send_commands with [MTA(own), UNL, MLA(addr), PPC, PPE] bytes.
 */
gpib_status_t gpib_ppoll_configure(gpib_context_t *ctx, uint8_t addr,
    uint8_t sense, uint8_t dio_line, gpib_handle_t *txn_out,
    gpib_handle_t *block_out);

/**
 * Controller: UNCONFIGURE PARALLEL POLL (482.2 §16.17) - send PPU.
 */
gpib_status_t gpib_ppoll_unconfigure(gpib_context_t *ctx,
    gpib_handle_t *txn_out, gpib_handle_t *block_out);

/**
 * Controller: FINDLSTN (488.2 §17.6, optional) - per candidate address:
 * UNL + MLA(addr) with ATN, then ONE probe DAB byte with ATN released.
 * A listener at that address is in LADS: its acceptor holds NDAC and
 * the byte handshakes (present). Nobody addressed: every acceptor
 * returns to AIDS with passive lines and the source handshake stalls
 * until the probe timeout (absent). Results: `results[i]` = true if a
 * listener was found at addrs[i]. The probe byte is received by a
 * present listener (488.2 warns FINDLSTN may disturb devices).
 *
 * PROFILE NOTE (F19, deliberate): the list operations (allspoll,
 * findrqs, reset, findlstn, pass_control, requestctl_pass) address
 * PRIMARY addresses only; extended (TE/LE) peers are reachable through
 * the per-request address form of WRITE/READ/SPOLL. Extending the
 * lists to full address records is an OPEN profile decision.
 */
gpib_status_t gpib_findlstn(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, bool *results, gpib_handle_t *txn_out,
    gpib_handle_t *block_out);

/**
 * Controller: REQUESTCTL (482.2 §17.5, optional) - assert SRQ (RQS) to
 * request control from the CIC; the CIC (another controller running
 * FINDRQS or REQUESTCTL-CIC-side) identifies the requester via serial
 * poll and passes control via TCT. This side just asserts rsv; the CIC
 * side is `gpib_requestctl_pass`. Controller-role builds only.
 */
gpib_status_t gpib_request_control(gpib_context_t *ctx);

/**
 * Controller (CIC side of REQUESTCTL): serial poll the address list to
 * find the requesting controller (RQS set), then pass control to it
 * via TCT. Returns the found index in `*found` on success, (size_t)-1
 * if nobody requests. BLOCKING composite: pumps the instance until the
 * internal FINDRQS completes; the trailing TCT action is asynchronous
 * and owned by the caller via txn_out/block_out (ack both when done).
 * The list is capped at 16 addresses.
 */
gpib_status_t gpib_requestctl_pass(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, size_t *found, gpib_handle_t *txn_out,
    gpib_handle_t *block_out);

/**
 * Local asynchronous cancel of one request; does not stop the instance
 * (LIF-05). Unknown -> NOT_FOUND, terminal -> ALREADY_TERMINAL.
 */
gpib_status_t gpib_request_cancel(gpib_context_t *ctx, gpib_handle_t txn);

/**
 * Own-address configuration; unsupported forms are rejected without touching
 * the active configuration (ADDR-06).
 */
gpib_status_t gpib_set_address(gpib_context_t *ctx, gpib_address_t address);

/**
 * Address-range contract shared by every submit path: primary 0..30 (31 is
 * the reserved UNL/UNT encoding), secondary 0..31 when present. Pure
 * function -- safe to call before building address lists (ADDR-07).
 */
bool gpib_address_supported(const gpib_address_t *a);

/**
 * Device PP1 individual status (F04): the application-side fact sampled
 * at each parallel poll (IDY). With ist == the configured sense the
 * assigned PPR line is driven TRUE, otherwise passive false (488.1
 * Table 25). Requires pp_capable at init; PP0 -> GPIB_UNSUPPORTED.
 */
gpib_status_t gpib_pp_set_ist(gpib_context_t *ctx, bool ist);

/* --- results -------------------------------------------------------------- */

/**
 * Read a terminal result without taking ownership (peek does not return
 * buffers, REQ-065 / resource_contract --3).
 */
gpib_status_t gpib_result_peek(gpib_context_t *ctx, gpib_handle_t handle,
    gpib_result_t *out);

/**
 * Owner-side ACK: releases exactly one record. A transaction ACK never
 * touches block records or block memory (FIX-01, REQ-070).
 */
gpib_status_t gpib_result_ack(gpib_context_t *ctx, gpib_handle_t handle);

/**
 * Read-only view of a confirmed RX prefix while the block is RESULT_READY
 * and not yet ACKed (REQ-065 [P], FIX-03): peek does not return ownership;
 * after ACK the borrowed view expires while the application's own pointer
 * stays valid. TX blocks do not expose a view.
 */
gpib_status_t gpib_result_view(gpib_context_t *ctx, gpib_handle_t block,
    const uint8_t **data, size_t *len);

/**
 * Explicit recovery attempt. Allowed only in RUNNING with a provably closed
 * old action and no latched fault; STOPPING/DRAINING refuse (RECOVERY_CONTRACT
 * --3). Local policy content remains [O].
 */
gpib_status_t gpib_recover(gpib_context_t *ctx);

/**
 * Pop the oldest observed raw device command byte (DCL/SDC/GET/GTL/LLO
 * candidates), FIFO; interpretation is the application's until the
 * normative baseline (REQ-016 structural, OPEN-01/02).
 * @return GPIB_OK and the byte, or GPIB_NO_RESULT when empty.
 */
gpib_status_t gpib_poll_device_event(gpib_context_t *ctx, uint8_t *cmd_byte);

/**
 * Pop the oldest device-FUNCTION event: DCAS clear (with DCL/SDC origin),
 * DTAS trigger, or an RL remote/local state change. One-shot per accepted
 * byte / transition, overflow is counted, never silently dropped.
 */
gpib_status_t gpib_poll_df_event(gpib_context_t *ctx, gpib_df_event_t *ev);

/**
 * Local request to return to local (rtl, --2.8): one-shot pulse consumed
 * by the next RL step; ignored under local lockout. Device role.
 */
gpib_status_t gpib_device_local(gpib_context_t *ctx);

/**
 * Device role: set the status byte (without B6; the core ORs the RQS bit
 * from the SR function during the serial-poll SPAS send). Combined with
 * gpib_device_service_request it forms the 488.2 status reporting model.
 */
gpib_status_t gpib_device_set_stb(gpib_context_t *ctx, uint8_t stb);

/**
 * Device role: request/release service (rsv local message of the SR
 * function); asserts the SRQ line while the request stands and the poll
 * has not yet read us out.
 */
gpib_status_t gpib_device_service_request(gpib_context_t *ctx, bool rsv);

gpib_status_t gpib_query_status(gpib_context_t *ctx, gpib_status_info_t *out);

/* --- binary trace --------------------------------------------------------- */

gpib_status_t gpib_trace_query(gpib_context_t *ctx, gpib_trace_info_t *out);

/** Copy up to @p cap records oldest-first into @p events; non-consuming. */
gpib_status_t gpib_trace_dump(gpib_context_t *ctx, gpib_trace_event_t *events,
    size_t cap, size_t *n);

#ifdef __cplusplus
}
#endif

#endif /* GPIB_H */
