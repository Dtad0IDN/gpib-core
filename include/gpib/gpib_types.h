/**
 * @file gpib_types.h
 * @brief Public type definitions: statuses, enums, handles, results.
 *
 * All enums and structs are documented; numeric values are NOT a stable
 * ABI ([O] OPEN-26) — only the symbolic names are part of the API.
 */
#ifndef GPIB_TYPES_H
#define GPIB_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Semantic status codes. Numeric values are NOT a stable ABI ([O] OPEN-26):
 * only the symbolic names are part of the source API. */
typedef enum gpib_status {
    GPIB_OK = 0,
    GPIB_INVALID_ARGUMENT,
    GPIB_INVALID_STATE,          /* default-deny: call not permitted in this lifecycle */
    GPIB_INVALID_HANDLE,         /* stale/double/wrong-kind handle or dead instance */
    GPIB_NO_RESULT,              /* ledger empty or record not terminal yet */
    GPIB_BUSY_RESOURCES,         /* deinit refused: un-ACK records or quiescence unproven */
    GPIB_RESOURCE_EXHAUSTED,     /* identity space exhausted; init refuses (REQ-072) */
    GPIB_NOT_FOUND,              /* cancel of an unknown request */
    GPIB_ALREADY_TERMINAL,       /* cancel of an already-terminal request */
    GPIB_ALREADY_STOPPING,       /* idempotent repeated shutdown in STOPPING */
    GPIB_ALREADY_STOPPED,        /* repeated shutdown/emergency in DRAINING */
    GPIB_SHUTDOWN_IN_PROGRESS,   /* per-request cancel while the instance stops */
    GPIB_RECOVERY_NOT_ALLOWED,   /* fail-closed recover (recovery guard unproven) */
    GPIB_UNSUPPORTED_ADDRESSING, /* REQ-079: secondary present, TE/LE not in this build */
    GPIB_NOT_CIC,          /* multi-controller: this controller is not
                              Controller-In-Charge and cannot seize the bus
                              (488.2 --16.2.8: only the system controller
                              may take charge unprompted via IFC) */
    GPIB_PORT_ERROR,             /* level-2 port primitive failed (e.g. arm) */
    GPIB_UNSUPPORTED
} gpib_status_t;

/* Sub-reasons for GPIB_BUSY_RESOURCES diagnostics (REQ-071: BUSY_RESOURCES is
 * a result code, not a lifecycle state). */
typedef enum gpib_busy_reason {
    GPIB_BUSY_NONE = 0,
    GPIB_BUSY_UNACK_RESULTS,
    GPIB_BUSY_STOP_TRANSITION_PENDING,
    GPIB_BUSY_PORT_TEARDOWN_PENDING
} gpib_busy_reason_t;

/* Terminal reason of a block/transaction result, kept separate from status. */
typedef enum gpib_term_reason {
    GPIB_TERM_NONE = 0,
    GPIB_TERM_COMPLETED,
    GPIB_TERM_NOT_STARTED, /* never dispatched to the executor (FIX-02/REQ-064) */
    GPIB_TERM_CANCELED,
    GPIB_TERM_SHUTDOWN,
    GPIB_TERM_FAULT,
    GPIB_TERM_PHASE_INTERRUPTED, /* P1 YIELD_TO_OWNER (R12), working name [O] */
    GPIB_TERM_TIMEOUT,
    GPIB_TERM_IFC
} gpib_term_reason_t;

/* Action state axis of the mailbox host model (MAILBOX_IRQ_CONTRACT --2).
 * Separate from block states and from instance lifecycle. */
typedef enum gpib_action_state {
    GPIB_ACTION_EMPTY = 0,
    GPIB_ACTION_PUBLISHED,
    GPIB_ACTION_ACTIVE,
    GPIB_ACTION_STOP_REQUESTED,
    GPIB_ACTION_QUIESCED,
    GPIB_ACTION_RESULT_ACCOUNTED
} gpib_action_state_t;

typedef enum gpib_action_kind {
    GPIB_ACTION_NONE = 0,
    GPIB_ACTION_TX_BLOCK,
    GPIB_ACTION_RX_BLOCK
} gpib_action_kind_t;

typedef enum gpib_fault_code {
    GPIB_FAULT_NONE = 0,
    GPIB_FAULT_EMERGENCY_STOP,
    GPIB_FAULT_HISTORY_LOST, /* mandatory edge history unrecoverable (EX-04) */
    GPIB_FAULT_DF_EVENT_LOST, /* DCAS/DTAS/RL pulse lost: mandatory
                                  delivery violated (RL/DC/DT extract §4.5) */
    GPIB_FAULT_INTERNAL
} gpib_fault_code_t;

/* Binary trace event kinds (errors_and_diagnostics.md --5). Fixed-size
 * records, ring semantics; numeric kind values are internal, not ABI. */
typedef enum gpib_trace_kind {
    GPIB_TRC_INIT = 1,
    GPIB_TRC_START,
    GPIB_TRC_SUBMIT,
    GPIB_TRC_SUBMIT_BLOCK,
    GPIB_TRC_PUBLISH,
    GPIB_TRC_ACCEPT,
    GPIB_TRC_COMPLETE,
    GPIB_TRC_ACCOUNT,
    GPIB_TRC_ACK_BLOCK,
    GPIB_TRC_ACK_TXN,
    GPIB_TRC_CANCEL,
    GPIB_TRC_SHUTDOWN,
    GPIB_TRC_EMERGENCY,
    GPIB_TRC_RECOVER,
    GPIB_TRC_FAULT,
    GPIB_TRC_QUARANTINE,
    GPIB_TRC_STALE_DROP,
    GPIB_TRC_SRQ,
    GPIB_TRC_DEADLINE_EXPIRE,
    GPIB_TRC_DRAINING,
    GPIB_TRC_DEVICE_CMD,
    GPIB_TRC_DEINIT
} gpib_trace_kind_t;

/* One fixed-size binary trace record (REQ-055/058). */
typedef struct gpib_trace_event {
    uint64_t ts_us;
    uint16_t kind; /* gpib_trace_kind_t */
    uint16_t a;    /* slot / action id, context dependent */
    uint16_t b;    /* generation / secondary slot */
    uint32_t c;    /* byte count / cause / counter, context dependent */
} gpib_trace_event_t;

/* Instance lifecycle (REQ-071). STOPPED is only the historical outcome of a
 * successful deinit and deliberately NOT a state of a live object. */
typedef enum gpib_lifecycle {
    GPIB_LC_UNINITIALIZED = 0,
    GPIB_LC_INITIALIZED,
    GPIB_LC_RUNNING,
    GPIB_LC_STOPPING,
    GPIB_LC_DRAINING
} gpib_lifecycle_t;

typedef enum gpib_request_kind {
    GPIB_REQ_WRITE = 0,
    GPIB_REQ_READ,
    GPIB_REQ_WRITE_MANY, /* high-level multicast write (REQ-017, R5) */
    GPIB_REQ_SPOLL,      /* serial poll: SPE, MTA(addr), read STB, SPD, UNT */
    GPIB_REQ_DEV_TX,     /* device role: supply data once talk-addressed */
    GPIB_REQ_DEV_RX,     /* device role: fill the sink once listen-addressed */
    GPIB_REQ_TCT,        /* controller: pass control (MTA(addr), TCT) */
    GPIB_REQ_CMDS,       /* controller: SEND COMMAND (arbitrary ATN bytes) */
    GPIB_REQ_ALLSPOLL,   /* controller: serial poll a list of addresses */
    GPIB_REQ_FINDRQS,    /* controller: poll until RQS, return the device */
    GPIB_REQ_RESET,      /* controller: RESET protocol (IFC+DCL+*RST list) */
    GPIB_REQ_PPOLL,      /* controller: PERFORM PARALLEL POLL (16.15) */
    GPIB_REQ_FINDLSTN    /* controller: FINDLSTN (17.6, NDAC probing) */
} gpib_request_kind_t;

/* Semantic address (REQ-078/079 [D], ADDRESS_API_CONTRACT v0.2).
 * A secondary value of 0 is a VALID secondary; absence is expressed ONLY by
 * secondary_present == false. C layout is a project draft ([P]); final ABI
 * stays open (OPEN-05/26). */
typedef struct gpib_address {
    uint8_t primary;
    uint8_t secondary;
    bool secondary_present;
} gpib_address_t;

/* Read/stream completion cause, reported separately from the byte count
 * (REQ-035). EOS policy itself stays [O] (OPEN-10). */
typedef enum gpib_end_cause {
    GPIB_END_NONE = 0,
    GPIB_END_LENGTH, /* fixed byte count reached */
    GPIB_END_EOI,    /* EOI observed with the last byte */
    GPIB_END_EOS,    /* reserved: EOS terminator, policy [O] */
    GPIB_END_CHAR    /* RECEIVE stop-character matched (488.2 16.2.6.3-4) */
} gpib_end_cause_t;

/* RECEIVE RESPONSE MESSAGE stop conditions (488.2 16.2.6): END required,
 * NL/comma/semicolon required, byte count optional, arbitrary DAB
 * optional. The rx_min_bytes=0 READ currently means "EOI only". */
typedef enum gpib_stop_cond {
    GPIB_STOP_EOI = 0,     /* stop on END (default) */
    GPIB_STOP_NL,          /* stop on NL (0x0A) */
    GPIB_STOP_COMMA,       /* stop on comma (0x2C) */
    GPIB_STOP_SEMICOLON,   /* stop on semicolon (0x3B) */
    GPIB_STOP_DAB          /* stop on arbitrary DAB match (16.2.6.6) */
} gpib_stop_cond_t;

typedef struct gpib_result {
    gpib_status_t status;
    gpib_term_reason_t reason;
    gpib_end_cause_t end_cause;
    size_t bytes_confirmed;   /* handshake-confirmed prefix, never observed-only */
    uint8_t stb;              /* status byte of a completed serial poll */
    bool last_byte_uncertain; /* REQ-040: fate of the last TX/RX byte unproven */
} gpib_result_t;

typedef enum gpib_handle_kind {
    GPIB_HANDLE_TXN = 1,
    GPIB_HANDLE_BLOCK = 2
} gpib_handle_kind_t;

/* Handle layout is a project draft ([P], OPEN-26). Invariant (FIX-01,
 * REQ-070/077): an un-ACKed block handle stays valid across transaction ACK
 * and transaction slot reuse; only the block's own ACK invalidates it. */
typedef struct gpib_handle {
    uint64_t epoch;
    uint16_t slot;
    uint32_t generation; /* H6 (final audit): 32-bit kills the 65 536-op
                            ABA -- stale handles stay stale */
    uint8_t kind; /* gpib_handle_kind_t */
    uint8_t reserved;
} gpib_handle_t;

#ifdef __cplusplus
}
#endif

#endif /* GPIB_TYPES_H */
