#ifndef GPIB_PORT_H
#define GPIB_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Level-2 port contract (port_contract.md). The first slice (arm,
 * safe_detached, executor_quiesced) is the semantic evidence interface.
 * The remaining entries are a HOST-MODEL slice for the S1 mailbox
 * prototype: `transfer` stands in for the future per-phase GPIO
 * handshake primitives and event capture (OPEN-03/04); it makes no
 * IEEE-488 timing claims.
 *
 * ---------------------------------------------------------------------------
 * PORT GUARANTEES REQUIRED FROM A REAL (LEVEL-2) ADAPTER
 * ---------------------------------------------------------------------------
 * The core is a single-owner, serialized state machine (REQ-021). The
 * port adapter MUST provide the following guarantees; the host model
 * satisfies them trivially by construction (single-threaded test loop):
 *
 * G1 Serialization: all calls into the core (submit/process/ack/...)
 *    and all core-visible observations (read_lines/poll_event) happen
 *    under ONE total order. A real adapter serializes IRQ bottom halves
 *    against the protocol owner through the mailbox/IRQ contract
 *    (MAILBOX_IRQ_CONTRACT, OPEN-03/25); the release/acquire publication
 *    required is proven by tests/test_litmus.c.
 *
 * G2 Snapshot consistency: read_lines returns all lines sampled at one
 *    instant; a real adapter uses an input latch or snapshot register,
 *    never a torn multi-GPIO read.
 *
 * G3 Edge history: between two read_lines calls the port must not lose
 *    mandatory edges (ATN/IFC/SRQ assert+release while masked). Losing
 *    them is a fault (HISTORY_LOST -> quarantine), never recoverable.
 *
 * G4 ISR fast path: an IRQ handler may capture edge facts and post them
 *    into the core's event queue, but MUST NOT step protocol FSMs,
 *    touch the ledger, or drive outputs - owner-only work. The ISR-safe
 *    API subset is emergency stop + evidence capture.
 *
 * G5 Quiescence evidence: safe_detached/executor_quiesced must be PROVEN
 *    properties (async fence complete), not timeouts; the core refuses
 *    deinit otherwise (LIF-07, MBX-11).
 *
 * A port that cannot meet G1-G5 is not a valid adapter; the core fails
 * closed (fault + STOPPING) on violations it can observe. */

typedef enum gpib_port_event_type {
    GPIB_PEVT_NONE = 0,
    GPIB_PEVT_ATN_COMMAND, /* command byte in `data`; triggers P1 yield */
    GPIB_PEVT_IFC,         /* external IFC observed */
    GPIB_PEVT_SRQ,         /* sideband request; must NOT terminate a block */
    GPIB_PEVT_DEADLINE,    /* action deadline expired (tag = action gen) */
    GPIB_PEVT_EOI,         /* end-of-identify observed on RX */
    GPIB_PEVT_HISTORY_LOST,/* two mandatory edges lost while masked (EX-04) */
    /* Raw observed command byte addressed to this device (DCL/SDC/GET/GTL/
     * LLO candidates). Passed through UNCLASSIFIED: the byte-to-meaning
     * mapping awaits the normative baseline (OPEN-01/02). */
    GPIB_PEVT_DEVICE_CMD
} gpib_port_event_type_t;

typedef struct gpib_port_event {
    uint8_t type;  /* gpib_port_event_type_t */
    uint16_t tag;  /* action generation when relevant; 0 = untagged.
                    * F20: matches the FULL uint16_t action generation
                    * (generations are monotonic per action slot and
                    * wrap at 65536 -- ports must deliver ISR facts
                    * within the lifetime of the action that armed
                    * them, so a wrapped collision cannot occur). */
    uint8_t data;  /* command byte for ATN_COMMAND */
    uint8_t line;  /* reserved for GPIO line facts */
} gpib_port_event_t;

typedef enum gpib_dir { GPIB_DIR_TX = 0, GPIB_DIR_RX = 1 } gpib_dir_t;

/* Line-level snapshot: IEEE LOGICAL message truth (TRUE = wire LOW). */
typedef struct gpib_line_snap {
    bool dav;   /* DAV asserted */
    bool nrfd;  /* NRFD asserted (NOT ready for data) */
    bool ndac;  /* NDAC asserted (NOT accepted) */
    bool atn;
    bool eoi;
    bool srq;
    bool ifc;
    bool ren;
    uint8_t dio;
    uint64_t now_us;
} gpib_line_snap_t;

/* Requested line states from the core; the adapter applies them to the
 * wires. dio != NULL drives a data byte, NULL releases the DIO drivers
 * (between bytes the engine keeps the last driven value by not calling
 * drive_data). ifc_request is only honored for system controllers (C2) and
 * is pulsed by the core for the T8 hold. */
typedef struct gpib_line_drive {
    bool dav;
    bool nrfd; /* asserted = NOT ready */
    bool ndac; /* asserted = NOT accepted */
    bool atn;
    bool eoi;
    bool srq;  /* device-side SR function pull (wired-OR line) */
    bool ren;  /* system controller REN (SRAS) */
    bool ifc;  /* system controller IFC (SIAS) */
} gpib_line_drive_t;

typedef struct gpib_port {
    const char *name;
    /* Bounded port arm step for start(); on failure the instance stays
     * INITIALIZED and the port must have performed no persistent changes
     * (LIF-02). */
    bool (*arm)(void *user);
    /* Provably safe port detach; required before deinit succeeds (LIF-04). */
    bool (*safe_detached)(void *user);
    /* EXECUTOR_QUIESCED evidence for the whole instance (MAILBOX contract
     * --5.2). Stronger than any ACTION_QUIESCED of a single action. */
    bool (*executor_quiesced)(void *user);
    /* Bounded electrical release of own outputs; NOT a protocol stop and
     * never an automatic IFC/DCL (RECOVERY_CONTRACT --3). */
    void (*emergency_inhibit)(void *user);
    /* Monotonic logical microseconds (REQ-024); host model only. */
    uint64_t (*now_us)(void *user);
    /* Move up to `cap` bytes of one bounded step; returns bytes moved. */
    size_t (*transfer)(void *user, gpib_dir_t dir, uint8_t *buf, size_t cap);
    /* Pop one queued port event; returns false when the queue is empty. */
    bool (*poll_event)(void *user, gpib_port_event_t *out);
    /* --- line-level slice (host model): when read_lines is non-NULL the
     * executor runs the SH/AH protocol engine instead of the byte pump.
     * Each read_lines call advances the simulated bus by one exchange. */
    bool (*read_lines)(void *user, gpib_line_snap_t *out);
    void (*drive)(void *user, const gpib_line_drive_t *req);
    void (*drive_data)(void *user, const uint8_t *dio); /* NULL = release */
    void *user;
} gpib_port_t;

/* Host null port: no hardware, no events, no bytes; always provably detached
 * and quiesced. Used by lifecycle/ledger tests. */
extern const gpib_port_t gpib_null_port;

#ifdef __cplusplus
}
#endif

#endif /* GPIB_PORT_H */
