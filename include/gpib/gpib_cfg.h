/**
 * @file gpib_cfg.h
 * @brief Compile-time configuration knobs for the core profile.
 *
 * Controls role (Controller/Device), ledger sizes, process budget,
 * trace ring capacity, and simulated line timings. All knobs have
 * #ifndef guards; limits never change the mandatory function set.
 */
#ifndef GPIB_CFG_H
#define GPIB_CFG_H

/* Compile-time profile knobs (DEC-33, configuration.md --1). Macro names are
 * a project draft ([P]); limits never change the mandatory function set of
 * the role (REQ-006/R4). */

#define GPIB_ROLE_CONTROLLER 0
#define GPIB_ROLE_DEVICE 1

#ifndef GPIB_CFG_ROLE
#define GPIB_CFG_ROLE GPIB_ROLE_CONTROLLER
#endif

/* Transaction terminal records and block ledger slots. Exhaustion rejects
 * new work non-blockingly without taking ownership (REQ-034). */
#ifndef GPIB_CFG_MAX_OPERATIONS
#define GPIB_CFG_MAX_OPERATIONS 4
#endif

#ifndef GPIB_CFG_MAX_BLOCKS
#define GPIB_CFG_MAX_BLOCKS 8
#endif

/* Bounded work per gpib_process call: executor steps (B_work of
 * PROGRESS_CONTRACT --3; numeric policy [O]). */
#ifndef GPIB_CFG_PROCESS_BUDGET
#define GPIB_CFG_PROCESS_BUDGET 8
#endif

/* Maximum listeners per WRITE_MANY prologue and maximum ATN command bytes
 * per SEND COMMAND: operations exceeding these are REJECTED before any
 * side effect (never silently truncated). The sequencer prologue buffer
 * is sized accordingly (12 >= 1 UNL + 10 MLA + 1 MTA). */
#ifndef GPIB_CFG_MAX_LISTENERS
#define GPIB_CFG_MAX_LISTENERS 10
#endif
#ifndef GPIB_CFG_MAX_CMDS
#define GPIB_CFG_MAX_CMDS 12
#endif

/* Binary trace ring capacity (REQ-055/058); 0 disables tracing at compile
 * time. Overflow drops the OLDEST records and counts them. */
#ifndef GPIB_CFG_TRACE_EVENTS
#define GPIB_CFG_TRACE_EVENTS 32
#endif

/* Device service-event ring: raw observed command bytes (DCL/SDC/GET/GTL/
 * LLO candidates) surfaced to the application, unclassified until the
 * normative baseline (OPEN-01/02). */
#ifndef GPIB_CFG_DEVICE_EVENTS
#define GPIB_CFG_DEVICE_EVENTS 4
#endif

/* Host-model line timings: SIMULATED microseconds per bus exchange round
 * (Table 39 values are not expressible in the host model; a real port
 * overrides these with its transceiver's measured numbers). */
#ifndef GPIB_CFG_T1_SOURCE_US
#define GPIB_CFG_T1_SOURCE_US 2u
#endif
/* FINDLSTN DAB probe: SDYS stall window before an address is declared
 * absent (Annex B empty-bus check has no counter of its own). */
#ifndef GPIB_CFG_PROBE_TIMEOUT_US
#define GPIB_CFG_PROBE_TIMEOUT_US 50u
#endif
/* PARALLEL POLL IDY settle (T6): a TIME bound per Table 39 (>= 2 us),
 * measured from the physical IDY start on the port clock (AUD-14). A
 * second core instance sharing the bus gets a real processing slice
 * inside the window; a frozen clock stalls the poll instead of
 * fabricating a result. */
#ifndef GPIB_CFG_T6_SETTLE_US
#define GPIB_CFG_T6_SETTLE_US 12u
#endif
/* IFC pulse length and REN low-time (T8 family): the standard demands
 * STRICTLY MORE than 100 us (488.1 3.3 / 488.2 16.2.11, NAT4882 sre),
 * so the simulated hold is 101 us -- 100 exactly would be a boundary
 * violation. Real ports re-anchor both to their transceiver edges.
 * PORT CONTRACT NOTE (Rev7 §3): the core counts the REN-low window from
 * the instance's time zero -- valid because a freshly attached port has
 * held REN released (bus pull-ups) since wire power-up. A port whose
 * PHYSICAL REN driver outlives one core instance (deinit/init cycle on
 * a live bus, uptime-based epoch) must itself anchor the low window to
 * its own last-release edge and is allowed to delay the first sre. */
#ifndef GPIB_CFG_T8_STRICT_US
#define GPIB_CFG_T8_STRICT_US 101u
#endif

/* FINDLSTN settle window (488.2 17.6.5): with ATN FALSE the controller
 * waits at least 1.5 ms before sampling NDAC. Simulated microseconds
 * (one bus round = one us in the host model); real ports re-anchor to
 * their own clock. Compile-time conformant floor (rev14 T14-02): a
 * build with a shorter window is NOT a 488.2 FINDLSTN -- refuse it
 * unless this guard is deliberately edited for a mutation lab. */
#ifndef GPIB_CFG_FINDLSTN_SETTLE_US
#define GPIB_CFG_FINDLSTN_SETTLE_US 1500u
#endif
#if GPIB_CFG_FINDLSTN_SETTLE_US < 1500u
#error "GPIB_CFG_FINDLSTN_SETTLE_US < 1500 us violates 488.2 17.6.5"
#endif

#endif /* GPIB_CFG_H */
