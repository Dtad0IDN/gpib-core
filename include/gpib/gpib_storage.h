#ifndef GPIB_STORAGE_H
#define GPIB_STORAGE_H

/* Compile-time typed storage header ([P], configuration.md --7). It reveals
 * the context representation to the placement site, while the regular API in
 * gpib.h stays opaque. Not part of the stable source API (OPEN-05/26).
 *
 * Rationale: a plain _Alignas'd byte array cast to a struct pointer does not
 * create an object of the effective struct type under C11, so the placement
 * site uses this typed storage object instead.
 *
 * Include this header exactly at the place that owns the instance memory. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib_cfg.h"
#include "gpib/gpib_port.h"
#include "gpib/gpib_proto.h"
#include "gpib/gpib_types.h"

/* Layout fingerprint: detects storage compiled with a different profile than
 * the core library (configuration.md --7 [D]). Multiplicative hash over all
 * layout-affecting knobs. */
#define GPIB_STORAGE_FINGERPRINT \
    (0x47504942u ^ ((uint32_t)(GPIB_CFG_ROLE) * 0x9E3779B9u) ^ \
     ((uint32_t)(GPIB_CFG_MAX_OPERATIONS) * 0x85EBCA6Bu) ^ \
     ((uint32_t)(GPIB_CFG_MAX_BLOCKS) * 0xC2B2AE35u) ^ \
     ((uint32_t)(GPIB_CFG_TRACE_EVENTS) * 0x27D4EB2Fu) ^ \
     ((uint32_t)(GPIB_CFG_DEVICE_EVENTS) * 0x165667B1u) ^ \
     ((uint32_t)(GPIB_CFG_MAX_CMDS) * 0x9E3779B1u) ^ \
     ((uint32_t)(GPIB_CFG_MAX_LISTENERS) * 0x85EBCA6Du) ^ \
     ((uint32_t)(GPIB_CFG_DF_EVENTS) * 0x27D4EB31u) ^ \
     ((uint32_t)(GPIB_CFG_PROCESS_BUDGET) * 0x165667B5u) ^ \
     ((uint32_t)sizeof(struct gpib_context)))

typedef enum gpib_txn_state {
    TXN_UNRESERVED = 0,
    TXN_RESERVED,
    TXN_OPEN,
    TXN_FINISHING,
    TXN_RESULT_READY,
    TXN_RECORD_RELEASED /* slot reusable; generation already bumped */
} gpib_txn_state_t;

typedef enum gpib_block_state {
    BLOCK_APP_OWNED = 0, /* free slot */
    BLOCK_CORE_QUEUED,
    BLOCK_EXECUTOR_IN_USE,
    BLOCK_WAIT_QUIESCENCE,
    BLOCK_ACCOUNTABLE,
    BLOCK_RESULT_READY
} gpib_block_state_t;

typedef struct gpib_txn_record {
    gpib_txn_state_t state;
    uint32_t generation; /* H6: 32-bit (bumped on ACK; 0 = never used) */
    gpib_address_t address;
    const gpib_address_t *listeners_first; /* WRITE_MANY list (borrowed) */
    size_t listener_count;
    gpib_request_kind_t kind;
    size_t block_count;
    size_t blocks_terminal;
    /* R17-01: per-transaction FIFO sequence for block dispatch. The
     * physical block-slot index must NEVER define transfer order: an
     * ACK frees a low slot mid-stream and a later submit_block reuses
     * it -- dispatching by slot reordered A B C D into A B D C. */
    uint32_t next_block_seq;
    size_t rx_limit;       /* READ stream target (rx_min_bytes), 0 = EOI only */
    size_t stream_bytes;   /* cross-block confirmed byte accumulation */
    uint64_t deadline_us;  /* absolute logical deadline, 0 = none ([P]) */
    uint8_t spoll_stb;     /* internal STB staging of a serial poll */
    uint8_t stop_cond;     /* gpib_stop_cond_t of the READ (16.2.6) */
    uint8_t stop_dab;      /* GPIB_STOP_DAB: the match byte */
    /* SEND COMMAND: byte count and the fixed copy (the caller's buffer
     * may be stack-scoped: cmd_stored[] keeps the data alive until ACK;
     * the sequencer reads ONLY cmd_stored). */
    size_t cmd_len;
    uint8_t cmd_stored[GPIB_CFG_MAX_CMDS];
    /* ALLSPOLL/FINDRQS: the address list (borrowed) and the STB results
     * (owned by the application, written by the core when each poll
     * completes). */
    const uint8_t *poll_addrs;
    size_t poll_count;
    size_t poll_index;
    uint8_t *poll_results;
    /* FINDRQS: index of the requesting device, (size_t)-1 until found. */
    size_t find_index;
    size_t *find_out; /* borrowed caller output, written at completion */
    /* FINDLSTN: candidate addresses and results (borrowed). */
    const uint8_t *probe_addrs;
    bool *probe_results;
    size_t probe_count;
    size_t probe_index;
    uint8_t probe_state; /* 0=send MLA, 1=observe NDAC */
    uint8_t ppr_byte;    /* PPOLL: the read PPR result */
    bool retired;        /* R17-02: generation exhausted; slot retired */
    /* RESET: the staged *RST message for the current target. */
    uint8_t rst_msg[6];
    gpib_result_t result;  /* valid in TXN_RESULT_READY */
} gpib_txn_record_t;

typedef struct gpib_block_record {
    gpib_block_state_t state;
    uint32_t generation; /* H6 */
    /* Correlation only, deliberately no live reference into the transaction
     * ledger: transaction slot reuse cannot corrupt this record (FIX-01,
     * REQ-077). */
    uint16_t txn_slot;
    uint32_t txn_generation; /* H6 */
    uint32_t txn_seq; /* R17-01: submit-order sequence within the txn:
                       * dispatch picks the LOWEST seq among queued
                       * blocks, independent of the physical slot. */
    /* Borrowed zero-copy payload, valid until the block's own ACK. */
    const uint8_t *data;
    size_t len;
    bool is_rx;         /* RX blocks expose a read-only view before ACK */
    bool quarantined;   /* undemostrable quiescence: pinned forever (LED-04) */
    bool retired;       /* R17-02: generation exhausted; never reused */
    uint8_t internal_payload; /* AUD-16: block-OWNED one-byte storage for
                               * SPOLL STB / PPOLL PPR / FINDRQS STB --
                               * the view must stay valid until THIS
                               * block's ACK, surviving txn slot reuse. */
    gpib_result_t result; /* valid in BLOCK_RESULT_READY */
} gpib_block_record_t;

/* Mailbox host model (MAILBOX_IRQ_CONTRACT --2): exactly one action slot per
 * instance, at most one ACTIVE. Descriptor fields are immutable after
 * publish. */
struct gpib_action {
    uint8_t state; /* gpib_action_state_t */
    uint8_t kind;  /* gpib_action_kind_t */
    uint16_t generation; /* bumped per publish, starts at 1. DELIBERATELY
                          * 16-bit (N12-01 audit): it pairs with the
                          * 16-bit port event tag (F20 port contract);
                          * the HANDLE/ledger identity path is the full
                          * 32-bit generation. */
    uint16_t block_slot;
    uint32_t block_generation; /* H6 */
    uint16_t txn_slot;
    const uint8_t *data;
    size_t len;      /* buffer capacity */
    size_t limit;    /* completion target (per-block) */
    size_t committed;
    uint64_t deadline_us; /* copied from the transaction descriptor */
    uint8_t stop_cause; /* gpib_term_reason_t latch on the action */
    bool eoi_terminated; /* RX stream ended by EOI, not by length */
    bool char_terminated; /* RX ended by the 16.2.6 stop character */
    bool last_byte_uncertain; /* REQ-040: in-flight byte at interruption */
    bool command_valid;
    uint8_t command_byte; /* ATN command byte, never part of RX payload */
};

struct gpib_context {
    uint32_t storage_fingerprint; /* stamped by GPIB_CONTEXT_STORAGE_DEFINE */
    gpib_lifecycle_t lifecycle;
    bool fault_latched;   /* independent latched flag, not a lifecycle value */
    gpib_fault_code_t fault_code;
    uint64_t first_fault_us;
    uint64_t last_update_us;
    gpib_busy_reason_t last_busy_reason;
    uint64_t epoch;       /* identity from the external epoch manager (R14) */
    bool system_controller;
    gpib_address_t own_address;
    const struct gpib_port *port;
    /* Instance stop latch: monotonic level, cause, optional target txn.
     * txn_slot == 0xFFFF means "instance-global" (shutdown/IFC/emergency). */
    struct {
        bool active;
        uint8_t cause; /* gpib_term_reason_t */
        uint16_t txn_slot;
    } stop_request;
    struct gpib_action action;
    uint64_t srq_events;
    uint64_t dropped_stale_events;
    size_t quarantined_blocks;
    /* Line-level protocol engine state (active when the port provides the
     * line slice). Sequencer: IDLE -> CMD (ATN phase, address/command
     * bytes via SH as CACS) -> DATA_TX (SH as TACS) / DATA_RX (AH) ->
     * LISTEN_CMDS (external ATN stole the bus: capture command bytes). */
    struct {
        uint8_t seq;      /* gpib_lseq_t */
        gpib_sh_t sh;
        gpib_ah_t ah;
        gpib_addr_engine_t addr;
        gpib_talker_t tk;   /* device role: talker FSM (own address) */
        gpib_listener_t ln; /* device role: listener FSM (own address) */
        gpib_controller_t ctrl; /* controller function groups */
        bool ctrl_tct_pulse;   /* TCT accepted this step (receiver side) */
        bool send_tct;         /* controller is sending TCT (sender side) */
        /* Prologue capacity: MTA + UNL + MAX_LISTENERS x (MLA + optional SAD)
         * vs MAX_CMDS, whichever is larger. */
        uint8_t cmds[GPIB_CFG_MAX_CMDS < (2 + 2 * GPIB_CFG_MAX_LISTENERS)
                         ? (2 + 2 * GPIB_CFG_MAX_LISTENERS)
                         : GPIB_CFG_MAX_CMDS];
        size_t n_cmds;
        size_t cmd_pos;
        uint8_t post[4];  /* command epilogue (serial poll: SPD, UNT) */
        size_t n_post;
        size_t post_pos;
        size_t tx_pos;    /* next data byte index within the action */
        size_t rx_captured;
        bool rx_eoi;
        uint8_t rx_holdoff; /* mandatory !rdy rounds after a capture */
        bool stop_char_hit; /* the 16.2.6 stop-character was received */
        bool we_drive_atn;
        bool srq_seen;    /* last observed SRQ line level */
        uint8_t last_cmd;
        bool last_cmd_valid;
        /* System controller line state (C2/C3): merged INTO every line
         * drive of the sequencer --- never driven as a standalone request
         * (that would clobber DAV/ATN of an active transfer). */
    bool sc_ifc;
    bool sc_ifc_last; /* previous round's sc_ifc: the OWN pulse reads
                       * back TRUE one exchange after its release
                       * (wired-OR) -- the external-IFC detector must
                       * not fire on our own trailing edge (F01). */
    bool rx_stop_hold; /* F13: a completed RECEIVE holds NRFD/NDAC TRUE
                        * (488.2 16.2.6 stop state) until the next
                        * control sequence; materialized by poll_idle. */
    bool rx_session_open; /* F05: an OPEN READ stream between blocks --
                           * block boundary is NOT stream end: the
                           * addressing stays latched, the acceptor holds
                           * NRFD (backpressure), and the next block
                           * resumes DATA_RX with NO new prologue. */
    uint16_t rx_session_slot; /* C5: the session belongs to EXACTLY this
                               * transaction (slot + generation): another
                               * READ (or a cancelled one) must never
                               * resume a stranger's stream. */
    uint32_t rx_session_gen; /* N12-01: MUST match the 32-bit txn
                              * generation width -- a 16-bit copy used
                              * to truncate past 65 535 and the
                              * scheduler disowned its own stream
                              * (permanent stall, rev12 probe). */
    bool pp_drove_dio; /* device PP1 (F04): WE own the DIO drivers this
                        * round for the PPR answer -- release when the
                        * PPAS window closes. */
    uint64_t probe_settle_deadline; /* FINDLSTN 17.6.5: end of the
                                     * >= 1.5 ms ATN-false window */
    bool sc_ren;
    bool cic_once; /* has EVER held CIC (power-on IFC or TCT) */
    uint32_t idy_rounds; /* AUD-14 runaway cap (time is the real bound) */
    uint64_t idy_start_us; /* AUD-14: physical IDY start (T6 by time) */
    } line;
/* Device service-event ring: raw command bytes, oldest-dropped policy
  * with a loss counter; interpretation is the application's until the
     * normative baseline arrives (REQ-016, OPEN-01/02). */
#ifndef GPIB_CFG_DF_EVENTS
#define GPIB_CFG_DF_EVENTS 8
#endif
    uint8_t dev_evt[GPIB_CFG_DEVICE_EVENTS];
    size_t dev_evt_head;
    size_t dev_evt_count;
    uint64_t dev_evt_dropped;
    /* Device-function events (DCAS/DTAS/RL) and the rtl pulse. */
    struct {
        uint8_t kind;       /* gpib_df_event_kind_t */
        uint8_t origin;
        uint8_t rl_state;   /* gpib_rl_state_t */
    } df_evt[GPIB_CFG_DF_EVENTS];
    size_t df_evt_head;
    size_t df_evt_count;
    uint64_t df_evt_dropped;
    bool rtl_pulse;
    gpib_rl_t rl;
    gpib_dc_t dc;
    gpib_dt_t dt;
    /* Device SR/STB: serial-poll answer bytes and the service request. */
    gpib_sr_t sr;
    uint8_t device_stb; /* STB without B6; core ORs RQS during SPAS */
    bool device_rsv;
    /* Device PP1 (488.1 2.9): the parallel poll responder of THIS core
     * (F04) -- configured remotely via PPC/PPE, answering on IDY. */
    gpib_pp_t pp;
    bool pp_ist; /* individual status: application-owned input */
    /* R6-04: the requestctl_pass composite stages FINDRQS poll results
     * HERE, not on the caller's stack -- a wedged port may keep the
     * action running past the composite's return, and instance memory
     * (unlike a dead frame) stays valid for that entire lifetime. */
    uint8_t requestctl_results[16];
    bool spas_tx_active; /* one STB send in flight (dev_wait SPAS path) */
    /* System controller line requests (C2/C3): IFC pulse timing uses the
     * T8 hold of the ctrl FSM IFENG group; REN persists until changed. */
    bool ifc_request;
    bool ren_request;
    /* Scratch drive request of the line sequencer (avoids shadowing). */
    gpib_line_drive_t req;
    gpib_txn_record_t txn[GPIB_CFG_MAX_OPERATIONS];
    gpib_block_record_t block[GPIB_CFG_MAX_BLOCKS];
#if GPIB_CFG_TRACE_EVENTS > 0
    /* Binary trace ring (errors_and_diagnostics --5): oldest records are
     * dropped on overflow and counted, never coalesced or reordered. */
    gpib_trace_event_t trace[GPIB_CFG_TRACE_EVENTS];
    size_t trace_head;   /* next write position */
    size_t trace_count;  /* valid records in the ring */
    uint64_t trace_dropped;
#endif
};

/* Define static storage for one core instance at the placement site. */
#define GPIB_CONTEXT_STORAGE_DEFINE(name) \
    static struct gpib_context name = { .storage_fingerprint = GPIB_STORAGE_FINGERPRINT }

#endif /* GPIB_STORAGE_H */
