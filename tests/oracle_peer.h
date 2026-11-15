#ifndef ORACLE_PEER_H
#define ORACLE_PEER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib.h"
#include "gpib/gpib_port.h"
#include "vbus.h"

/* T04: the INDEPENDENT wire oracle -- a test partner that shares ZERO
 * protocol code with the core under test. It never calls gpib_sh_step,
 * gpib_ah_step, gpib_talker_step, gpib_listener_step, gpib_pp_step or
 * gpib_decode_atn_byte: the acceptor/talker/parallel-poll behaviors are
 * hand-written from the WIRE rules of IEEE 488.1 (handshake truth,
 * command byte ranges), deliberately structured differently from the
 * core FSMs. Only the vbus wired-OR simulator is shared -- it is wire
 * infrastructure, not a protocol oracle.
 *
 * The peer also records a per-round WAVEFORM trace and checks source
 * invariants online (DIO stable while DAV asserted; no ATN flip mid
 * byte), so tests can assert on the observed wire, not on core state. */

/* Waveform ring: sized for the LONGEST single scenario -- the chaos
 * stress soaks ~12k rounds per seed (24 deadline-bounded ops with
 * random gaps, drains and settles); wedged scenarios (O14) still opt
 * out via trace_enabled. */
#define ORACLE_TRACE_MAX 16384u
#define ORACLE_CMD_LOG_MAX 64u
#define ORACLE_RX_MAX 256u
/* Per-drive journal: ~2 drives per round across a full chaos seed. */
#define ORACLE_DRIVE_JOURNAL_MAX 32768u

/* Rev10 §4.3: a per-DRIVE journal -- EVERY port->drive() call from the
 * core is recorded, independently of the per-round snapshot ring. A
 * transient forbidden release between two snapshots (glitch + correct
 * pulse inside one round) is invisible to the ring but lands here. */
typedef struct oracle_drive_rec {
    uint8_t atn : 1;
    uint8_t eoi : 1;
    uint8_t dav : 1;
    uint8_t nrfd : 1;
    uint8_t ndac : 1;
    uint8_t ifc : 1;
    uint8_t ren : 1;
    uint8_t srq : 1;
    uint16_t round; /* snapshot-ring index at the time of the drive */
} oracle_drive_rec_t;

/* Independent command classification (own encoding table, 488.1 6). */
typedef enum oracle_cmd {
    OC_OTHER = 0,
    OC_GTL,
    OC_SDC,
    OC_PPC,
    OC_GET,
    OC_TCT,
    OC_LLO,
    OC_DCL,
    OC_PPU,
    OC_SPE,
    OC_SPD,
    OC_UNL,
    OC_UNT,
    OC_MLA,
    OC_MTA,
    OC_SEC
} oracle_cmd_t;

typedef struct oracle_cmdrec {
    oracle_cmd_t cmd;
    uint8_t raw;
    uint8_t arg; /* decoded address / secondary value */
} oracle_cmdrec_t;

typedef struct oracle_trace {
    uint8_t atn : 1;
    uint8_t eoi : 1;
    uint8_t dav : 1;
    uint8_t nrfd : 1;
    uint8_t ndac : 1;
    uint8_t ifc : 1;
    uint8_t ren : 1;
    uint8_t srq : 1;
    uint8_t dio;
} oracle_trace_t;

typedef struct oracle_peer {
    gpib_port_t base; /* the port the CORE talks to */
    vbus_t *bus;
    int core_id; /* participant id of the core (set by the fixture) */
    int self_id; /* our participant id */
    uint64_t now_us;

    /* addressing (own bookkeeping from decoded commands) */
    uint8_t addr;
    bool listen; /* MLA(own) active, not unlistened */
    bool talk;   /* MTA(own) active, not untalked */
    bool spe;    /* serial poll enable armed */

    /* acceptor (hand-written 488.1 handshake) */
    bool dav_prev;
    bool byte_taken; /* captured the current DAV byte, NDAC released */
    uint8_t rx[ORACLE_RX_MAX];
    size_t rx_len;

    /* independent command log */
    oracle_cmdrec_t cmds[ORACLE_CMD_LOG_MAX];
    size_t n_cmds;

    /* talker: scripted source */
    const uint8_t *tx;
    size_t tx_len;
    size_t tx_pos;
    bool tx_armed;
    bool tx_eoi_last;
    bool tx_stall; /* fault injection: never asserts DAV (O6) */
    bool freeze_clock; /* fault injection (O14): snap time stops */
    bool read_fail; /* fault injection (Rev10 §7): read_lines fails */
    bool quiesce_fail; /* fault injection (rev11 N11-06): the port never
                          certifies executor quiescence until cleared */
    bool hold_byte; /* fault injection (O15): the acceptor NEVER releases
                     * NDAC for the next byte -- the core's SH waits mid
                     * cycle, so a cancel strikes a provable in-flight
                     * byte (deterministic C3 scenario) */
    int tx_phase; /* 0=wait_rfd 1=dav_hold 2=wait_ndac_clear */
    bool stb_mode; /* serial-poll answer overrides tx */
    bool stb_sent;

    /* service request */
    bool rsv;
    uint8_t stb;
    bool srq_asserted;

    /* parallel poll responder (direct rule reading, no FSM) */
    bool pp_capable;
    bool pp_configured;
    bool pp_ist;
    bool pp_sense;
    uint8_t pp_line;
    bool pp_window; /* PPC accepted, awaiting PPE/PPD */

    /* counters the tests may assert on */
    unsigned dcl_count;
    unsigned sdc_count;
    unsigned get_count;
    unsigned gtl_count;

    /* waveform + online invariants */
    oracle_trace_t trace[ORACLE_TRACE_MAX];
    size_t trace_len;
    bool trace_overflow;   /* Rev6 §5.4: ring exhausted -- must stay 0 */
    bool trace_enabled;    /* O14 wedged scenarios may disable the ring */
    /* per-drive journal (Rev10 §4.3): captures intra-round glitches */
    oracle_drive_rec_t dr_journal[ORACLE_DRIVE_JOURNAL_MAX];
    size_t dr_n;
    bool dr_overflow;
    /* last observed levels INDEPENDENT of the ring (Rev10 §6.4): the
     * drive_data stability check keeps working with tracing disabled */
    bool last_dav_lvl;
    bool have_last_lvl;
    bool ifc_seen_round; /* the CURRENT round observed an IFC pulse:
                          * mid-byte aborts are legal (AUD-11/derr
                          * exemptions) */
    unsigned violations;    /* invariant breaches (must stay 0) */
    unsigned viol_atn;      /* split for diagnosis: ATN mid-byte flips */
    unsigned viol_conflict; /* split for diagnosis: vbus conflicts */
    uint8_t core_dio_prev;  /* INV2 tracking */
    bool core_dav_prev;
    bool core_atn_prev;     /* INV: no ATN flip while DAV held */
    uint32_t conflicts_prev; /* vbus conflict delta -> violations */

    /* observed core facts */
    bool saw_ifc;
    bool saw_ren;
} oracle_peer_t;

void oracle_init(oracle_peer_t *p, vbus_t *bus, int core_id, uint8_t addr);

/* Public alias of the port read_lines callback: the chaos stress port
 * (test_stress.c) wraps it with adversarial bus events each round. */
bool oracle_port_read_lines(void *user, gpib_line_snap_t *snap);

/* The honest safe_detached evidence function (installed by oracle_init);
 * tests that override the callback can restore it through this alias. */
bool oracle_port_safe_detached(void *user);

/* Script the talker reply (EOI on the last byte when eoi_last). */
void oracle_arm_reply(oracle_peer_t *p, const uint8_t *data, size_t n,
    bool eoi_last);

/* Fault injection (O6): a stalled talker never starts the byte. */
void oracle_stall_talker(oracle_peer_t *p, bool stall);

/* Serial-poll surface: set the status byte and request service. */
void oracle_set_stb(oracle_peer_t *p, uint8_t stb, bool rsv);

/* Parallel-poll surface: capability + individual status. */
void oracle_set_pp(oracle_peer_t *p, bool capable, bool ist);

#endif /* ORACLE_PEER_H */
