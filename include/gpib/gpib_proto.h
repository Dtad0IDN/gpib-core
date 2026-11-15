#ifndef GPIB_PROTO_H
#define GPIB_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IEEE 488.1-1987 protocol engine modules ([B-STD] working basis,
 * docs/norms/IEEE_488_1_1987_SH_AH_Commands.md).
 *
 * Pure functions of their inputs: no port access, no context access. The
 * future line-level executor feeds observed line messages and local role
 * states in, applies the requested line states out, and forwards the
 * events. All signal names are IEEE LOGICAL messages: TRUE means the
 * active-low wire is physically LOW. */

/* --- ATN command decoding (--4) ------------------------------------------- */

typedef enum gpib_ifcmd_class {
    GPIB_CMD_UNDEFINED = 0, /* handshaken but not executed */
    GPIB_CMD_GTL,
    GPIB_CMD_SDC,
    GPIB_CMD_PPC,
    GPIB_CMD_GET,
    GPIB_CMD_TCT,
    GPIB_CMD_LLO,
    GPIB_CMD_DCL,
    GPIB_CMD_PPU,
    GPIB_CMD_SPE,
    GPIB_CMD_SPD,
    GPIB_CMD_UNL,
    GPIB_CMD_UNT,
    GPIB_CMD_MLA,     /* arg = 0..30 (0x3F is UNL, not MLA31) */
    GPIB_CMD_MTA,     /* arg = 0..30 (0x5F is UNT, not MTA31) */
    GPIB_CMD_SECONDARY /* arg = 0..31; PPE/PPD interpretation is contextual */
} gpib_ifcmd_class_t;

typedef struct gpib_ifcmd {
    gpib_ifcmd_class_t cls;
    uint8_t raw; /* raw DIO byte as observed */
    uint8_t arg; /* address / secondary code where applicable */
} gpib_ifcmd_t;

/* Decode one accepted ATN-mode byte. DIO8 (bit 7) does not participate. */
gpib_ifcmd_t gpib_decode_atn_byte(uint8_t dio_byte);

/* --- ATN command byte builders (symmetric to the decoder) --- */

/* Fixed (unaddressed) command bytes, IEEE 488.1 Table 6.2. */
#define GPIB_UNL_BYTE 0x3Fu
#define GPIB_UNT_BYTE 0x5Fu
#define GPIB_SPE_BYTE 0x18u
#define GPIB_SPD_BYTE 0x19u
#define GPIB_TCT_BYTE 0x09u
#define GPIB_PPC_BYTE 0x05u
#define GPIB_PPU_BYTE 0x15u

/* Addressed command bytes: 5-bit primary/secondary address fields. */
static inline uint8_t gpib_mk_mla(uint8_t primary) {
    return (uint8_t)(0x20u | (primary & 0x1Fu));
}
static inline uint8_t gpib_mk_mta(uint8_t primary) {
    return (uint8_t)(0x40u | (primary & 0x1Fu));
}
static inline uint8_t gpib_mk_sad(uint8_t secondary) {
    return (uint8_t)(0x60u | (secondary & 0x1Fu));
}
/* PPE secondary: 110S P P P (S = sense bit, PPP = DIO line 0-7). */
static inline uint8_t gpib_mk_ppe(uint8_t sense, uint8_t dio_line) {
    return (uint8_t)(0x60u | (sense ? 0x08u : 0u) | (dio_line & 0x07u));
}

/* --- primary addressing engine (structural; [P] pending the T/L tables) --
*
* Tracks primary addressing from decoded ATN commands using the unambiguous
 * 1987 semantics: UNL unlistens every listener; MTA(own) unlistens and
 * talk-addresses the device; MTA(other) untalks the previous talker and
 * leaves listeners untouched; MLA(other) does NOT unlisten (multiple
 * simultaneous listeners are legal, this is the write_many basis).
 * Secondary addressing (TE/LE) is pending. */

typedef enum gpib_addr_event {
    GPIB_AE_NONE = 0,
    GPIB_AE_LISTEN_ON,
    GPIB_AE_LISTEN_OFF,
    GPIB_AE_TALK_ON,
    GPIB_AE_TALK_OFF,
    /* Any fixed device command (GTL/SDC/PPC/GET/TCT/LLO/DCL/PPU/SPE/SPD):
     * the engine passes it through; addressed-vs-universal filtering by
     * lads is the consumer's decision ([P]). */
    GPIB_AE_DEVICE_CMD
} gpib_addr_event_t;

typedef struct gpib_addr_engine {
    bool lads;          /* listener addressed (primary) */
    bool tads;          /* talker addressed (primary) */
    uint8_t own_primary; /* 0..30 (31 is UNL/UNT aliasing, never an address) */
} gpib_addr_engine_t;

void gpib_addr_init(gpib_addr_engine_t *e, uint8_t own_primary);

/* Process one decoded ATN command; returns the change event, if any. */
gpib_addr_event_t gpib_addr_step(gpib_addr_engine_t *e,
    const gpib_ifcmd_t *cmd);

/* --- Talker function (--2.5, Fig. 5) ---------------------------------------
 * Data-role states plus the separate serial-poll pair; the pair exists
 * only in serial-poll-capable subsets (T1/T2/T5/T6). */

typedef enum gpib_t_state {
    GPIB_T_TIDS = 0,
    GPIB_T_TADS,
    GPIB_T_TACS,
    GPIB_T_SPAS
} gpib_t_state_t;

typedef enum gpib_t_sp_state {
    GPIB_T_SPIS = 0,
    GPIB_T_SPMS
} gpib_t_sp_state_t;

/* TE (extended talker) secondary-addressing pair (extract 3):
 * TPIS --MTA&&ACDS--> TPAS; TPAS --PCG&&!MTA&&ACDS--> TPIS. */
typedef enum gpib_t_sec_state {
    GPIB_T_TPIS = 0, /* talker primary idle (secondary not armed) */
    GPIB_T_TPAS      /* talker primary addressed state (armed for MSA) */
} gpib_t_sec_state_t;

typedef struct gpib_talker {
    gpib_t_state_t state;
    gpib_t_sp_state_t sp;
    gpib_t_sec_state_t sec;  /* TE pair: TPIS/TPAS */
    bool secondary_capable;  /* TE5-TE8; plain T when false */
    uint8_t own_secondary;   /* MSA(n) that completes our addressing */
    bool serial_poll_capable; /* T1/T2/T5/T6 */
    bool unaddr_on_mla;       /* T5-T8 */
    uint8_t own_primary;
} gpib_talker_t;

typedef struct gpib_t_in {
    bool ifc;
    bool atn;
    bool ton;          /* local talk-only (permanently false for T2/T4/T6/T8) */
    bool cmd_valid;    /* one interface command accepted in ACDS this step */
    gpib_ifcmd_t cmd;
} gpib_t_in_t;

typedef struct gpib_t_out {
    bool talk_addressed; /* entered TADS */
    bool talk_released;  /* left to TIDS */
    bool entered_spas;
} gpib_t_out_t;

void gpib_talker_init(gpib_talker_t *t, uint8_t own_primary,
    bool serial_poll_capable, bool unaddr_on_mla);
void gpib_talker_init_ex(gpib_talker_t *t, uint8_t own_primary,
    bool serial_poll_capable, bool unaddr_on_mla,
    bool secondary_capable, uint8_t own_secondary);
void gpib_talker_step(gpib_talker_t *t, const gpib_t_in_t *in,
    gpib_t_out_t *out);

/* --- Listener function (--2.6, Fig. 7) ------------------------------------- */

typedef enum gpib_l_state {
    GPIB_L_LIDS = 0,
    GPIB_L_LADS,
    GPIB_L_LACS
} gpib_l_state_t;

/* LE (extended listener) secondary-addressing pair (extract 3):
 * LPIS --MLA&&ACDS--> LPAS; LPAS --PCG&&!MLA&&ACDS--> LPIS. */
typedef enum gpib_l_sec_state {
    GPIB_L_LPIS = 0,
    GPIB_L_LPAS
} gpib_l_sec_state_t;

typedef struct gpib_listener {
    gpib_l_state_t state;
    gpib_l_sec_state_t sec;  /* LE pair: LPIS/LPAS */
    bool secondary_capable;  /* LE3-LE4; plain L when false */
    uint8_t own_secondary;   /* MSA(n) that completes our addressing */
    bool unaddr_on_mta; /* L3/L4 subset feature */
    uint8_t own_primary;
} gpib_listener_t;

typedef struct gpib_l_in {
    bool ifc;
    bool atn;
    bool lon;
    bool ltn; /* controller local listen request (gated by CACS) */
    bool lun;
    bool cacs;
    bool cmd_valid;
    gpib_ifcmd_t cmd;
} gpib_l_in_t;

typedef struct gpib_l_out {
    bool listen_addressed; /* entered LADS */
    bool listen_released;  /* left to LIDS */
} gpib_l_out_t;

void gpib_listener_init(gpib_listener_t *l, uint8_t own_primary,
    bool unaddr_on_mta);
void gpib_listener_init_ex(gpib_listener_t *l, uint8_t own_primary,
    bool unaddr_on_mta, bool secondary_capable, uint8_t own_secondary);
void gpib_listener_step(gpib_listener_t *l, const gpib_l_in_t *in,
    gpib_l_out_t *out);

/* --- Service Request function (--2.7, Fig. 9) ------------------------------ */

typedef enum gpib_sr_state {
    GPIB_SR_NPRS = 0,
    GPIB_SR_SRQS,
    GPIB_SR_APRS
} gpib_sr_state_t;

typedef struct gpib_sr {
    gpib_sr_state_t state;
} gpib_sr_t;

typedef struct gpib_sr_in {
    bool rsv;  /* local service request message */
    bool spas; /* our talker is in SPAS */
} gpib_sr_in_t;

typedef struct gpib_sr_out {
    bool srq_asserted; /* state SRQS: SRQ line pulled */
    bool rqs;          /* state APRS: poll answer carries RQS=1 */
} gpib_sr_out_t;

void gpib_sr_init(gpib_sr_t *sr);
void gpib_sr_step(gpib_sr_t *sr, const gpib_sr_in_t *in, gpib_sr_out_t *out);

/* --- Controller function (--2.12, Fig. 14) ---------------------------------
 * Decomposed into five INDEPENDENT groups per the normative extract; states
 * of different groups may be active simultaneously. The main group uses
 * hold timers T6/T7/T9/T10; the IFC/REN drivers use T8. */

typedef enum gpib_c_state {
    GPIB_C_CIDS = 0,
    GPIB_C_CADS,
    GPIB_C_CACS,
    GPIB_C_CTRS,
    GPIB_C_CSBS,
    GPIB_C_CSHS,
    GPIB_C_CSWS,
    GPIB_C_CAWS,
    GPIB_C_CPWS,
    GPIB_C_CPPS
} gpib_c_state_t;

typedef enum gpib_c_srq_state { GPIB_C_CSNS = 0, GPIB_C_CSRS } gpib_c_srq_state_t;
typedef enum gpib_c_sc_state { GPIB_C_SNAS = 0, GPIB_C_SACS } gpib_c_sc_state_t;
typedef enum gpib_c_ifc_state { GPIB_C_SIIS = 0, GPIB_C_SINS, GPIB_C_SIAS } gpib_c_ifc_state_t;
typedef enum gpib_c_ren_state { GPIB_C_SRIS = 0, GPIB_C_SRNS, GPIB_C_SRAS } gpib_c_ren_state_t;

typedef struct gpib_controller {
    gpib_c_state_t main;
    gpib_c_srq_state_t srq;
    gpib_c_sc_state_t sc;
    gpib_c_ifc_state_t ifc;
    gpib_c_ren_state_t ren;
    /* Independent hold timers: the groups run in parallel and must never
     * share a deadline. */
    uint64_t main_deadline;
    bool main_hold;
    /* AUD-21: wrap-safe hold windows -- (now - start) >= duration;
     * an absolute `now + dur` deadline wraps at UINT64_MAX and let the
     * hold expire instantly. Deadlines above remain for compat but are
     * no longer authoritative. */
    uint64_t main_start_us;
    uint64_t main_dur_us;
    uint64_t ifc_start_us;
    uint64_t ifc_dur_us;
    uint64_t ren_t8_us;
    uint64_t ifc_deadline;
    bool ifc_hold;
    uint64_t ren_deadline;
    bool ren_hold;
    /* F14: timestamp of the SRAS->SRNS transition -- the moment OUR REN
     * driver actually released the line. Re-assertion requires
     * now_us >= ren_released_us + t8_us (REN FALSE >= 100 us, 488.2
     * 16.2.11 / NAT4882 sre), anchored to the WIRE edge, not to the
     * repeat of the request. */
    uint64_t ren_released_us;
} gpib_controller_t;

typedef struct gpib_c_in {
    /* remote facts */
    bool atn;          /* observed ATN line */
    bool ifc;          /* observed IFC line (someone's SIAS) */
    bool srq;          /* observed SRQ line */
    bool tct_received; /* TCT accepted in ACDS while our talker is TADS */
    bool tads;         /* our talker in TADS */
    bool anrs;         /* our acceptor in ANRS */
    bool sdys, strs;   /* our source handshake states */
    /* local messages */
    bool send_tct;     /* we are sending TCT from CACS (transfer away) */
    bool gts, tcs, tca, rpp, rsc, sic, sre;
    uint64_t now_us;
    uint32_t t6_us, t7_us, t8_us, t9_us, t10_us;
} gpib_c_in_t;

typedef struct gpib_c_out {
    bool cic;          /* controller-in-charge (main != CIDS) */
    bool cacs;         /* active controller state: send commands via SH */
    bool atn_request;  /* main group wants ATN asserted */
    bool ifc_assert;   /* our IFC driver in SIAS */
    bool ren_assert;   /* our REN driver in SRAS */
    bool srq_active;   /* SRQ monitor in CSRS */
    bool became_cic;
    bool lost_cic;
} gpib_c_out_t;

void gpib_controller_init(gpib_controller_t *c);
void gpib_controller_step(gpib_controller_t *c, const gpib_c_in_t *in,
    gpib_c_out_t *out);

/* --- Remote/Local function (--2.8, Fig. 10) -------------------------------- */

typedef enum gpib_rl_state {
    GPIB_RL_LOCS = 0,
    GPIB_RL_REMS,
    GPIB_RL_LWLS,
    GPIB_RL_RWLS
} gpib_rl_state_t;

typedef struct gpib_rl {
    gpib_rl_state_t state;
    bool lockout_capable; /* RL1; RL2 has no LWLS/RWLS */
} gpib_rl_t;

typedef struct gpib_rl_in {
    bool ren;         /* observed REN line (logical TRUE = LOW) */
    bool rtl;         /* local return-to-local request (must not be steady) */
    bool lads;        /* listener addressed (or LE addressed) */
    bool cmd_valid;   /* one interface command accepted in ACDS this step */
    gpib_ifcmd_t cmd; /* MLA / LLO / GTL candidates */
} gpib_rl_in_t;

typedef struct gpib_rl_out {
    gpib_rl_state_t entered; /* state entered this step (one-shot events) */
    bool changed;
} gpib_rl_out_t;

void gpib_rl_init(gpib_rl_t *rl, bool lockout_capable);
void gpib_rl_step(gpib_rl_t *rl, const gpib_rl_in_t *in, gpib_rl_out_t *out);

/* --- Device Clear function (--2.10, Fig. 12) -------------------------------- */

typedef enum gpib_dc_state { GPIB_DC_DCIS = 0, GPIB_DC_DCAS } gpib_dc_state_t;

typedef struct gpib_dc {
    gpib_dc_state_t state;
    bool sdc_capable; /* DC1; DC2 reacts to DCL only */
} gpib_dc_t;

typedef struct gpib_dc_in {
    bool acds; /* the command byte is in its ACDS acceptance window */
    bool dcl;  /* DCL byte is the current one */
    bool sdc;  /* SDC byte is the current one */
    bool lads;
} gpib_dc_in_t;

typedef struct gpib_dc_out {
    bool clear_pulse; /* one-shot on ENTRY into DCAS */
} gpib_dc_out_t;

void gpib_dc_init(gpib_dc_t *dc, bool sdc_capable);
void gpib_dc_step(gpib_dc_t *dc, const gpib_dc_in_t *in, gpib_dc_out_t *out);

/* --- Device Trigger function (--2.11, Fig. 13) ------------------------------ */

typedef enum gpib_dt_state { GPIB_DT_DTIS = 0, GPIB_DT_DTAS } gpib_dt_state_t;

typedef struct gpib_dt {
    gpib_dt_state_t state;
} gpib_dt_t;

typedef struct gpib_dt_in {
    bool get;   /* GET byte is the current one */
    bool lads;
    bool acds;
} gpib_dt_in_t;

typedef struct gpib_dt_out {
    bool trigger_pulse; /* one-shot on ENTRY into DTAS */
} gpib_dt_out_t;

void gpib_dt_init(gpib_dt_t *dt);
void gpib_dt_step(gpib_dt_t *dt, const gpib_dt_in_t *in, gpib_dt_out_t *out);

/* --- Parallel Poll function (488.1 --2.9, PP1) ------------------------------
 * Verified against docs/norms/IEEE_488_1_1987_PP.md (Fig. 11, Tables
 * 24-27). TWO independent state groups:
 *   Group A (response):  PPIS --[PPE&&PACS&&ACDS]--> PPSS --[ATN&&IDY]-->
 *     PPAS --[!ATN||!IDY]--> PPSS (all within t5).
 *   Group B (configure): PUCS --[PPC&&LADS&&ACDS]--> PACS
 *     --[PCG&&!PPC&&ACDS]--> PUCS.
 * PP1 (Table 27) includes the remote PPE/PPD arcs and excludes lpe;
 * PP0 = not capable. In PPAS the assigned PPR line is driven TRUE iff
 * ist == S (Table 25 / 2.9.3.3); ist != S sends passive false. PPU
 * unconfigures without PACS; PPE/PPD are honored only in PACS (PUCS
 * "shall ignore any PPE or PPD messages", 2.9.3.4). DCL has NO PP arc
 * in 2.9; IFC reset of PP comes from the IFC section ([P] extract
 * pending, see the mock's IFC handler). */

typedef enum gpib_pp_state {
    GPIB_PP_PPIS = 0, /* idle: not configured */
    GPIB_PP_PPSS,     /* standby: configured, waiting for ATN&&IDY */
    GPIB_PP_PPAS      /* active: responding to the parallel poll */
} gpib_pp_state_t;

typedef enum gpib_pp_cfg_state {
    GPIB_PP_PUCS = 0, /* unaddressed to configure: ignore PPE/PPD */
    GPIB_PP_PACS      /* addressed to configure: act on PPE/PPD */
} gpib_pp_cfg_state_t;

typedef struct gpib_pp {
    gpib_pp_state_t state;     /* Group A (response) */
    gpib_pp_cfg_state_t cfg;   /* Group B (configure) */
    bool capable;       /* PP1; PP0 when false */
    uint8_t dio_line;   /* PPE P1-P3: which PPR message (Table 26) */
    bool sense;         /* PPE S bit: respond true when ist == S */
    bool ist;           /* individual status (local, device-specific) */
} gpib_pp_t;

typedef struct gpib_pp_in {
    bool cmd_valid;      /* a handshaken ATN byte (ACDS occurred) */
    gpib_ifcmd_t cmd;   /* PPC, PPE/PPD (secondary), PPU, other PCG */
    bool lads;          /* listener addressed (PUCS -> PACS gate) */
    bool atn;
    bool eoi;           /* IDY = ATN && EOI */
    bool ist;           /* the device's current individual status */
} gpib_pp_in_t;

typedef struct gpib_pp_out {
    bool ppas_active;  /* Group A is in PPAS this step */
    uint8_t drive_line; /* assigned PPR line (valid when ppas_active) */
    bool drive_level;   /* true = drive the line (ist == S); false =
                         * passive false (ist != S) — Table 25 */
    bool config_changed;
} gpib_pp_out_t;

void gpib_pp_init(gpib_pp_t *pp, bool capable);
void gpib_pp_step(gpib_pp_t *pp, const gpib_pp_in_t *in, gpib_pp_out_t *out);

/* --- Source Handshake (--2) ----------------------------------------------- */

typedef enum gpib_sh_state {
    GPIB_SH_SIDS = 0,
    GPIB_SH_SGNS,
    GPIB_SH_SDYS,
    GPIB_SH_STRS,
    GPIB_SH_SWNS,
    GPIB_SH_SIWS
} gpib_sh_state_t;

typedef struct gpib_sh {
    gpib_sh_state_t state;
    uint64_t t1_start_us;    /* R17-03: T1 window START (wrap-safe) */
    uint64_t t1_deadline_us; /* T1 settle window DURATION (wrap-safe form) */
    bool in_flight;          /* byte interrupted in STRS: fate unproven */
    bool require_dac_low;    /* optional !DAC guard of the SDYS->STRS row:
                                refuse to transfer into a passive (empty)
                                bus where RFD and DAC are both TRUE */
} gpib_sh_t;

typedef struct gpib_sh_in {
    /* Role inputs. */
    bool atn;
    bool tacs, spas, cacs, ctrs;
    /* Local message from the byte producer. */
    bool nba;
    /* Observed logical line messages (wired-OR of all acceptors). */
    bool rfd; /* RFD=1 <-> NRFD physically HIGH */
    bool dac; /* DAC=1 <-> NDAC physically HIGH */
    /* Policy parameters. */
    uint64_t now_us;
    uint32_t t1_us; /* settle policy: >= 2000 ns open-collector, see Table 39 */
} gpib_sh_in_t;

typedef struct gpib_sh_out {
    bool dav_requested;  /* deterministic policy: TRUE only in STRS */
    bool may_change_byte;/* SIDS/SGNS/SWNS/SIWS per the baseline */
    bool byte_accepted;  /* STRS -> SWNS: accepted by ALL acceptors */
    bool role_lost;      /* left a live state due to SH_LOSES_ROLE */
} gpib_sh_out_t;

void gpib_sh_init(gpib_sh_t *sh);
/* One FSM evaluation; all absent transitions retain the current state.
 * Deterministic conflict policy: SH_LOSES_ROLE wins over data progress
 * (fail-safe: stop driving before transferring). */
void gpib_sh_step(gpib_sh_t *sh, const gpib_sh_in_t *in, gpib_sh_out_t *out);

/* --- Acceptor Handshake (--3) --------------------------------------------- */

typedef enum gpib_ah_state {
    GPIB_AH_AIDS = 0,
    GPIB_AH_ANRS,
    GPIB_AH_ACRS,
    GPIB_AH_ACDS,
    GPIB_AH_AWNS
} gpib_ah_state_t;

typedef struct gpib_ah {
    gpib_ah_state_t state;
    uint64_t t3_start_us;    /* R17-04: T3 window START (wrap-safe) */
    uint64_t t3_deadline_us; /* T3 accept window DURATION (wrap-safe form) */
} gpib_ah_t;

typedef struct gpib_ah_in {
    bool atn;
    bool lads, lacs;
    bool tcs;  /* take control synchronously (controller FSM) */
    bool rdy;  /* local message from the consuming side */
    bool dav;  /* observed logical DAV (TRUE = asserted LOW) */
    uint64_t now_us;
    uint32_t t3_us; /* implementation-dependent, strictly positive */
} gpib_ah_in_t;

typedef struct gpib_ah_out {
    bool rfd_true;       /* release NRFD (RFD=1); assert NRFD when FALSE */
    bool dac_true;       /* release NDAC (DAC=1); assert NDAC when FALSE */
    bool byte_valid;     /* entered ACDS: capture DIO/EOI/ATN consistently */
    bool accepted_event; /* ACDS -> AWNS: byte accepted */
} gpib_ah_out_t;

void gpib_ah_init(gpib_ah_t *ah);
/* Deterministic conflict policy: AH_NOT_ADDRESSED wins over data progress
 * (passive release of NRFD/NDAC). */
void gpib_ah_step(gpib_ah_t *ah, const gpib_ah_in_t *in, gpib_ah_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GPIB_PROTO_H */
