# IEEE 488.1-1987 — RL / DC / DT supplement (normative extract)

Status: engineering reformulation, not a verbatim reproduction. Edition:
ANSI/IEEE Std 488.1-1987. Companion to IEEE_488_1_1987_T_L_C_SR.md.
Page references are the standard's printed pages (PDF index = +7).

## 0. Conventions

Entries: FROM — CONDITION — TO. ACDS is the AH accept-data window of the
current command byte; LADS the listener-addressed state. REN, GET, GTL,
LLO, DCL, SDC are IEEE logical messages, not GPIO levels. rtl is a LOCAL
message (return to local), not a bus command. t4 bounds the transition on
REN drop. Command predicates are valid only for ONE accepted byte at
ATN=TRUE qualified by ACDS.

## 1. RL — Remote/Local (§ 2.8; Fig. 10; Tables 21-23, pp. 35-37)

States: LOCS (local), REMS (remote), LWLS (local with lockout),
RWLS (remote with lockout). RL sends no interface messages itself.

    INIT — pon — LOCS
    LOCS — REN && !rtl && MLA && ACDS — REMS
    LOCS — REN && LLO && ACDS — LWLS
    LWLS — MLA && ACDS — RWLS
    LWLS — !REN — LOCS                     [within t4]
    REMS — LLO && ACDS — RWLS
    REMS — !REN — LOCS                     [within t4]
    REMS — GTL && LADS && ACDS — LOCS
    REMS — rtl && (!LLO || !ACDS) — LOCS
    RWLS — !REN — LOCS                     [within t4]
    RWLS — GTL && LADS && ACDS — LWLS

LE note (Fig. 10): with an extended listener, (MSA && LPAS) replaces MLA
in both MLA transitions. rtl: processed in REMS only when not shadowed by
a simultaneously accepted LLO; ignored in LWLS/RWLS; must not be asserted
permanently (§ 2.8.5). GTL keeps lockout (RWLS -> LWLS); REN drop clears
it (-> plain LOCS).

Subsets (Table 23): RL0 none; RL1 full (requires L/LE); RL2 without local
lockout (no LWLS/RWLS, rtl permanently false).

Application note: LOCS does not forbid all GPIB traffic (§ 2.8.5); in
REMS/RWLS the device function must react to subsequent input messages.

## 2. DC — Device Clear (§ 2.10; Fig. 12; Tables 28-30, pp. 41-43)

    DCIS — ACDS && (DCL || (SDC && LADS)) — DCAS
    DCAS — !ACDS || !(DCL || (SDC && LADS)) — DCIS

DC2 (no selected clear): DCL only. DCL is universal (no addressing); SDC
needs LADS. Entering DCAS is the device-function clear event; it does NOT
reset interface functions (that is IFC's job) and does not unaddress.
Recommend one DF event DEVICE_CLEAR with origin DCL/SDC, tied to the
ENTRY into DCAS, not to repeated polling.

Subsets: DC0 none; DC1 DCL+SDC (requires L/LE); DC2 DCL only (AH1).

## 3. DT — Device Trigger (§ 3.11; Fig. 13; Tables 31-33, pp. 43-44)

    DTIS — GET && LADS && ACDS — DTAS
    DTAS — !GET || !LADS || !ACDS — DTIS

GET acts on addressed listeners. One-shot DEVICE_TRIGGER event on DTAS
entry; the started operation continues independently of later DT state
changes; a new operation waits for the previous one (DF contract).

Subsets: DT0 none; DT1 full (requires L/LE).

## 4. Implementation guidance

1. Keep RL/DC/DT as separate FSMs fed by the AH/L command stream and REN
   observation; never merge into one big machine.
2. Qualify commands strictly by ACDS; GTL/SDC/GET additionally need LADS;
   DCL/LLO do not.
3. REN is observed, not owned, by RL.
4. DCAS/DTAS semantics are 488.1-level events; parser/queue resets are
   488.2 obligations, not automatic here.
5. DCAS/DTAS entries are short pulses; delivery must be guaranteed or
   explicitly refused under overload, never silently dropped.
6. pon -> DCIS/DTIS is a project convention (Figs. 12/13 show no pon
   arrow), unlike pon -> LOCS for RL.

## 5. Directed test vectors

RL-01 LOCS,REN=0,MLA -> LOCS; RL-02 LOCS,REN=1,!rtl,MLA -> REMS;
RL-03 LOCS,REN=1,LLO -> LWLS; RL-04 LWLS,MLA -> RWLS;
RL-05 REMS,rtl (no simultaneous LLO) -> LOCS; RL-06 REMS,rtl with LLO&&ACDS -> RWLS;
RL-07 RWLS,rtl -> stays; RL-08 RWLS,addr,GTL -> LWLS;
RL-09 RWLS,REN->0 -> LOCS (t4); RL-10 RL+LE LOCS,REN=1,MSA&&LPAS -> REMS.
DC-01 DCIS,unaddressed,DCL -> DCAS one event; DC-02 DCIS,unaddr,SDC -> stays;
DC-03 DCIS,addr,SDC -> DCAS (DC1); DC-04 same in DC2 -> stays;
DC-05 DCAS ends -> DCIS, addressing kept.
DT-01 DTIS,unaddr,GET -> stays; DT-02 DTIS,addr,GET -> DTAS one event;
DT-03 DTAS ends -> DTIS; DT-04 op continues after DTAS->DTIS;
DT-05 new GET while op running does not start a second op.
