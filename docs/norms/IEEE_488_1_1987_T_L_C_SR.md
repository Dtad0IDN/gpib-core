# IEEE 488.1 — Engineering Extract: T, L, C, TE/LE, SR State Machines

Purpose: source material for an independent C11 FSM implementation of the
GPIB Core. Normative basis: ANSI/IEEE Std 488.1-1987.
Status: self-compiled logic summary, not a verbatim copy of the normative
text and not a compliance certificate. The 1987 edition was later replaced
by the 2003 revision. Page numbers below are the STANDARD's printed pages,
not PDF page numbers (offset +7 in the project copy).

Project verification note (2026-10-23): cross-checked against the already
implemented SH/AH/decoder/addressing modules and the Annex B sequence;
accepted as [B-STD] working basis. See the review notes at the bottom.

## 0. Conventions

Each entry: FROM — CONDITION — TO; &&, ||, ! are AND, OR, NOT. pon is
power-on. If no transition condition holds, the state is retained.

- ATN, IFC, SRQ and other messages are IEEE logical values, not GPIO
  levels. For active-low lines TRUE means the wire is physically LOW.
- ACDS is the Accept Data state of AH: interface command bytes are accepted
  exactly during it. STRS is the Source Transfer state of SH.
- MTA / MLA address THIS talker/listener; OTA is another talker's address;
  OSA another secondary address; MSA matches our own secondary address.
- ton, lon, ltn, lun, gts, tcs, tca, rpp, sic, sre, rsc, rsv are local
  messages of the respective functions, not DIO byte codes.
- t2, t4 are exit-within intervals; T6..T10 are controller holds. These are
  NOT the subset numbers T1..T8.
- CACS, TADS, SPAS etc. in conditions are state-activity predicates.
  !SPMS means serial poll mode is not active.

## 1. Talker (T), § 2.5, Fig. 5, Tables 8-9, 11 (pp. 21-27)

States: TIDS idle; TADS addressed, waiting for ATN false; TACS transferring
data bytes; SPIS serial poll disabled; SPMS serial poll enabled; SPAS
status byte transfer during serial poll. SPIS/SPMS are a separate pair;
they must not be replaced by TIDS/TACS.

    INIT — pon — TIDS
    TIDS — !IFC && ((MTA && ACDS) || ton) — TADS
    TADS — !ATN && !SPMS — TACS
    TADS — !ATN && SPMS — SPAS
    TADS — ACDS && OTA — TIDS
    TADS — ACDS && MLA — TIDS                 [optional per standard]
    TADS — IFC — TIDS                          [within t4]
    TACS — ATN — TADS                          [within t2]
    TACS — IFC — TIDS                          [within t4]
    SPAS — ATN — TADS                          [within t2]
    SPAS — IFC — TIDS                          [within t4]
    INIT — pon — SPIS
    SPIS — !IFC && SPE && ACDS — SPMS
    SPMS — SPD && ACDS — SPIS
    SPMS — IFC — SPIS                          [within t4]

- In TACS, DAB/END are transferred under control of a separate SH FSM;
  not "automatically because of TACS".
- In SPAS the STB with the RQS bit is sent; RQS is formed with the SR FSM.
  Re-sending the STB without a new ATN is permitted; RQS must not be
  cleared outside the SR rules.
- In TIDS/TADS the device sends no useful data.

### 1.1 Talker subsets — Table 11, p. 27

Subset | Basic talker | Serial poll | Talk-only (ton) | Auto-unaddress on MLA
T0 no no no no; T1 yes yes yes no; T2 yes yes no no; T3 yes no yes no;
T4 yes no no no; T5 yes yes yes yes; T6 yes yes no yes; T7 yes no yes yes;
T8 yes no no yes.

T3/T4/T7/T8 have no SPIS/SPMS/SPAS. In T2/T4/T6/T8 ton is permanently
FALSE. T5-T8 require a matching Listener (L1-L4 or LE1-LE4) and the
MLA-unaddress transition. All non-zero T require SH1 and AH1. T1/T3 do not
auto-unaddress on MLA; T5/T7 do.

## 2. Listener (L), § 2.6, Fig. 7, Tables 13-14, 16 (pp. 28-31)

LIDS not addressed; LADS addressed, data not yet active; LACS accepting.
L itself sends nothing; actual reception is the AH FSM.

    INIT — pon — LIDS
    LIDS — !IFC && ((MLA && ACDS) || lon || (ltn && CACS)) — LADS
    LADS — !ATN — LACS                         [within t2]
    LADS — UNL && ACDS — LIDS
    LADS — lun && CACS — LIDS
    LADS — MTA && ACDS — LIDS                 [optional per standard]
    LADS — IFC — LIDS                          [within t4]
    LACS — ATN — LADS                          [within t2]
    LACS — IFC — LIDS                          [within t4]

In LACS one data byte is delivered to the application per ACDS event, not
per GPIO read. During LADS no application data is delivered.

### 2.1 Listener subsets — Table 16, p. 31

Subset | Basic listener | Listen-only (lon) | Auto-unaddress on MTA
L0 no no no; L1 yes yes no; L2 yes no no; L3 yes yes yes; L4 yes no yes.

All non-zero L require AH1. L2/L4: lon permanently FALSE. L3/L4 have the
LADS -> LIDS on own MTA transition; the standard ties them to T/TE
capabilities.

## 3. Extended Talker / Listener (TE/LE), §§ 2.5-2.6, Figs. 6, 8

Secondary addressing adds an armed pair of states; the addressed state is
entered only on a matching secondary after a matching primary.

    # TE pair
    INIT — pon — TPIS
    TPIS — MTA && ACDS — TPAS
    TPAS — PCG && !MTA && ACDS — TPIS
    # TE entry into TADS
    TIDS — !IFC && ((MSA && ACDS && TPAS) || ton) — TADS
    # TE re-address exits
    TADS — OTA && ACDS — TIDS
    TADS — OSA && TPAS && ACDS — TIDS
    TADS — MSA && LPAS && ACDS — TIDS       [optional]
    TADS — IFC — TIDS                        [within t4]

    # LE pair
    INIT — pon — LPIS
    LPIS — MLA && ACDS — LPAS
    LPAS — PCG && !MLA && ACDS — LPIS
    # LE entry into LADS
    LIDS — !IFC && ((MSA && ACDS && LPAS) || lon || (ltn && CACS)) — LADS
    # LE LADS -> LIDS on secondary
    LADS — UNL && ACDS — LIDS
    LADS — lun && CACS — LIDS
    LADS — MSA && TPAS && ACDS — LIDS       [optional]
    LADS — IFC — LIDS                        [within t4]

The remaining TADS<->TACS/SPAS, SPIS<->SPMS, LADS<->LACS transitions are
unchanged. TE0-TE8 / LE0-LE4 mirror T/L with extended addressing.

## 4. Service Request (SR), § 2.7, Fig. 9; Tables 18-20 (pp. 32-34)

NPRS no request; SRQS request asserted on the bus; APRS request retained
for the serial poll answer while SRQ is released.

    INIT — pon — NPRS
    NPRS — rsv && !SPAS — SRQS
    SRQS — !rsv && !SPAS — NPRS
    SRQS — SPAS — APRS
    APRS — !rsv && !SPAS — NPRS

- In SRQS, SRQ is TRUE (wire LOW). In APRS, SRQ is released and the
  polled talker sends RQS=1 in the status byte.
- No direct APRS -> SRQS return exists; the old SRQ must not be
  re-asserted spontaneously after the poll.
- SR0: function absent; SR1: full FSM. SR1 requires a serial-poll-capable
  talker subset (T1/T2/T5/T6 or TE equivalents).

## 5. Controller (C), § 2.12, Fig. 14; Tables 34-37 (pp. 45-55)

IMPORTANT: the controller is NOT one six-state FSM. The standard defines
several independent parallel groups (20 unique mnemonics total). Do not
implement them as a single enum without decomposition.

### 5.1 Transfer/command/poll group (10 states)

CIDS, CADS, CACS, CTRS, CSBS, CSHS, CSWS, CAWS, CPWS, CPPS.

    INIT — pon — CIDS
    CIDS — (!IFC && TCT && TADS && ACDS) || SIAS — CADS
    CADS — !ATN — CACS
    CADS — IFC && !SACS — CIDS             [within t4]
    CACS — TCT && ACDS && !TADS — CTRS   [!TADS may be omitted]
    CACS — rpp && !SDYS && !STRS — CPWS
    CACS — gts && !SDYS && !STRS — CSBS
    CACS — IFC && !SACS — CIDS             [within t4]
    CTRS — !STRS — CIDS
    CTRS — IFC && !SACS — CIDS             [within t4]
    CSBS — tcs && ANRS — CSHS
    CSBS — tca — CSWS
    CSBS — IFC && !SACS — CIDS             [within t4]
    CSHS — elapsed(T10) — CSWS
    CSHS — !tcs — CSBS
    CSHS — IFC && !SACS — CIDS             [within t4]
    CSWS — elapsed(T7) || TADS — CAWS
    CSWS — IFC && !SACS — CIDS             [within t4]
    CAWS — !rpp && elapsed(T9) — CACS
    CAWS — rpp — CPWS
    CAWS — IFC && !SACS — CIDS             [within t4]
    CPWS — elapsed(T6) — CPPS
    CPWS — !rpp — CAWS
    CPWS — IFC && !SACS — CIDS             [within t4]
    CPPS — !rpp — CAWS
    CPPS — IFC && !SACS — CIDS             [within t4]

Notes:
- In CACS ATN is TRUE and the controller sends interface bytes via SH.
- In CSBS ATN is FALSE for the talker/listener transfer.
- In CSHS ATN is still FALSE (synchronous takeover waits for a safe
  moment); in CSWS it is already TRUE.
- In CPWS/CPPS ATN and IDY are active; CPWS waits for line settling,
  CPPS reads the PPR lines.
- In CTRS the TCT byte must fully complete before releasing the role.
- tca (asynchronous takeover) is dangerous mid-byte: § 2.12.5 warns about
  misinterpreting data as a command.

### 5.2 SRQ monitor pair

    CSNS — SRQ — CSRS
    CSRS — !SRQ — CSNS

Reflects the bus-wide request, not this device's own SR function.

### 5.3 System Controller authority

    SNAS — rsc — SACS
    SACS — !rsc — SNAS

SACS grants the right to drive IFC and REN regardless of who is CIC.

### 5.4 IFC control

    INIT — pon — SIIS
    SIIS — SACS && !sic — SINS
    SIIS — SACS && sic — SIAS
    SINS — sic — SIAS
    SINS — !SACS — SIIS
    SIAS — !sic && elapsed(T8) — SINS
    SIAS — !SACS — SIIS

SIIS: no right; SINS: IFC released; SIAS: IFC asserted. The T8 hold on
SIAS -> SINS is mandatory.

### 5.5 REN control

    INIT — pon — SRIS
    SRIS — SACS && !sre — SRNS
    SRIS — SACS && sre && elapsed(T8 in SRIS) — SRAS
    SRNS — sre && elapsed(T8 of sre) — SRAS
    SRNS — !SACS — SRIS
    SRAS — !sre — SRNS
    SRAS — !SACS — SRIS

SRIS: no right; SRNS: REN not asserted; SRAS: REN asserted. When leaving
SACS the REN driver must be disabled safely.

### 5.6 Controller subsets, Table 37, p. 55

C1-C4 are independent capabilities; one of C5-C28 is additionally chosen
for the interface-message mechanics. C0: no controller. C1: system
controller authority. C2: send IFC and take control. C3: drive REN.
C4: respond to SRQ. C5: full capability set (send messages, receive/transfer
control, pass to self, parallel poll, synchronous takeover). C6-C28:
restricted combinations per the original Table 37.

A full multi-controller profile targets C1+C2+C3+C4+C5, provided all are
really implemented and test-verified. C5 does not replace C1-C4.

## 6. Address/command byte encoding (reference)

See § 2.13, Table 38, pp. 59-61. ATN=TRUE makes DIO interface commands:

    GTL 0x01  SDC 0x04  PPC 0x05  GET 0x08
    TCT 0x09  LLO 0x11  DCL 0x14  PPU 0x15
    SPE 0x18  SPD 0x19  UNL 0x3F  UNT 0x5F

    LAD(a) = 0x20 | (a & 0x1F)
    TAD(a) = 0x40 | (a & 0x1F)
    SAD(a) = 0x60 | (a & 0x1F)

0x3F and 0x5F are UNL/UNT, never plain addresses.

## 7. Porting guidance

1. Model T, L, SR as separate FSMs; decompose the controller into the 5
   independent groups. States of different groups may be active at once.
2. Decode interface bytes only on completed AH acceptance (ACDS) with the
   matching ATN, never on a DIO edge.
3. Respect IFC priority and electrical safety; do not confuse logical TRUE
   with pin voltage.
4. Move t2, t4, T6-T10 into the portable time layer; verify numeric limits
   against the same edition of the standard and the port data.
5. Negative tests required: TADS->TIDS on OTA, LADS->LIDS on UNL,
   SRQS->APRS->NPRS, CSBS->CSHS->CSWS->CAWS->CACS, CACS->CTRS->CIDS.
6. Before using this text as the normative matrix, cross-check against
   Figs. 5-9, 14 and Tables 11-12, 16-17, 20, 37 of the source document
   (especially optional conditions and simultaneous-event conflicts).

Limitations: SH/AH, PP, RL, DC, DT semantics, timing values, physical
drivers and electrical limits are intentionally not reproduced here.

## Project review notes (verification round)

Accepted as [B-STD] after the following observations:

1. Source edition: the cited download is named ".../IEEE-488_78.pdf", but
   the transcribed table numbering (Table 39 timing, Table 37, Fig. 14)
   matches the 1987 edition. Baseline stays 488.1-1987; page references
   keep the author's +7 offset note until the edition is re-confirmed.
2. The !IFC qualifiers on TIDS->TADS and SPIS->SPMS and the "IFC && !SACS"
   exits are defensive renditions rather than verbatim diagram arrows:
   safe to implement, marked as engineering annotations.
3. CSWS -> CAWS guard "(T7 elapsed) || TADS": verify against Fig. 14
   before relying on the TADS disjunct in multi-controller scenarios.
4. L3/L4 "auto-unaddress on own MTA" is a SUBSET feature: L1/L2 keep
   LADS on MTA(own). The core's current primary-addressing engine
   implements the L3/L4 behavior unconditionally; the full L FSM must
   gate this on the chosen subset.
5. ltn/lun (CACS-gated controller listen control) are new local messages
   not yet modeled in the core; required for the controller-as-listener
   path of the full L FSM.
6. Terminology mapping to the core decoder: LAD=MLA, TAD=MTA, SAD=MSA;
   the byte values agree with gpib_decode_atn_byte exactly.
