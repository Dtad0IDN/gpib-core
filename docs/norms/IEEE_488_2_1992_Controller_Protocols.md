# IEEE 488.2-1992 — Controller Protocol Requirements (normative extract)

Status: engineering extract from the project copy of the standard
(§14–§17). Working basis [B-STD] for the controller protocol layer.

## §14.1 Controller IEEE 488.1 subset requirements (Table 14-1)

SH1, AH1; Talker T5–T8/TE5–TE8 (T5/T6 if pass-control); Listener
L3/L4/LE3/LE4; SR1 (if pass-control) else SR0; RL0/RL1; PP0/PP1;
DC0/DC1; DT0/DT1; Controller: C1+C2+C3+C4 mandatory, one of C5..C27.
E2 electrical: open collector on SRQ/NRFD/NDAC, three-state on
DAV/EOI/ATN/REN/IFC/DIO.

## §16.2 Required control sequences (exact driver algorithms)

- **SEND COMMAND (16.2.1)**: ATN TRUE, EOI FALSE; send each 7-bit command
  with DIO8 FALSE. ATN/EOI held until another sequence changes them.
- **SEND SETUP (16.2.2)**: ATN TRUE; send controller's TALK address; UNL;
  each listen address.
- **SEND DATA BYTES (16.2.3)**: ATN FALSE; data bytes; terminator (END is
  sent WITH the last byte; null terminator allowed for piecemeal sends).
- **SEND (16.2.4)** = SEND SETUP + SEND DATA BYTES.
- **RECEIVE SETUP (16.2.5)**: ATN TRUE; UNL; controller's LISTEN address;
  talk address.
- **RECEIVE RESPONSE MESSAGE (16.2.6)**: ATN FALSE; handshake until the
  stop condition. Stop on END required; on NL, comma, semicolon
  required; byte count / DAB match optional. NRFD/NDAC held TRUE after.
- **RECEIVE (16.2.7)** = RECEIVE SETUP + RECEIVE RESPONSE MESSAGE.
- **SEND IFC (16.2.8)**: system controller only; pulse IFC TRUE for more
  than 100 µs (T8); no other sequence until IFC FALSE.
- **DEVICE CLEAR (16.2.9)**: selected: SEND SETUP + SDC. All: ATN TRUE +
  DCL. ATN remains TRUE, EOI FALSE.
- **ENABLE LOCAL CONTROLS (16.2.10)**: selected: SEND SETUP + GTL.
  All: REN FALSE.
- **ENABLE REMOTE (16.2.11)**: selected: if REN FALSE, wait >= 100 us;
  REN TRUE; SEND SETUP. All: REN TRUE (same wait).
  (ERRATUM 31.10.2026: this line previously read "100 ms" -- an OCR
  artifact (mu rendered as m). Corrected to 100 us per the IEEE text
  and the NI NAT4882 Programmer Reference (auxiliary sre: REN must be
  FALSE >= 100 us before re-assertion).)
- **SET RWLS (16.2.12)**: ENABLE REMOTE with addresses; LLO. ATN TRUE.
- **SEND LLO (16.2.13)**: ATN TRUE; LLO.
- **READ STATUS BYTE (16.2.18)**: ATN TRUE, EOI FALSE; UNL; controller's
  listen address; SPE; talk address; ATN FALSE; handshake one DAB (STB &
  RQS), store it, ignore EOI; ATN TRUE; SPD; UNT.
- **TRIGGER (16.2.19)**: selected: SEND SETUP + GET. All addressed:
  ATN TRUE + GET.

Optional: PASS CONTROL (16.14), PERFORM/CONFIGURE/UNCONFIGURE PARALLEL
POLL (16.15–16.17).

## §17.1 RESET protocol (mandatory)

Three levels, in order:
1. Bus initialization: ENABLE REMOTE (no addresses) then SEND IFC.
2. Message exchange initialization: DEVICE CLEAR without addresses (DCL).
3. Device initialization: SEND "*RST<PMT>" to each supplied address.

## §17.3 ALLSPOLL protocol (mandatory)

Reads the STB of every SR1 device in a supplied address list:

    ATN TRUE; UNL; LAG(controller); SPE
    WHILE address list not exhausted
      TAG(next address); ATN FALSE; handshake DAB (STB & RQS); store
      ATN TRUE
    SPD; UNT

Notes: only the first STB per device is read; SRQ state at invocation is
irrelevant; polling addresses without SR1 can hang the handshake (the
address list must contain only SR1-capable devices).

## §17.2 FINDRQS (optional)

Same skeleton, but repeat TAG/read until RQS bit TRUE; return the
requesting address and its STB, or an error after the list is exhausted.
Address ordering is the polling priority.

## Status in this core

- SEND/SEND SETUP/SEND DATA BYTES/RECEIVE/RECEIVE SETUP: implemented by
  the line sequencer prologues; the SEND SETUP order (MTA(controller)
  first, then UNL, then listeners) and the READ STATUS BYTE order
  (UNL, MLA(controller), SPE, MTA) are aligned with the standard.
- SEND COMMAND: gpib_send_commands (arbitrary ATN command list).
- SEND IFC / ENABLE REMOTE / REN control: controller FSM groups (SIAS/
  SRAS with T8) — [P] host-model wiring.
- DEVICE CLEAR / TRIGGER / LLO / GTL / SDC / DCL: SEND COMMAND with the
  corresponding byte patterns (16.2.9/16.2.19/16.2.13).
- READ STATUS BYTE: gpib_serial_poll (SPE, MTA, STB, SPD, UNT).
- ALLSPOLL: sequential gpib_serial_poll per address ([P]: the
  single-SPE armed pass comes with the FINDRQS work).
