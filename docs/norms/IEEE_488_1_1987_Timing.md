# IEEE 488.1-1987 — Normative Timing Baseline (Table 39)

Status: **[B-STD]** working basis per STD-001; transcribed from the project
copy of the standard (Table 39). Numeric norm for the timing engine and VT
thresholds. Clause references pending final clause-numbering pass.

Convention (footnote * of the table):

- lower-case `t` — MAXIMUM time allowed to make a state transition;
- upper-case `T` — MINIMUM time a function shall REMAIN in a state before
  exiting.

| ID  | Functions (applies to)        | Description                                        | Value            |
|-----|-------------------------------|----------------------------------------------------|------------------|
| T1  | SH                            | settling time for multiline messages               | >= 2 us (OC)     |
| t2  | SH,AH,T,L,LE,TE               | response to ATN                                    | <= 200 ns        |
| T3  | AH                            | interface message accept time                      | > 0 s (impl.dep) |
| t4  | T,TE,L,LE,C,RL                | response to IFC or REN false                       | < 100 us         |
| t5  | PP                            | response to ATN AND EOI                            | <= 200 ns        |
| T6  | C                             | parallel poll execution time                       | >= 2 us          |
| T7  | C                             | controller delay so current talker sees ATN        | >= 500 ns        |
| T8  | C                             | length of IFC or REN false                         | > 100 us         |
| T9  | C                             | delay for EOI                                      | >= 1.5 us        |
| T10 | C                             | delay for DAV                                      | >= 1.5 us        |

Footnotes:

- T1 with three-state drivers on DIO/DAV/EOI: (1) >= 1100 ns; (2) >= 700 ns
  if ATN is known to be three-stated inside the controller (not recommended);
  (3) >= 500 ns for all bytes after the first following each false transition
  of ATN (first byte per (1) or (2)); (4) >= 350 ns under the conditions of
  clause 5.2 and its warning note.
- T3: time required to ACCEPT (not necessarily respond to) interface
  messages; implementation dependent.
- T9: >= 600 ns with three-state drivers.
- T9 note: delay required for EOI, NDAC, NRFD lines to indicate valid states.

## Engineering consequences for this core

1. **t2 <= 200 ns is not achievable by a software GPIO handshake** (this is
   a hardware-path requirement). Consequence for the compliance matrix:
   software-handshake builds will deviate on t2; measure the actual ATN
   response at VT2 and record the deviation honestly. This matches the
   project plan: VT2 inter-block pause / activation latency metrics.
2. **T1** selects the port transceiver mode policy: open-collector builds
   use >= 2 us; three-state-capable ports may use the reduced values —
   parameter of the port adapter (port_contract `transceiver_mode`, [P]).
3. **T8 > 100 us** drives the System Controller IFC assertion length when
   the IFC path is implemented (Controller role, later increment).
4. **T7 >= 500 ns** applies between ATN assertion and command-byte DAV by
   the controller (the "current talker must see ATN" window).
5. Open-collector T9/T10 (>= 1.5 us) shape the EOI/DAV delay steps of the
   SH byte-cycle in the line-level executor.

This edition defines T1..T10 (no T11+; those belong to later documents).
