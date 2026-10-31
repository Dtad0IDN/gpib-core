# gpib-core — portable IEEE-488 (GPIB) core

Portable level-3 GPIB core (IEEE-488.1-1987 + IEEE-488.2-1992), C11,
zero heap, single static context. Implementation of
`GPIB_Core_Spec_1.0-draft.2.2.3` + `GPIB_Core_Next_Contracts_v0.2` +
the normative extracts in `docs/norms/`.

**Status: host-level ТЗ scope + the optional layer are functionally
complete; four external re-audits (Rev5-Rev8, 2026-10-31/11-01) triaged
item-by-item and closed with regressions.** Both roles (Controller /
Device) run the real IEEE-488.1 protocol engine on a wired-OR line
model; all mandatory 488.2 control sequences and protocols (§16.2,
§17.1/17.3) plus FINDRQS, the RECEIVE stop conditions, and the optional
features — TE/LE secondary addressing, PP1 parallel poll, FINDLSTN,
REQUESTCTL/PASSCTL — are implemented and verified end-to-end against
simulated instruments and core-to-core. The §2.9 PP extract arrived and
the PP1 FSM was brought to full Fig. 11 conformance (both state groups,
Table 25 output semantics). The audit campaign closed: F01 external-IFC
device reset (+E2E), F02/F17 C-model synchronization (gts/tca/rpp fed
from the sequencer), F03/F16 TCT modeling, F04 device-core PP1
(core-to-core E2E), F05 READ stream sessions (ONE RECEIVE per response,
NRFD backpressure between blocks), F06/F07/F08/F12 contract hardening,
F09/F10 addressed-command qualification, F11 own-address reinit, F13
RECEIVE stop state, F14 strict T8/REN-low timings, F15 steal-observer,
F18 SR round evaluation, F22-F25 module E conformance, the T03
drive-structure UB, T01/T02/T08/T14 test infrastructure, R6-01 REN
determinism (poisoned-memory regression), R6-02/R7-01 stop-state and
SC-line continuity (the single drive composer: REN/IFC persist through
EVERY drive; the off-bus detach releases them exactly once at
STOPPING→DRAINING, and the test ports certify `safe_detached` from the
wire instead of stubbing it), R6-03 mid-session re-addressing refused,
R6-04 composite results moved to instance memory, T8-01/T8-02 honest
port contracts + per-drive intent invariants. The **independent wire
oracle (T04)** — built last — has already found three real core bugs no
FSM-shared test could see (see the reviewer section). Remaining: the
REQUESTCTL composite lifecycle fault-injection, independent
multi-controller takeover tests, level-2 hardware adapters and VT
benches (see "Not implemented (deliberate)" below).

## Build & test (host)

MinGW GCC, no dependencies — builds SEVEN binaries and runs them all:

```
powershell -ExecutionPolicy Bypass -File build.ps1
```

| Binary | Purpose |
|---|---|
| `test_s1` | Controller-role full suite (24 suites, 147 328 checks) |
| `test_device` | Device-role E2E against a simulated controller (425 checks) |
| `matrix_c_full/min`, `matrix_d_full/min` | 4-configuration CI matrix (role × profile), mandatory-subset parity + G-MEM RAM report (2176 / 1064 bytes) |
| `test_bigcfg` | compile-time border configuration (MAX_LISTENERS=11, MAX_CMDS=13) |

CMake variant (when available):
`cmake -S . -B build && cmake --build build && ctest`.

The test binary accepts a suite-count argument for bisecting:
`build\test_s1.exe 15` runs the first 15 suites only.

## Layout

| Path | Contents |
|---|---|
| `include/gpib/gpib_types.h` | status codes, lifecycle, semantic address, handles, stop conditions, end causes |
| `include/gpib/gpib_cfg.h` | compile-time profile knobs (role, ledger limits, budgets, line timings) |
| `include/gpib/gpib.h` | public semantic API (Doxygen EN; [P] names, ABI [O]) |
| `include/gpib/gpib_port.h` | level-2 port contract: byte-pump slice + line slice + null port |
| `include/gpib/gpib_storage.h` | typed compile-time storage header ([P] configuration.md §7) |
| `include/gpib/gpib_proto.h` | pure IEEE-488.1 FSM modules (SH/AH/T/L/SR/RL/DC/DT/C/addressing/decoder) |
| `include/gpib/gpib_4882.h` | module E: 488.2 status model + common commands |
| `src/core.c` | lifecycle, process, emergency, recover, snapshots, DF events, IFC/REN requests |
| `src/submit.c` | submit/cancel + every operation constructor (write/read/write_many/spoll/allspoll/findrqs/reset/TCT/CMDS/device supply/sink/ppoll_configure/ppoll_unconfigure/parallel_poll/findlstn/request_control/requestctl_pass) |
| `src/lineexec.c` | line-level sequencer: CMD/DATA_TX/DATA_RX/EPILOGUE/POLL_ADDR/PPOLL_IDY/FINDLSTN_PROBE/DEV_WAIT phases, the single drive composer (SC lines persist through every drive, R7-01), the off-bus electrical detach at STOPPING→DRAINING (T8-01), external-IFC reset, READ stream sessions, DF events |
| `src/executor.c` | mailbox host model: action slot, stop latch, accounting |
| `src/ledger.c` `src/results.c` | block ledger, transaction terminal records, views |
| `src/proto.c` | all IEEE-488.1 FSMs (normative-extract traceable) |
| `src/mod4882.c` | 488.2 registers, STB computation, common-command parser |
| `src/epoch.c` `src/trace.c` `src/port_null.c` | identity manager, binary trace ring, null port |
| `tests/` | 24 suites: lifecycle/ledger/mailbox/interleaving/litmus, protocol vectors, line simulation, E2E against simulated instruments (two per bus), dual-core TCT, device-role E2E, 488.2 unit + slice, optional features (TE/LE, PP1, FINDLSTN, REQUESTCTL), S2 oracle, **independent wire oracle O1-O14 (T04: a partner sharing ZERO protocol code with the core — own handshake, own command table, waveform trace, per-DRIVE interception, wire conflict invariants, honest `safe_detached`)**; plus the 4-config matrix binaries |
| `docs/norms/` | verified engineering extracts: 488.1 timing, Annex B sequence, SH/AH+commands, T/L/C/SR, RL/DC/DT, 488.2 controller protocols |

## Architecture (what runs where)

- **Byte-pump ports** (null/mock): the original host model — actions move
  bytes through `transfer()`; used by the mailbox/interleaving suites.
- **Line ports** (`read_lines`/`drive`/`drive_data`): actions run the REAL
  protocol — the sequencer drives SH/AH FSMs over wired-OR lines
  (NRFD/NDAC/DAV/ATN/EOI/SRQ/IFC/REN/DIO); used by all E2E suites and
  the device-role binary.
- **Both roles, one codebase**: compile-time role selection
  (`GPIB_CFG_ROLE`); the Controller initiates (submit family), the Device
  waits for external addressing (supply/sink/service channel).

## Implemented semantics (traceability)

### Core (ТЗ / contracts)

- Lifecycle `UNINITIALIZED→INITIALIZED→RUNNING→STOPPING→DRAINING`,
  default-deny, explicit `start()`, `BUSY_RESOURCES` busy reasons
  (REQ-071/076, LIF-01…08).
- Block ledger: reserved-before-accept, ownership back only on ACK;
  queued shortcut `NOT_STARTED` without fake QUIESCED (FIX-02/REQ-064).
- Independent transaction terminal record after all blocks terminal,
  block ACKs not required (REQ-033/077, TXN-01/02); block handles survive
  txn slot reuse (FIX-01/REQ-070, TXN-03, LED-06).
- Epoch manager outside context: never-reused identities, exhaustion
  refuses init (REQ-072, TXN-05).
- Semantic address REQ-078/079: `secondary_present` flag, `0` is a valid
  secondary, rejection before side effects, atomic write_many (ADDR-01…06).
- Mailbox host model (MAILBOX v0.2): monotonic action generations,
  immutable descriptors, stop latch, stale-fact drop+count; P1 YIELD
  (R12) with command-byte capture; SRQ sideband; shutdown independent of
  ACKs; stuck-port quiescence; HISTORY_LOST quarantine (MBX-01…14,
  QSC-01/02).
- Emergency stop (idempotent, electrical only), fault snapshot,
  recover guard, RX views (REQ-065, FIX-03), streaming backpressure
  (REQ-026), 10 MB stress with byte oracles (REQ-031), structural
  deadlines (REQ-025), binary trace ring (REQ-055/058), write_many
  single-action semantics (R5), device service channel (REQ-016).
- Verification discipline: exhaustive interleaving enumeration + seeded
  storms with one invariant checker (G-MBX), memory-order litmus under
  real threads (MAILBOX §9), 4-config CI matrix with RAM report (G-MEM).

### IEEE 488.1-1987 protocol engine (all functions)

- SH (SIDS…SIWS, T1 gating, optional `!DAC` empty-bus guard, in-flight
  uncertainty), AH (AIDS…AWNS, T3 gating, tcs blocking, rdy
  backpressure) — §6 vectors + LS-01…05 line integration.
- ATN command decoder: fixed commands, MLA/MTA/SECONDARY groups,
  UNL/UNT ≠ address 31, DIO8 masking, 0x1F undefined in the 1987 baseline.
- Talker T (serial-poll pair, T5-T8 subsets), Listener L (ltn/lun,
  L3/L4 unaddress-on-own-MTA), SR (no APRS→SRQS resurrection, SPAS STB
  with RQS), RL (LOCS/REMS/LWLS/RWLS, rtl shadowing, GTL/REN-drop
  semantics), DC (DCL/SDC split, DC1/DC2), DT (one-shot DTAS) — all
  directed vectors from the normative extracts.
- Controller C: five independent groups (main chain CIDS…CPPS with
  T6/T7/T9/T10 holds and !SDYS/!STRS guards; SRQ monitor; SC authority;
  IFC driver with T8; REN driver with T8) — §7.5 vectors + core-to-core
  TCT: two real instances on one shared bus, sender CACS→CTRS→CIDS,
  receiver CIDS→CADS→CACS and holds ATN.
- System controller lines (C2/C3): `gpib_interface_clear` (T8 pulse,
  resets instrument interface functions within t4), `gpib_remote_enable`
  (persistent SRAS; the low-time window is anchored to the actual wire
  release edge, R6-01/F14). SC lines merge INTO every sequencer drive
  through the single composer (R7-01: an exchange-scoped release can
  never drop REN/IFC electrically); the ONLY all-outputs release is the
  off-bus detach at STOPPING→DRAINING, certified by an honest
  `safe_detached` (T8-01).

### IEEE 488.2-1992 module E + controller protocols

- Status model: ESR/ESE/SRE (PON/URQ/CME/EXE/DDE/QYE/RQC/OPC bits),
  STB = ESB|MAV without B6, MSS→rsv, response queue; common commands
  `*CLS *ESE *ESE? *ESR? *IDN? *OPC *RST *SRE *SRE? *STB? *TST? *WAI`
  with query/command form separation and CME on errors. Vertical slice:
  controller programs `*ESE 1;*OPC` → SRQ → serial poll reads ESB|RQS →
  serviced (DV-04).
- Control sequences §16.2: SEND SETUP / RECEIVE SETUP / READ STATUS
  BYTE orders byte-exact per the standard; SEND COMMAND
  (`gpib_send_commands`, arbitrary ATN bytes) — building block for
  DEVICE CLEAR / TRIGGER / SEND LLO (E2E-05); SEND IFC (E2E-06).
- RECEIVE stop conditions §16.2.6: EOI (default), NL, comma, semicolon —
  stop byte is part of the data; `GPIB_END_CHAR` closes the READ
  transaction (E2E-07, four modes on one response).
- Protocols §17: **RESET** (`gpib_system_reset`: REN+IFC+DCL+per-address
  `*RST<NL>^END` with full SEND SETUP each — SP-05), **ALLSPOLL**
  (one armed-SPE pass, per-address MTA+STB — SP-03, two instruments on
  one bus), **FINDRQS** (early exit on RQS with caller-found index —
  SP-04). All mandatory protocols implemented.
- **TE/LE secondary addressing** (extract 3): TPIS/TPAS and LPIS/LPAS
  pairs in the Talker/Listener FSMs with MSA(own) completion, PCG-only
  disarm (secondary commands never disarm the pair), OTA/UNT/UNL exits.
  **End-to-end**: secondary addresses are accepted by submit (0..31),
  SAD bytes are sent in the WRITE/READ/WRITE_MANY prologues, the device
  core arms its TE/LE pairs from its own secondary, and the simulated
  instrument receives data only when the full MLA+SAD sequence arrives
  (E2E-08).
- **PP1 parallel poll** (verified against the §2.9 extract, Fig. 11 +
  Tables 24–27): BOTH state groups — response (PPIS/PPSS/PPAS) and
  configure (PUCS/PACS). PPE/PPD are honored only in PACS (PUCS shall
  ignore them, 2.9.3.4); PPC∧LADS opens the window, any other PCG byte
  closes it; PPU unconfigures unaddressed; PPSS→PPAS on IDY is
  UNCONDITIONAL in ist — ist==S gates only the PPR output (Table 25:
  true iff ist==S, else passive false); DCL has no PP arc. Unit vectors
  (15 scenarios) + E2E-09 including the ist≠S passive-false wire check.
- **Arbitrary DAB stop match** (§16.2.6.6): `stop_cond=GPIB_STOP_DAB`
  with `stop_dab` byte — the fifth stop condition.
- **PP controller sequences** (§16.15–16.17): `gpib_parallel_poll`
  (assert ATN+EOI, settle T6, read PPR from the DIO wired-OR),
  `gpib_ppoll_configure` (send PPC+PPE to a device),
  `gpib_ppoll_unconfigure` (send PPU) — E2E-09.
- **FINDLSTN** (§17.6): `gpib_findlstn(addrs, n, results)` — per
  candidate UNL + MLA(addr) with ATN, then ONE probe DAB byte at ATN
  released: a listener in LADS holds NDAC and handshakes (present);
  nobody addressed → every AH falls back to AIDS (passive lines) and
  the SH SDYS stall (Annex B empty-bus check) times out (absent).
  Epilogue UNL leaves no listener addressed (E2E-10).
- **REQUESTCTL** (§17.5): `gpib_request_control()` (requesting side:
  assert rsv), `gpib_requestctl_pass(addrs, n, &found)` (CIC side:
  blocking FINDRQS composite, then an asynchronous TCT whose handles
  flow to the caller — documented contract). PASSCTL is the existing
  `gpib_pass_control` (§17.4). E2E-11: RQS → found → MTA+TCT on the
  wire → NOT_CIC for further controller submits.

## For the independent reviewer

**Build & run everything** (Windows/MinGW, no CMake needed):
`powershell -ExecutionPolicy Bypass -File build.ps1`
Seven binaries (s1, device, 4x matrix, bigcfg), ~147k checks total,
exit code 0 on green. Linux/CMake:
`cmake -B build && cmake --build build && ctest` (GCC/Clang, pthreads;
CTest runs 7 targets: s1, device_smoke, 4x matrix, bigcfg).

**Verification layers** (each independent of the previous):
1. Directed FSM vectors transcribed from the normative extracts
   (`docs/norms/`) — every state/arc of SH/AH/T/TE/L/LE/SR/RL/DC/DT/C/PP.
2. The S2 oracle: an exhaustive-input cross-check of SH/AH transition AND
   output tables against a second, table-based transcription (not the
   production switch/case).
3. E2E against simulated instruments (mock_lines, FSM-shared — kept as
   integration tests).
4. Core-to-core: two real instances exchanging TCT on one wire.
5. **The T04 independent wire oracle** (`tests/oracle_peer.c`): a partner
   sharing ZERO protocol code with the core (own handshake, own command
   table, waveform trace, online wire invariants, per-DRIVE
   interception). It has found THREE real core bugs that no FSM-shared
   test could see:
   - EOI asserted with the status byte terminated the serial poll before
     its SPD/UNT epilogue (488.2 16.2.6: the controller must ignore EOI
     on the STB);
   - a two-talker DIO window: the oracle (source) legitimately held its
     DIO drivers through the controller's next command byte;
   - the lost accept pulse: the terminal-READ early return skipped the
     AH's final drive, so the NDAC release of the LAST byte never
     reached the wire — on hardware the talker hangs in SWNS holding
     its data lines forever.
   O8 asserts REN/IFC persistence on EVERY single drive() call (the
   Rev7 probe technique — one-round glitches invisible to snapshot
   traces); O12 verifies the electrical detach lifecycle (shutdown with
   REN live releases at the DRAINING boundary; a port that refuses to
   certify blocks deinit with BUSY_PORT_TEARDOWN_PENDING).

**Traceability:** every audit item is either fixed with a regression
test or explicitly deferred below — F01-F26 (IEEE review, 2026-10-31),
T01-T16 (test audit), R6-01..R6-04 (Rev6), R7-01 (Rev7), T8-01/T8-02
(Rev8), and the Rev9 recommendations (final-accept-pulse sequence test,
REQUESTCTL stuck-port fault injection, exact teardown busy reason with
port recovery, silent-on-green diagnostics, a dedicated oracle CTest
target, and the Rev10 findings: --suite isolation for that target, a
per-DRIVE time-stamped journal catching intra-round output glitches
(sensitivity proven by re-running the auditor's own mutation), and
read_lines-failure injection). The PP1 function is verified against the §2.9 extract; RL/DC/DT
against §5; SH/AH against the SH/AH extract; controller protocols
against the 488.2 extract. The REN low-time anchoring for ports with
pre-existing physical history is a documented port contract
(`gpib_cfg.h`, T8_STRICT note).

**Known deferred (deliberate, profile-level):** EOS terminator policy
(OPEN-10), zero-length block policy (OPEN-23), device event
classification (OPEN-01/02), response queue depth 1 ([P]), ABI freeze
(OPEN-26), primary-only address lists in the bulk operations (F19), the
minimal module-E argument parser (u8 only, F26), hardware timings
(level-2 ports re-anchor T1/T3/T6/T8 to real transceiver edges).

## Not implemented (deliberate)

- **REQUESTCTL composite lifecycle** (R6-04 residue): the FINDRQS
  results live in instance memory (no stack-lifetime risk) and the
  bounded drain cancels+drains on timeout, but the full formalization
  (fault-injected stuck port, repeated-composite gating, ACK result
  checking) is pending its own test pass.
- **Independent multi-controller takeover tests**: the
  sequencer-master/C-FSM-model split is deliberate (single-controller
  profile); tcs/tca takeover chains and dual-controller fault injection
  await the hardware-adjacent stage.
- **PP IFC-reset extract pending**: the mock resets the PP groups on
  IFC (both groups to pon states); §2.9 itself specifies no IFC arc —
  that behavior belongs to the IFC section, whose extract is still
  pending ([P]). The §2.9 verification is otherwise complete.
- Spec-level [O] deferrals: EOS terminator policy (OPEN-10,
  `GPIB_END_EOS` reserved), zero-length block policy (OPEN-23), device
  event classification (OPEN-01/02), response queue depth 1 ([P]),
  ABI/name freeze (OPEN-26).
- Small API conveniences: the write/read convenience wrappers do not
  expose `deadline_us` (use raw `gpib_submit`); a fast
  `GPIB_TERM_NO_LISTENER` fail (vs deadline timeout) is a known
  candidate for VISA-facing adapters.
- Per-suite CTest entries (today the 24 suites share one ctest target
  for bisection through the suite-count argument); a `gcovr`/CI badge
  pipeline.
- Level-2 hardware adapters (STM32H743 / K1986VE1FI / ESP32 GPIO+IRQ
  ports; an LPT bit-bang adapter design exists in the session notes)
  and VT1/VT2/VT3 benches - the next stage; the line-port contract is
  the integration surface.
- Doxygen HTML generation, C ABI freeze (all names/values remain [O]).

## Normative basis

`docs/norms/` — engineering extracts, verified during implementation:

| Document | Covers |
|---|---|
| `IEEE_488_1_1987_Timing.md` | Table 39: T1-T10, t2/t4/t5 |
| `IEEE_488_1_1987_Handshake_Sequence.md` | Annex B byte cycle (23 steps) |
| `IEEE_488_1_1987_SH_AH_Commands.md` | SH/AH transitions + ATN command bytes |
| `IEEE_488_1_1987_T_L_C_SR.md` | T/L/TE/LE/SR/C(5 groups) + subsets |
| `IEEE_488_1_1987_RL_DC_DT.md` | RL/DC/DT + directed vectors |
| `IEEE_488_1_1987_PP.md` | §2.9 PP function: Fig. 11 (both state groups), Tables 24–27, subset rules |
| `IEEE_488_2_1992_Controller_Protocols.md` | §14 subsets, §16.2 sequences, §17 protocols |

Known caveat (recorded in the extract): the source PDF is the 1978
printing; table/figure numbering matches the 1987 edition. Page
references carry the +7 offset note.
