# GPIB CORE — PRIORITY 1: SH/AH TRANSITIONS AND ATN COMMAND BYTES

Status: reference for implementing and testing a portable C11 GPIB Core.
Baseline: ANSI/IEEE Std 488.1-1987, §§ 2.3, 2.4, 2.13; Tables 2, 3, 5, 6, 38, 39.
Date: 2026-10-23

This is an engineering interpretation, not a verbatim reproduction of the standard.
Optional transitions are labelled OPTIONAL. Do not assume that this document
alone establishes compliance with all IEEE-488 revisions or timing requirements.

## 1. CONVENTIONS AND SHARED SIGNALS

Boolean ATN=1, DAV=1 and similar expressions mean the IEEE-488 logical
message is TRUE. For these active-low interface lines, TRUE means the wire
is physically LOW. FALSE is the electrically deasserted (HIGH) level.

IEEE logical RFD=1 corresponds to physical NRFD HIGH (all listeners ready).
IEEE logical DAC=1 corresponds to physical NDAC HIGH (all listeners accepted).
RFD=0 or DAC=0 means at least one active acceptor holds that wire LOW.

    pon   = power-on
    nba   = local "new byte available" message from the source's data producer
    rdy   = local "ready for next message" from the acceptor's consuming side
    tcs   = take control synchronously, from the controller FSM
    TACS  = talker active state
    SPAS  = serial poll active state
    CACS  = controller active state
    CTRS  = controller transfer state
    LADS  = listener addressed state
    LACS  = listener active state
    T1    = minimum byte-settling interval (normally >= 2 us; reduced
            alternatives depend on driver and bus conditions and must NOT be
            assumed)
    t2    = max response to ATN, <= 200 ns per the cited 1987 standard
    T3    = positive implementation-dependent minimum interface-message
            accept time

Derived predicates for an SH implementation:

    SRC_ACTIVE = TACS || SPAS || CACS
    SH_LOSES_ROLE = (ATN && !CACS && !CTRS) || (!ATN && !TACS && !SPAS)

Derived predicate for AH role departure:

    AH_NOT_ADDRESSED = !ATN && !LADS && !LACS

NOTES:
* Command/data interpretation depends on ATN and active role, not solely on DIO.
* DIO/EOI must be held stable in SDYS and STRS.
* The port must map logical signal truth to actual hardware with explicit
  inversion and safe output-enable handling.

## 2. SOURCE HANDSHAKE (SH) FSM

States and output DAV:

    SIDS: source idle                  DAV=FALSE (passive); byte may be changed
    SGNS: source generates next byte   DAV=FALSE; byte may be changed
    SDYS: byte setup and ready wait    DAV=FALSE; DIO/EOI held stable
    STRS: byte transfer                DAV=TRUE; DIO/EOI held stable
    SWNS: wait for next cycle          DAV=TRUE OR FALSE; byte can be changed
    SIWS: idle-wait while interrupted  DAV=FALSE (passive); byte can be changed

Transition table (all absent transitions retain current state):

    FROM  -> TO    GUARD                                     TIME / REMARK
    -----    ----  ----------------------------------------  ----------------------
    INIT  -> SIDS  pon                                       initialization
    SIDS  -> SGNS  SRC_ACTIVE                                role enables source
    SGNS  -> SDYS  nba                                       new byte presented
    SGNS  -> SIDS  SH_LOSES_ROLE                             within t2
    SDYS  -> STRS  RFD && T1_elapsed [&& !DAC]              !DAC check OPTIONAL
    SDYS  -> SIDS  SH_LOSES_ROLE                             within t2
    STRS  -> SWNS  DAC                                       accepted by ALL AHs
    STRS  -> SIWS  SH_LOSES_ROLE                             within t2, interruption
    SWNS  -> SGNS  !nba                                      new cycle requested
    SWNS  -> SIWS  SH_LOSES_ROLE                             within t2
    SIWS  -> SIDS  !nba                                      idle if no new byte
    SIWS  -> SWNS  SRC_ACTIVE                                transmission resumes

IMPORTANT IMPLEMENTATION DETAILS:
* nba may become TRUE only in SIDS or SGNS (standard §2.3.5); it may be
  deasserted in other states. Do not substitute an arbitrary queue-nonempty
  flag without modeling the handshake of the local byte producer.
* In SWNS the standard permits either value of DAV. Choose a deterministic
  policy for the implementation and prove correct cycling and interruption.
* A byte is not considered definitely accepted merely because DAV was
  asserted; SH observes logical DAC=TRUE (all acceptors released NDAC).
* For loss-of-role during STRS, the source can be asynchronously interrupted;
  completion status of the in-flight byte must be tracked separately.
* QUIESCED and ACK of application zero-copy buffers are separate local
  resource protocols, not SH interface states.

## 3. ACCEPTOR HANDSHAKE (AH) FSM

The AH function always participates in receiving interface commands while
ATN=TRUE, including when it has not been addressed as a data listener.

States and output RFD/DAC (IEEE logical messages):

    AIDS: idle                         RFD=TRUE(passive), DAC=TRUE(passive)
    ANRS: not ready                    RFD=FALSE,         DAC=FALSE
    ACRS: ready to accept              RFD=TRUE(passive), DAC=FALSE
    ACDS: data presented / valid       RFD=FALSE,         DAC=FALSE
    AWNS: received, await DAV release  RFD=FALSE,         DAC=TRUE(passive)

Physical equivalents:

    logical RFD FALSE -> assert NRFD LOW; RFD TRUE -> release NRFD HIGH.
    logical DAC FALSE -> assert NDAC LOW; DAC TRUE -> release NDAC HIGH.

Transition table:

    FROM  -> TO    GUARD                                    TIME / REMARK
    -----    ----  ---------------------------------------  -----------------------
    INIT  -> AIDS  pon                                      initialization
    AIDS  -> ANRS  ATN || LACS || LADS                      within t2
    ANRS  -> ACRS  !tcs && ((ATN && !DAV) || rdy)          !DAV qualifier OPTIONAL
    ANRS  -> AIDS  AH_NOT_ADDRESSED                         as role goes inactive
    ANRS  -> AWNS  DAV                                      OPTIONAL (unusual)
    ACRS  -> ACDS  DAV                                      valid byte available
    ACRS  -> AIDS  AH_NOT_ADDRESSED                         as role goes inactive
    ACRS  -> ANRS  !ATN && !rdy                             within t2
    ACDS  -> AWNS  (ATN && T3_elapsed) || (!ATN && !rdy)   accepted message
    ACDS  -> AIDS  AH_NOT_ADDRESSED                         as role goes inactive
    ACDS  -> ACRS  !DAV                                     OPTIONAL async take-over
    AWNS  -> ANRS  !DAV                                     ready for next cycle
    AWNS  -> AIDS  AH_NOT_ADDRESSED                         as role goes inactive

WARNINGS:
* Byte-valid observation belongs to ACDS (DAV asserted). Record DIO/EOI/ATN
  consistently before publishing the acceptance of a byte.
* The source must NOT see NDAC released before the receiver has securely
  captured/staged the byte. With zero-copy, reserve capacity or apply
  backpressure.
* In data mode (ATN=FALSE), the acceptor must lower rdy at the appropriate
  point to transition ACDS -> AWNS; holding rdy=TRUE indefinitely can stall
  completion even though DAV has been received.
* While in ACRS, rdy cannot spontaneously become FALSE; on the standard
  diagram ACRS -> ANRS is tied to ATN becoming FALSE.
* For ATN=TRUE, command-handshake acceptance is distinct from application RX.
* The optional ANRS -> ACRS no-DAV qualification exists for compatibility
  with synchronous take-control conditions; handle tcs as its own controller
  signal.
* Refusal to act on a command does not imply refusal to handshake its byte.

## 4. ATN=TRUE: INTERFACE COMMAND DECODING

Fixed command bytes (normal software representation; DIO8 sent as 0):

    MNEMONIC   HEX   DEC   CLASS       MEANING
    --------   ----  ----  ----------  ------------------------------------
    GTL        0x01     1  addressed   Go To Local
    SDC        0x04     4  addressed   Selected Device Clear
    PPC        0x05     5  addressed   Parallel Poll Configure
    GET        0x08     8  addressed   Group Execute Trigger
    TCT        0x09     9  addressed   Take Control
    LLO        0x11    17  universal   Local Lockout
    DCL        0x14    20  universal   Device Clear
    PPU        0x15    21  universal   Parallel Poll Unconfigure
    SPE        0x18    24  universal   Serial Poll Enable
    SPD        0x19    25  universal   Serial Poll Disable
    UNL        0x3F    63  address     Unlisten (not MLA31)
    UNT        0x5F    95  address     Untalk (not MTA31)

Edition note: IEEE 488.1-2003 adds the 0x1F CFE command (HS488). It is
not classified as a fixed command in this 1987-baseline decoder; handling
should be deliberately enabled only by a documented standard/profile choice.

Address/secondary byte groups, exact seven-bit forms:

    01LLLLL    0x20 .. 0x3F  Listener Address Group (LAG)
    10TTTTT    0x40 .. 0x5F  Talker Address Group (TAG)
    11SSSSS    0x60 .. 0x7F  Secondary Command Group (SCG)

Corresponding eight-bit byte patterns (most significant bit first):

    001LLLLL   010TTTTT   011SSSSS

    MLA(n)  = 0x20 | n, n = 0..30; n=31 becomes UNL
    MTA(n)  = 0x40 | n, n = 0..30; n=31 becomes UNT
    MSA(n)  = 0x60 | n, n = 0..31; semantics depend on addressing context

Parallel poll subcommands live inside SCG:

    PPE = 0x60 .. 0x6F (bit pattern 110SPPP, seven-bit)
    PPD = 0x70 .. 0x7F (bit pattern 111DDDD, seven-bit)
    (actual interpretation depends on PPC sequence and the PP function)

Decoder dispatch recommendation:

    if ATN is false -> DATA, not any of these interface commands.
    when ATN is true and AH accepted a byte:
      c = byte & 0x7F;              // DIO8 is don't-care for interface coding
      if c == 0x3F -> UNL
      else if c == 0x5F -> UNT
      else if 0x20 <= c <= 0x3E -> MLA(c & 0x1F)
      else if 0x40 <= c <= 0x5E -> MTA(c & 0x1F)
      else if 0x60 <= c <= 0x7F -> SECONDARY(c & 0x1F)
      else if fixed recognized opcode -> that command
      else -> UNDEFINED_OR_UNSUPPORTED (still complete handshake)

For commands the seventh bit position maps to DIO7 and the high bit DIO8
normally does not participate in decoding. Send DIO8 deasserted (zero in
logical command-byte representation), but do not conflate logical byte bits
with the physical active-low voltage level or any transceiver inversion.

## 5. NON-BLOCKING CORE INTEGRATION

* bus_executor is the single logical owner of line outputs; SH/AH provide
  requested line states, and platform GPIO/transceiver code applies them.
* Source byte owner keeps DIO/EOI immutable while in SDYS or STRS.
* AH captures an incoming byte only in ACDS; decode as command if ATN is
  TRUE, or deliver to RX data path if in LACS and ATN is FALSE.
* Handshake acceptance, operation-result ACK, and reclaiming zero-copy
  memory are three different milestones. Do not release application memory
  early.
* Preserve interruptibility by IFC, cancel and independent handshake
  deadlines. IFC response belongs to integrated role/controller handling;
  do not add an undocumented IFC arrow to isolated SH/AH standard diagrams.
* Multi-listener testing must model NRFD and NDAC as wired-AND active-low
  physical lines: every active receiver must release before RFD/DAC is TRUE.
* Prioritize standardized timing proofs, especially t2 <= 200 ns.
  Software-only GPIO/ISR may fail this on a real MCU under worst-case
  interrupt latency. Correct FSM code does not by itself imply
  electrical/temporal compliance.

## 6. MINIMUM VERIFICATION VECTORS

SH:
  - pon -> SIDS; active role -> SGNS; nba -> SDYS; wait >=T1 and RFD
    -> STRS; wait DAC -> SWNS; remove nba -> SGNS.
  - Slowest of two listeners holds NRFD low: STRS cannot begin.
  - Slowest of two listeners holds NDAC low: byte not confirmed complete.
  - Transition of ATN/revocation of role from every live SH state; check
    deadline t2, line release and ambiguity of in-flight byte.
  - DIO/EOI never change during SDYS or STRS.

AH:
  - pon -> AIDS; assert ATN -> ANRS -> ACRS; assert DAV -> ACDS;
    wait T3 -> AWNS; release DAV -> ANRS.
  - Data input with ATN=FALSE and LACS active: drive rdy with backpressure
    and complete ACDS -> AWNS -> ANRS without losing bytes.
  - Take control synchronously (tcs) prevents ANRS -> ACRS.
  - Invalid/unsupported command is handshaken but not executed.

COMMANDS:
  - 0x01 GTL; 0x04 SDC; 0x05 PPC; 0x08 GET; 0x09 TCT;
    0x11 LLO; 0x14 DCL; 0x15 PPU; 0x18 SPE; 0x19 SPD.
  - 0x20 MLA0; 0x3E MLA30; 0x3F UNL (not MLA31).
  - 0x40 MTA0; 0x5E MTA30; 0x5F UNT (not MTA31).
  - 0x60, 0x6F, 0x70, 0x7F route to SECONDARY with contextual PPE/PPD.
  - Same byte with ATN=FALSE must remain application data, not a command.
  - Check both software seven-bit command and hardware line polarity.
