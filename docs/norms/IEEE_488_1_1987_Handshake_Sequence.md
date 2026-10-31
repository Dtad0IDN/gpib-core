# IEEE 488.1-1987 — Three-Wire Handshake Byte Sequence (Annex B, Fig B.1)

Status: **[B-STD]** working basis; sequential description of Fig B.1 as
transcribed from the project copy. The `t-2..t14` markers are EVENT ORDER
references of the figure, not the normative Table 39 values; the normative
minimum times (T1 settle, T9/T10 delays) apply inside the marked windows.

## Source (SH) actions

1. Initialize DAV high (H) — data not valid.
2. (Acceptors initialize NRFD low, NDAC low.)
3. Check error condition (NRFD and NDAC both high = no acceptors present),
   then set the data byte on DIO.
4. Delay to let data settle on DIO (window: T1, Table 39).
5. (Acceptors ready: NRFD goes high when the LAST acceptor is ready.)
6. On sensing NRFD high: set DAV low — data settled and valid.
10. On sensing NDAC high (all accepted): set DAV high — data now invalid.
11. Change data on DIO.
12. Delay to allow settling (next cycle's T1 window).
16. On sensing NRFD high again: DAV low (next byte).
20. On NDAC high: DAV high [as 10].
21. Remove the data byte from DIO after DAV high.

## Acceptor (AH) actions

2. Initialize NRFD low (not ready), NDAC low (not accepted).
5. Ready for the byte: set NRFD high (line goes high only when the LAST
   acceptor releases — wired-OR).
7. On DAV low: set NRFD low (hold-off the next byte), then accept the data
   (each acceptor at its own rate).
8. Accepted: set NDAC high (line stays low while ANY acceptor still drives
   it low).
9. Last acceptor sets NDAC high — all have accepted, NDAC line goes high.
13. On sensing DAV high: set NDAC low in preparation for the next cycle.
14./15. Ready again: NRFD high (first / last acceptor; line high at last).
17.–19. Repeat of 7–9 for the next byte.
22. On DAV high: NDAC low for the next cycle.
23. All three handshake lines back at initialized states [as 1, 2].

## Mapping to the core executor (host model -> future line engine)

- Steps 3–4  : SH `SDYS`  (data on DIO, settle window, T1 policy).
- Step 6     : SH `SDYS -> STRS` transition gate = RFD (NRFD high), DAV low.
- Steps 7–9  : AH `ACDS` accept + `DAC` wired-OR collection (NDAC high only
               from the LAST acceptor) — the aggregation our single result
               accounting models.
- Step 10    : SH `STRS -> SWNS` on DAC; DAV high.
- Steps 13–15: AH re-arm (NDAC low, then NRFD high) — the backpressure
               pause point of the RX path.
- Step 3 error check (NRFD && NDAC both high) — bus-without-acceptors
               detection candidate for the port/adapter layer.

Notes: lower-case markers here belong to the figure; do not confuse them
with Table 39 identifiers. Wired-OR semantics of NRFD/NDAC (first/last
acceptor behavior) is what tests/vbus models (VB-01).
