; neg_p6_pause_replay_budget.gcode - documented deviation (P6, U11/U02).
; A mid-window PAUSE/RESUME replay baked into the file: PAUSE injects its own
; M2800+M400 cut and V-2 maintenance retract; RESUME injects the U-only
; re-prime compensation. Consequences the validator reports, and which are the
; documented reason the exported file must stay pause-invariant:
;   R10 - the injected re-prime is a U-only move after priming (fiber without
;         matrix), which the contract forbids inside a window;
;   R07 - the compensation feed makes sum(U) exceed the L+1 budget bound.
; At runtime the firmware compensation is out-of-band (macro-injected) and
; harmless; it is the L budget advisory (U02) that keeps sum(U) meaningful, and
; any tool that folds macro traffic back into the file (replay, naive
; post-processing) must be rejected. Normal operator pause at a window
; boundary never produces this shape (see good_p6_pause_between_windows.gcode).
# EXPECT: ERROR R07
# EXPECT: ERROR R10
# EXPECT: WARN R01U
G21
G90
M83
T0
M106 P2 S255
; LAYER:1 [0.2]
M1001 L57
G1 F1500 U55 ; Extrude restart
G0 X107.500 Z0.200 F30000
G1 F600 V4 ; Extrude restart
G1 X110.000 V0.04200 U2.00000 P0.021 F300
; --- injected PAUSE trace (print_control.cfg: cut, dock, V-2 retract) ---
M2800
M400
G1 F600 V-2
; --- injected RESUME trace (re-prime by recorded material delta) ---------
G1 F1500 U2.00000
G1 F600 V-1
M1002
