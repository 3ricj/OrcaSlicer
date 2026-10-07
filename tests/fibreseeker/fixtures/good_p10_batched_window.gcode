# P10.0/B0 audit-probe fixture: the exact byte output of
# Dialect fixture: two islands in one fiber window
# over two finalized 10 mm square runs on one layer (150/150 and 200/150,
# default FiberEmitParams), wrapped in the T0 composite-head activation prelude
# of machine contract s8.5. One lifecycle window packages the whole batch:
# single M1001 (L = floor(55 + 20) = 75), single restart/prime, runs joined by
# one attached U/V-null travel at the lift feedrate, single cut/handshake/
# retract/close. Probe bytes only (never a shipping lifecycle).
# EXPECT: WARN R01U
# EXPECT: WARN R15
T0
M104 S270 T0
M109 S270 T0
; LAYER:7 [0.50]
M1001 L75
G1 F1200 Z1.70
G1 X150.00 Y150.00 F1200
G1 F1500 U55.000 ; Extrude restart
G1 F1200 Z0.50
G1 F600 V4.000 ; Extrude restart
G1 X160.00 Y150.00 V5.000 U2.500 P2.000 F1200
G1 X160.00 Y160.00 V5.000 U2.500 P2.000 F1200
G1 X150.00 Y160.00 V5.000 U2.500 P2.000 F1200
G1 X150.00 Y150.00 V5.000 U2.500 P2.000 F1200
G1 X200.00 Y150.00 F1200
G1 X210.00 Y150.00 V5.000 U2.500 P2.000 F1200
G1 X210.00 Y160.00 V5.000 U2.500 P2.000 F1200
G1 X200.00 Y160.00 V5.000 U2.500 P2.000 F1200
G1 X200.00 Y150.00 V5.000 U2.500 P2.000 F1200
; Start to cut
M2800
M400
;CUT DISTANCE 54.8
G1 X200.00 Y140.00 F1200
; Cutting completed.
G1 F600 V-1.000 ; Retract
G1 F1200 Z1.10
M1002
