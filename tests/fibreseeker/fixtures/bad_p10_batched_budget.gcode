# P10.0/B0 audit-probe negative fixture: identical bytes to
# good_p10_batched_window.gcode except the advisory window budget claims L60
# while the batch carries 55 (restart) + 20 (deposits) = 75 mm of U. The
# validator must reject the window as over-budget (R07), proving the batched
# budget convention is enforced from the emitted text alone.
# EXPECT: ERROR R07
# EXPECT: WARN R01U
# EXPECT: WARN R15
T0
M104 S270 T0
M109 S270 T0
; LAYER:7 [0.50]
M1001 L60
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
