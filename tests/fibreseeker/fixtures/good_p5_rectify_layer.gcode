# P5 rectilinear-stripe layer: the exact byte output of
# "rectilinear dialect fixture: square /
# angle 0 / spacing 4 -> three stripes" (three horizontal stripes over a
# 100..110 mm square, produced by Fiber::build_rectify_runs over one contour
# ring and emitted with default FiberEmitParams), prefixed with the T0
# composite-head activation prelude of machine contract s8.5. The C++ golden
# test and this independent validator lock the same bytes from both sides
# (dual-lock pattern, cf. P3/P4).
# EXPECT: WARN R01U
# EXPECT: WARN R15
T0
M104 S270 T0
M109 S270 T0
; LAYER:5 [0.50]
M1001 L57
G1 F1200 Z1.70
G1 X100.00 Y101.00 F1200
G1 F1200 U55.000 ; Extrude restart
G1 F1200 Z0.50
G1 F600 V4.000 ; Extrude restart
G1 X110.00 Y101.00 V5.000 U2.500 P2.000 F1200
; Start to cut
M2800
M400
;CUT DISTANCE 54.8
G1 X120.00 Y101.00 F1200
; Cutting completed.
G1 F600 V-1.000 ; Retract
G1 F1200 Z1.10
M1002
M1001 L57
G1 F1200 Z1.70
G1 X100.00 Y105.00 F1200
G1 F1200 U55.000 ; Extrude restart
G1 F1200 Z0.50
G1 F600 V4.000 ; Extrude restart
G1 X110.00 Y105.00 V5.000 U2.500 P2.000 F1200
; Start to cut
M2800
M400
;CUT DISTANCE 54.8
G1 X120.00 Y105.00 F1200
; Cutting completed.
G1 F600 V-1.000 ; Retract
G1 F1200 Z1.10
M1002
M1001 L57
G1 F1200 Z1.70
G1 X100.00 Y109.00 F1200
G1 F1200 U55.000 ; Extrude restart
G1 F1200 Z0.50
G1 F600 V4.000 ; Extrude restart
G1 X110.00 Y109.00 V5.000 U2.500 P2.000 F1200
; Start to cut
M2800
M400
;CUT DISTANCE 54.8
G1 X120.00 Y109.00 F1200
; Cutting completed.
G1 F600 V-1.000 ; Retract
G1 F1200 Z1.10
M1002
