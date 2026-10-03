; warn_first_move.gcode - first move issued before G21/G90 setup (R15, warning class).
# EXPECT: WARN R15
T0
; LAYER:1 [0.2]
G0 X150.000 F6000
G1 F600 V-1
