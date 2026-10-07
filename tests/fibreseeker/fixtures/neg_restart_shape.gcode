; neg_restart_shape.gcode - wrong lifecycle shape: V-only prime comes first (R08),
; and the second material move is a joint U+V move instead of a V-only prime (R08P).
# EXPECT: ERROR R08
# EXPECT: ERROR R08P
# EXPECT: WARN R01U
G21
G90
M83
T0
; LAYER:1 [0.2]
M1001 L55
G1 F600 V4
G1 F1500 U55 V1.155 P0.021
M2800
M400
G1 F600 V-1
M1002
