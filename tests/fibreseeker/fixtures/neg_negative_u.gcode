; neg_negative_u.gcode - fiber retraction attempt (R03).
# EXPECT: ERROR R03
G21
G90
M83
T0
; LAYER:1 [0.2]
G1 X110.000 U-5.0 V-1.0 F600
