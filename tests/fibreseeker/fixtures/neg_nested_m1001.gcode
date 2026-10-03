; neg_nested_m1001.gcode - second M1001 while first window still open (R06N).
; Everything else legal: single window lifecycle, budget exact (L57 = 55 + 2).
# EXPECT: ERROR R06N
# EXPECT: WARN R01U
G21
G90
M83
T0
; LAYER:1 [0.2]
M1001 L57
M1001 L57
G1 F1200 U55
G1 F600 V4
G1 X110.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
