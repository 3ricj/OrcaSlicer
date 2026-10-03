; neg_uv_on_t1.gcode - U/V material move while T1 active (R04) and no window (R06).
# EXPECT: ERROR R04
# EXPECT: ERROR R06
G21
G90
M83
T1
; LAYER:1 [0.2]
G1 X110.000 V0.04200 U2.00000 P0.021 F300
