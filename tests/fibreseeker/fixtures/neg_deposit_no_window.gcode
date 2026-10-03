; neg_deposit_no_window.gcode - fiber deposit with no M1001 window open (R06).
# EXPECT: ERROR R06
G21
G90
M83
T0
; LAYER:1 [0.2]
G1 X110.000 V0.04200 U2.00000 P0.021 F300
G1 F600 V-1
