; neg_e_with_uv.gcode - E mixed with U/V on one deposit move (R02).
; Under an active T0 the same move also violates the no-E-on-T0 rule, so the
; honest expected set is {R02, R05}: the firmware rejects E+U/V mixing on the
; move itself (R02) independently of which tool is active (R05).
# EXPECT: ERROR R02
# EXPECT: ERROR R05
# EXPECT: WARN R01U
G21
G90
M83
T0
; LAYER:1 [0.2]
M1001 L57
G1 F1500 U55
G1 F600 V4
G1 X110.000 V0.04200 U2.00000 E1.0 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
