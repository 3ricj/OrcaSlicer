; neg_fiber_without_matrix.gcode - after prime, a deposit move carries U>0 with no V
; (fiber deposited without matrix support) (R10).
# EXPECT: ERROR R10
# EXPECT: WARN R01U
G21
G90
M83
T0
; LAYER:1 [0.2]
M1001 L65
G1 F1500 U55
G1 F600 V4
G1 X110.000 U10.00000 F300
M2800
M400
G1 F600 V-1
M1002
