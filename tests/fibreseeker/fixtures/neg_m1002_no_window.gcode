; neg_m1002_no_window.gcode - window close without an open window (R06C).
# EXPECT: ERROR R06C
# EXPECT: WARN R01U
G21
G90
M83
T0
; LAYER:1 [0.2]
M1002
G0 X150.000 F6000
