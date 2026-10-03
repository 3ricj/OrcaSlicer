; neg_e_on_t0.gcode - plastic extrusion while T0 composite head active (R05).
; R05 is tool-gated, so the bare T0 statement is required here.
# EXPECT: ERROR R05
G21
G90
M83
T0
; LAYER:1 [0.2]
G1 X150.000 E0.5 F1500
