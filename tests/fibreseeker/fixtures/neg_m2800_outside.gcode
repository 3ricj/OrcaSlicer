; neg_m2800_outside.gcode - cutter fired with no fiber window open (R09X).
# EXPECT: ERROR R09X
G21
G90
M83
T0
; LAYER:1 [0.2]
M2800
M400
G0 X150.000 F6000
