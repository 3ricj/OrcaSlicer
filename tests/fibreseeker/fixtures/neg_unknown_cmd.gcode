; neg_unknown_cmd.gcode - command with no firmware handler and not tolerated (R01).
# EXPECT: ERROR R01
G21
G90
M83
T1
; LAYER:1 [0.2]
FOOBAR X1
G0 X150.000 F6000
