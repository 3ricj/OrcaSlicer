; neg_fan_bad_p.gcode - M106 P=9 is outside the fan port map 0..5 (R12).
# EXPECT: ERROR R12
G21
G90
M83
T1
; LAYER:1 [0.2]
M106 P9 S200
G0 X150.000 F6000
