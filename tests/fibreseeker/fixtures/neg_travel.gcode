; neg_travel.gcode - X target beyond the machine travel limit (R14).
; Machine travel X -45..325 per data/machine_profile.json.
# EXPECT: ERROR R14
G21
G90
M83
T1
; LAYER:1 [0.2]
G0 X400.000 F6000
G0 X150.000 F6000
