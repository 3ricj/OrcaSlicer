; warn_mode_and_shape.gcode - warning-class coverage without any error:
;   R07W  M1001 without L (budget unverifiable)
;   R16   restart feed 40 below calibrated tail 54.8
;   R12W  M106 without explicit P
;   R13   deposit before any '; LAYER:' marker
;   R13W  layer marker decreases (2 -> 1)
;   R01U  tolerated-unknown M1001/M1002
# EXPECT: WARN R07W
# EXPECT: WARN R16
# EXPECT: WARN R12W
# EXPECT: WARN R13
# EXPECT: WARN R13W
# EXPECT: WARN R01U
G21
G90
M83
T0
M106 S255
M1001
G1 F1200 U40
G1 F600 V4
G1 X110.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
; LAYER:2 [0.4]
; LAYER:1 [0.2]
G0 X150.000 F6000
