# Dialect fixture: one fiber window, U restart, V prime, joint deposits, cut, close.
# for a finalized FiberRun: 10 mm square (100,100)-(110,110), layer 3, z=0.30,
# fiber_rate 0.25 mm/mm, P=2.0, deposit feed 1200, default FiberEmitParams
# (restart 55 @F1200, hop 1.2, prime V4 @F600, tail 54.8, separation 10 @F1200,
# retract V-1 @F600, lift 0.6). tests/libslic3r/test_fiber_run.cpp asserts the
# emitter still produces the lifecycle block below byte-for-byte. The single
# leading `T0` is the tool-activation prelude supplied by the machine sequence
# (the emitter itself is pure), needed for the validator's tool tracking.
# This file independently validates clean through fs_gcode_validator.
# EXPECT: WARN R01U
# EXPECT: WARN R15
T0
; LAYER:3 [0.30]
M1001 L65
G1 F1200 Z1.50
G1 X100.00 Y100.00 F1200
G1 F1500 U55.000 ; Extrude restart
G1 F1200 Z0.30
G1 F600 V4.000 ; Extrude restart
G1 X110.00 Y100.00 V5.000 U2.500 P2.000 F1200
G1 X110.00 Y110.00 V5.000 U2.500 P2.000 F1200
G1 X100.00 Y110.00 V5.000 U2.500 P2.000 F1200
G1 X100.00 Y100.00 V5.000 U2.500 P2.000 F1200
; Start to cut
M2800
M400
;CUT DISTANCE 54.8
G1 X100.00 Y90.00 F1200
; Cutting completed.
G1 F600 V-1.000 ; Retract
G1 F1200 Z0.90
M1002
