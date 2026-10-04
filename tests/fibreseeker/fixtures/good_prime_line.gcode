; good_prime_line.gcode - composite-head priming window (fs_fiber_prime).
; Positive fixture: the exact block Fiber::emit_fiber_prime_line writes for the
; shipped CF-nozzle profile (restart 55 @F1200, hop 1.2, prime V4, retract V1,
; rate 0.98, P 0.034, feed 1200, tail 54.8) over an 80 mm line from (10,10) to
; (90,10) at Z 0.20, tool-wrapped. tests/libslic3r/test_fiber_strand.cpp asserts
; the emitter still produces this block byte-for-byte.
;
; Shape is the reference priming window (see good_priming.gcode): window open,
; U-only restart, V-only prime, joint deposits, cut, V-only tail, retract,
; window close. It sits before any '; LAYER:' marker because it belongs to no
; layer, which is exactly what WARN R13 means to the validator.
# EXPECT: WARN R01U
# EXPECT: WARN R13
G21
G90
M83
T0 ; switch extruder type to:FIBER
M1001 L79
G1 F1200 Z1.40
G1 X10.00 Y10.00 F1200
G1 F1200 U55.000 ; Extrude restart
G1 F1200 Z0.20
G1 F600 V1.000 ; Recover matrix retract
G1 F600 V3.000 ; Matrix prime
G1 X35.20 Y10.00 V0.840 U24.696 P0.034 F1200
; Start to cut
M2800
M400
;CUT DISTANCE 54.8
G1 X90.00 Y10.00 V1.826 F1200
; Cutting completed.
G1 F600 V-1.000 ; Retract
G1 F1200 Z0.80
M1002
T1 ; switch extruder type to:PLASTIC
; LAYER:1 [0.20]
SET_PRINT_STATS_INFO CURRENT_LAYER=1
G1 X150.000 Y120.000 F6000
G1 X152.000 E0.8 F1500
