; s06_close_kinds.gcode - one file exercising all three close kinds.
;
;   W1 inter-strand: closes on T0, next window opens before any head change,
;      and NOTHING moves in Z. S06 owes release+closure only.
;   W2 inter-strand with a travel hop: same, but a Z rise to 0.80 from 0.20
;      (exactly the 0.60 mm clearance) sits between the close and the next
;      window. A hop is accepted; an inadequate one is not.
;   W3 departure: closes on T0 and the T0->T1 change lands between windows, so
;      it owes withdrawal -> clearance lift -> station entry, in that order.
;   W4 terminal: closes on T0 with no head change anywhere after it. It is
;      terminal, NOT inter-strand: it is followed by shutdown, not by another
;      strand. Closure is the whole duty; no incoming-head transition exists.
;
; Pinned by tests/fibreseeker/test_fs_s06_close_kinds.py.
G21
G90
M83
T0
M106 P2 S255
M1001 L57
G1 F1200 U55
G0 X107.500 Y10.000 Z0.20 F30000
G1 F600 V4
G1 X108.000 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 X110.000 Y10.000 V0.480 F600
; Cutting completed.
G1 F600 V-1.000 ; Retract
M1002
G0 X120.000 Y10.000 F6000
M1001 L57
G1 F1200 U55
G0 X120.000 Y10.000 Z0.20 F30000
G1 F600 V4
G1 X121.000 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 X122.000 Y10.000 V0.480 F600
; Cutting completed.
G1 F600 V-1.000 ; Retract
M1002
G1 F1200 Z0.80 ; restart hop, 0.60 mm clearance
G0 X130.000 Y10.000 F6000
M1001 L57
G1 F1200 U55
G0 X130.000 Y10.000 Z0.20 F30000
G1 F600 V4
G1 X131.000 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 X132.000 Y10.000 V0.480 F600
; Cutting completed.
G1 F600 V-1.000 ; Retract
; FS_RELEASE_BEGIN length_mm=6.800 speed_mm_s=10.000
G1 X138.800 Y10.000 F600
; FS_RELEASE_END physical_clearance=unmeasured
M1002
M400
G1 F600 V-4.000 ; Toolchange matrix retract
G1 F1200 Z0.80
MOVE_TO_BRUSH_STATION
CLEAN_NOZZLE
M104 S180 T0 ; standby
M104 S250 T1 ; pre-charge
M109 S250 T1
MOVE_OUT_BRUSH_STATION
T1 ; switch extruder type to:PLASTIC
G1 X140.000 Y10.000 F3000
G1 X150.000 Y10.000 E1.000 F1500
T0 ; switch extruder type to:COMPOSITE
M1001 L57
G1 F1200 U55
G0 X160.000 Y10.000 Z0.20 F30000
G1 F600 V4
G1 X161.000 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 X162.000 Y10.000 V0.480 F600
; Cutting completed.
G1 F600 V-1.000 ; Retract
; FS_RELEASE_BEGIN length_mm=6.800 speed_mm_s=10.000
G1 X168.800 Y10.000 F600
; FS_RELEASE_END physical_clearance=unmeasured
M1002
M400
G1 F1200 Z1.40 ; shutdown clearance
M107
; shutdown
