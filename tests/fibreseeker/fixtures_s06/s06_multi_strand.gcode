; s06_multi_strand.gcode - several strands inside BOTH a departing
; activation and the final activation.
;
;   Activation 1 (T0, strands 1-2) hands over to T1:
;     W1 interstrand  - closes on T0, W2 opens before any head change
;     W2 departure    - the T0->T1 change lands before the next window
;   Activation 2 (T0, strands 3-4) is the last one and never hands over:
;     W3 interstrand  - closes on T0, W4 opens, no head change precedes it
;     W4 terminal     - last window, no later window AND no later head
;                       change, so shutdown follows, not an incoming head
;
; The point: W2 and W4 sit at the same structural position (last strand
; of their activation) and get OPPOSITE kinds, because what follows the
; activation differs. W2 has a head change coming first; W4 has nothing.
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
G1 X108.500 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 X110.000 Y10.000 V0.480 F600
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
G1 F600 V-1.000 ; Retract
; FS_RELEASE_BEGIN length_mm=6.800 speed_mm_s=10.000
G1 X128.800 Y10.000 F600
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
G1 F600 V-1.000 ; Retract
M1002
G1 F1200 Z0.80 ; restart hop, 0.60 mm clearance
G0 X170.000 Y10.000 F6000
M1001 L57
G1 F1200 U55
G0 X170.000 Y10.000 Z0.20 F30000
G1 F600 V4
G1 X171.000 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 X172.000 Y10.000 V0.480 F600
G1 F600 V-1.000 ; Retract
; FS_RELEASE_BEGIN length_mm=6.800 speed_mm_s=10.000
G1 X178.800 Y10.000 F600
; FS_RELEASE_END physical_clearance=unmeasured
M1002
M400
G1 F1200 Z1.40 ; shutdown clearance
M107
; shutdown
