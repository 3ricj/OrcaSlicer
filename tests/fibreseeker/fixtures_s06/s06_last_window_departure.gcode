; s06_last_window_departure.gcode - the last window of the file DEPARTS.
;
;   W1 interstrand - closes on T0, W2 opens before any head change
;   W2 departure   - the last window there is, but a T0->T1 change
;                    follows it, so T0 hands over into a non-depositing
;                    tail and owes withdrawal + lift + station entry
;
; The previous rule read 'no physical head change after the close' as
; proof of terminal and 'a head change exists' as insufficient when no
; window followed. Both are wrong: a last window with a head change is a
; DEPARTURE, and a last window with no head change is TERMINAL.
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
M400
G1 F1200 Z1.40 ; shutdown clearance
M107
; shutdown
