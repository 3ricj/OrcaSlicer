; good_interstrand_then_departure.gcode - two strands inside ONE T0
; activation, then a real T0 departure.
;
; Both closes happen while T0 is selected, yet only the second is a departure:
;
;   W1 closes and the next fibre window opens before any head change, so T0
;      stays active between strands and NO departure withdrawal is owed.
;   W2 closes and the head change T0->T1 lands between the windows, so T0
;      really leaves and exactly one V -4.000 is owed before the lift.
;
; A rule of the form "head at close == 0 therefore owe V-4" demands a
; withdrawal at W1 and is wrong. Pinned by
; tests/fibreseeker/test_fs_departure_rule.py.
G21
G90
M83
T0
M106 P2 S255
M1001 L57
G1 F1200 U55
G0 X107.500 Y10.000 Z0.200 F30000
G1 F600 V4
G1 X110.000 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
G0 X120.000 Y10.000 F6000
M1001 L57
G1 F1200 U55
G0 X120.000 Y10.000 Z0.200 F30000
G1 F600 V4
G1 X122.000 Y10.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
M400
G1 F600 V-4.000 ; Toolchange matrix retract
G1 F1200 Z1.08
MOVE_TO_BRUSH_STATION
CLEAN_NOZZLE
M104 S180 T0 ; standby
M104 S250 T1 ; pre-charge
M109 S250 T1
MOVE_OUT_BRUSH_STATION
T1 ; switch extruder type to:PLASTIC
G1 X130.000 Y10.000 F3000
G1 X140.000 Y10.000 E1.000 F1500
