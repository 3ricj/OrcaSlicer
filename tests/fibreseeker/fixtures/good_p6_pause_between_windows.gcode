; good_p6_pause_between_windows.gcode - operator-safe pause boundary (P6, U11).
; Recommended pause position: after a fiber window is fully closed (cut, retract,
; M1002) and before the next M1001. PAUSE/RESUME run entirely out-of-band
; (host-triggered macros in print_control.cfg, E19): the exported file is
; pause-invariant, so the validator sees this same clean shape with or without
; a pause. Comments annotate what the firmware executes at the marked boundary.
# EXPECT: WARN R01U
G21
G90
M83
T0
M106 P2 S255
; LAYER:1 [0.2]
M1001 L57
G1 F1200 U55 ; Extrude restart
G0 X107.500 Z0.200 F30000
G1 F600 V4 ; Extrude restart
G1 X110.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
; === SAFE PAUSE BOUNDARY -------------------------------------------------
; PAUSE: M2800+M400 (redundant here - window already cut), Z lift, nozzle
; clean, dock G1 X50 Y100 F6000, V-2 maintenance retract (>=180C),
; extrude_restart_flag=False, pause position + material delta recorded.
; RESUME: temps/fans restored, SET_EXTRUDER_MODE S=0, M83,
; G1 F<saved> U<saved delta> re-prime, RESTORE_NOZZLE_TO_PRINT, return to
; pause X/Y/Z, TENSION_SENSOR_RESUME, RESUME_BASE.
; Note: the V-2 retract biases the absolute V base by -2 mm until the next
; window's absolute V prime rebases it -> pause at window boundaries.
; --------------------------------------------------------------------------
G0 X120.000 F600
M1001 L57
G1 F1200 U55 ; Extrude restart
G0 X120.000 Z0.200 F30000
G1 F600 V4 ; Extrude restart
G1 X122.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
