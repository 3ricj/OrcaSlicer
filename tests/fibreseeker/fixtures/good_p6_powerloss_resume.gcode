; good_p6_powerloss_resume.gcode - powerloss recovery preamble followed by the
; resumed fiber window (P6, U11). Shape of what the machine executes after a
; power failure mid-print (E19: patched virtual_sdcard.py): the recovery driver
; (PREPARE_RECOVER_PRINT_FILE + RECOVER_PRINT_FILE) re-homes, re-heats, re-activates
; the recorded tool, cuts the fiber (M2800, T0), restores material positions with
; G92 E/U/V, then continues the file from the recorded byte offset. This fixture
; flattens that out-of-band preamble into the file (allowlist-only subset) and
; continues with a fresh M1001 window. Validation point: after recovery the print
; re-enters at a window boundary, so every contract rule still holds. There is no
; U-re-prime in the powerloss path (unlike RESUME): first-deposit quality after
; recovery is a hardware-qualification item.
# EXPECT: WARN R01U
G21
G90
M83
; === POWERLOSS RECOVERY PREAMBLE (recovery driver injected, out-of-band) ====
SET_KINEMATIC_POSITION Z=0.0
G1 Z4.0 F300
G28 X Y
MOVE_TO_BRUSH_STATION
TENSION_SENSOR_START
M140 S60
M104 S230
M141 S45
M109 S230
M190 S60
T0
MOVE_TO_BRUSH_STATION
G90
G1 X150.000 Y150.000 F12000
M221 S100
M106 P2 S255
G92 E0 U0 V0
M83
M400
M2000 L6
; === RESUMED WINDOW (file continues at recorded byte offset) ================
; LAYER:12 [2.4]
M1001 L57
G1 F1200 U55 ; Extrude restart
G0 X200.000 Z2.400 F30000
G1 F600 V4 ; Extrude restart
G1 X202.000 V0.04200 U2.00000 P0.021 F300
M2800
M400
G1 F600 V-1
M1002
