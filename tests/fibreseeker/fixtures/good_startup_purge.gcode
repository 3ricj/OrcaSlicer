; good_startup_purge.gcode - the composite-head startup purge, closed by the
; paired tool-change sequence (fs_fiber_prime + the tool-change module).
;
; Positive fixture for the startup transition, which is the one place the
; exporter reaches a fibre window before any layer exists. It is the bytes
; Fiber::emit_toolchange_to_fiber + Fiber::emit_fiber_prime_line (unwrapped) +
; Fiber::emit_toolchange_to_plastic write for the shipped FibreSeeker3 SK3 CF
; profile (fs_t0_temp 270, standby 180/150, plastic working 250, toolchange
; matrix retract V-4 @F600, brush on, aux fans on), with the cooling demand not
; yet resolved so no demand-derived P1/P2 lines are emitted.
;
; The point of this fixture is the two halves AROUND the window, not the window:
; good_prime_line.gcode already pins the strand lifecycle byte-for-byte.
;
; Startup entry (plastic -> fibre) carries what a bare T0 did not: a standby
; target on the plastic head being put away, so it is not left at its printing
; temperature through the purge dwell, plus the working-temperature pre-charge
; and the ONE blocking wait the plate pays for the composite head.
;
; Startup exit (fibre -> plastic) carries what the strand's own V-retract plus a
; bare T1 did not: the per-window matrix withdrawal (the third quantity of the
; stationary-V ledger, legal only while T0 is still selected), the standby park
; of the composite head, the blocking restore of the plastic head, the brush
; visit, and the P3/P5 drive. Without the park and the restore, the composite
; head stayed commanded at 270 for the whole dwell between the purge and the
; first model fibre strand - the ooze window this sequence exists to remove.
;
; The park is what forces the plate's first model window to re-charge and wait
; for T0 rather than assume it is still hot from startup; that is the exporter's
; m_fs_t0_hot latch, not emitted text, and good_toolchange_pair.gcode pins the
; re-waited entry.
;
; The window carries no '; LAYER:' marker because it belongs to no layer, which
; is what WARN R13 means. The validator independently confirms here: no tool
; switch with a window open (R06T), no E move under T0 (R05), no U/V move under
; T1 (R04), every M106 P word inside 0..5 (R12, which licenses P5), and the
; budget closes.
# EXPECT: WARN R01U
# EXPECT: WARN R13
G21
G90
M83
M104 S270 T0
M400
M104 S150 T1 ; standby
M104 S180 T0 ; standby
M104 S270 T0 ; pre-charge
M109 S270 T0
MOVE_TO_BRUSH_STATION
CLEAN_NOZZLE
MOVE_OUT_BRUSH_STATION
M106 P3 S255 ; auxiliary fan on while T0 deposits
M106 P5 S255 ; exhaust fan on while T0 deposits
T0 ; switch extruder type to:FIBER
M1001 L79
G1 F1200 Z1.40
G1 X10.00 Y10.00 F1200
G1 F1500 U55.000 ; Extrude restart
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
M400
G1 F600 V-4.000 ; Toolchange matrix retract
M104 S180 T0 ; standby
M104 S250 T1 ; pre-charge
M109 S250 T1
MOVE_TO_BRUSH_STATION
CLEAN_NOZZLE
MOVE_OUT_BRUSH_STATION
M106 P3 S0 ; auxiliary fan off while T1 deposits
M106 P5 S0 ; exhaust fan off while T1 deposits
T1 ; switch extruder type to:PLASTIC
; LAYER:1 [0.20]
SET_PRINT_STATS_INFO CURRENT_LAYER=1
G1 X150.000 Y120.000 F6000
G1 X152.000 E0.8 F1500
