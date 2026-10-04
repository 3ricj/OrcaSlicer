; good_toolchange_pair.gcode - the paired composite tool-change sequence, as
; Fiber::emit_toolchange_to_fiber / emit_toolchange_to_plastic now write it for
; the shipped FibreSeeker3 SK3 CF profile (fs_t0_temp 270, standby 180/150,
; plastic working 250, toolchange matrix retract V-4 @F600, brush on).
;
; This is the owner ruling of 2026-10-04 as a fixture: the vendor machine
; supplies the WHOLE sequence at every switch, so both halves must be present
; and paired. The window body between M1001 and M1002 is the byte-identical
; priming-window shape already pinned by good_prime_line.gcode, so this fixture
; isolates the tool-change blocks rather than re-testing the strand dialect.
;
; Both blocks open with M400, matching the vendor: every tool-change block in
; all six reference exports leads with a planner flush, so no queued extrusion
; can still be in flight when the temperature lines land.
;
; Temperature handling: at the plastic->fibre switch BOTH heads get a standby
; target, which is the clause the owner objective states literally, and the head
; being ACTIVATED then also gets a WORKING-temperature pre-charge immediately
; before its blocking M109,
; so it is already climbing by the time the wait is reached (M104 S270 T0 /
; M109 S270 T0 going into fibre, M104 S250 T1 / M109 S250 T1 coming out).
;
; The vendor divergence is recorded, not copied. Measured over 636 M104-bearing
; blocks in the six reference exports (315 plastic->fibre, 321 fibre->plastic),
; the vendor emits exactly two M104 per block and 0/636 give the ACTIVATED head
; a standby target. The activated head standby line below is therefore a
; measured NO-OP: that head is already parked at that temperature, because the
; opposite half dropped it there when the previous window closed, and M104 does
; not block. It is emitted inside the readiness gate only, where an M109 follows
; to undo it.
;
; What the validator independently confirms here: no bare M106 (R12W would fire
; on an un-routed fan command), every M106 P word inside 0..5 (R12, which is
; what licenses P5 here), no E move under T0 (R05), no U/V move under T1
; (R04), no tool switch with a window open (R06T), and the budget still closes.
; The two caller-side E lines are the writer own shape, not a transcription:
; the withdrawal is GCodeWriter::retract_for_toolchange(), so its feedrate is
; filament_retraction_speed (the PETG profile carries 20, overriding the machine
; 25) and its length is retract_length_toolchange = 10; the recovery is
; GCodeWriter::unretract(), whose feedrate is deretraction_speed = 25 (no filament
; override) and whose comment argument is already semicolon-led, so the formatter
; emits the doubled separator seen below. use_relative_e_distances = 1 in the
; shipped profile, so reset_e() emits no G92 line.
; The brush macros and M73 are allowlisted commands.
# EXPECT: WARN R01U
G21
G90
M83
M73 P0 Q1 L300 E300
T1 ; switch extruder type to:PLASTIC
G1 X150.000 Y120.000 F6000
G1 X152.000 E0.8 F1500
; ---- plastic -> fibre: flush, withdrawal, standby pair, wait, brush ----
M400
G1 E-10 F1200 ; retract for toolchange
M104 S150 T1 ; standby
M104 S180 T0 ; standby
M104 S270 T0 ; pre-charge
M109 S270 T0
MOVE_TO_BRUSH_STATION
CLEAN_NOZZLE
MOVE_OUT_BRUSH_STATION
M106 P2 S255 ; fibre-side cooling, fan4, owned by T0 (depositing)
M106 P1 S0 ; part-cooling, fan3, owned by T1 (idle)
M106 P3 S255 ; auxiliary fan on while T0 deposits
M106 P5 S255 ; exhaust fan on while T0 deposits
T0 ; switch extruder type to:FIBER
; LAYER:3 [0.44]
SET_PRINT_STATS_INFO CURRENT_LAYER=3
M1001 L79
G1 F1200 Z1.64
G1 X10.00 Y10.00 F1200
G1 F1200 U55.000 ; Extrude restart
G1 F1200 Z0.44
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
G1 F1200 Z1.04
M1002
; ---- fibre -> plastic: flush, second matrix withdrawal, standby, brush ----
M400
G1 F600 V-4.000 ; Toolchange matrix retract
M104 S180 T0 ; standby
M104 S250 T1 ; pre-charge
M109 S250 T1
MOVE_TO_BRUSH_STATION
CLEAN_NOZZLE
MOVE_OUT_BRUSH_STATION
M106 P2 S0 ; fibre-side cooling, fan4, owned by T0 (idle)
M106 P1 S255 ; part-cooling, fan3, owned by T1 (depositing)
M106 P3 S0 ; auxiliary fan off while T1 deposits
M106 P5 S0 ; exhaust fan off while T1 deposits
T1 ; switch extruder type to:PLASTIC
G1 E10 F1500  ;  ; unretract
G1 X154.000 E0.8 F1500
