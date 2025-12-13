; === GENERATED G-CODE (v3.9.5) ===
; Output Filename: kvadrat70.gcode
; Marker Offset: 0.0450
; Dwell Time: 2000 ms
G10 X1.100 Y1.100 A0.000 ; Initial pose set
M5 T2000 ; Pen UP
; ---------------------------------------------------
; CAD: (1.100, 1.100) -> (1.800, 1.100)
; Case: First Line (Already at start)
G1 A0.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Initial Backup
M3 T2000
G2 X1.755 Y1.100 F0.10

; ---------------------------------------------------
; CAD: (1.800, 1.100) -> (1.800, 1.800)
; Case: Connected Corner (Using only L moves)
M5 T2000
G2 L0.045 F0.10 ; Step forward into corner
G4 P2000
G1 A90.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Backup for next line
M3 T2000
G2 X1.800 Y1.755 F0.10

; ---------------------------------------------------
; CAD: (1.800, 1.800) -> (1.100, 1.800)
; Case: Connected Corner (Using only L moves)
M5 T2000
G2 L0.045 F0.10 ; Step forward into corner
G4 P2000
G1 A180.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Backup for next line
M3 T2000
G2 X1.145 Y1.800 F0.10

; ---------------------------------------------------
; CAD: (1.100, 1.800) -> (1.100, 1.100)
; Case: Connected Corner (Using only L moves)
M5 T2000
G2 L0.045 F0.10 ; Step forward into corner
G4 P2000
G1 A-90.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Backup for next line
M3 T2000
G2 X1.100 Y1.145 F0.10

; === FINALIZE SEQUENCE ===
M5 T2000 ; Pen UP
G2 L0.045 F0.10 ; Align Center (Undo final offset)
G4 P2000
G1 A-135.000 F0.05 ; Rotate to Park
G4 P2000
G2 X1.000 Y1.000 F0.10 ; Move to Park
G4 P2000
G1 A0.000 F0.05 ; Final rotation

; === END PROGRAM ===

; === ORIGINAL INPUT LINES START ===
; kvadrat70.gcode; Имя выходного файла
; 0.0 0.0 0.0 ; Смещение системы координат робота от системы координат чертежа. Где стоит ноль робота на чертеже.
; 
; 1.1 1.1 1.8 1.1
; 1.8 1.1 1.8 1.8
; 1.8 1.8 1.1 1.8
; 1.1 1.8 1.1 1.1
; 1.0 1.0
; === ORIGINAL INPUT LINES END ===
