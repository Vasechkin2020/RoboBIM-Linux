; === GENERATED G-CODE (v3.9.7 Coords Match) ===
; Output Filename: balorobot24.gcode
; Marker Offset: 0.0450
; Dwell Time: 2000 ms
G10 X2.000 Y1.000 A0.000 ; Initial pose set
M5 T2000 ; Pen UP
; ---------------------------------------------------
; CAD: (2.000, 1.000) -> (4.000, 1.000)
; Case: First Line
G1 X3.955 Y1.000 F0.05 ; Rotate to Target
G4 P2000
G2 L0.045 F-0.10 ; Initial Backup
M3 T2000
G2 X3.955 Y1.000 F0.10

; ---------------------------------------------------
; CAD: (4.000, 1.000) -> (4.000, 2.000)
; Case: Connected Corner
M5 T2000
G2 L0.045 F0.10 ; Step into corner
G4 P2000
G1 X4.000 Y1.955 F0.05 ; Rotate to Target
G4 P2000
G2 L0.045 F-0.10 ; Backup
M3 T2000
G2 X4.000 Y1.955 F0.10

; ---------------------------------------------------
; CAD: (4.000, 2.000) -> (2.000, 1.000)
; Case: Connected Corner
M5 T2000
G2 L0.045 F0.10 ; Step into corner
G4 P2000
G1 X2.040 Y1.020 F0.05 ; Rotate to Target
G4 P2000
G2 L0.045 F-0.10 ; Backup
M3 T2000
G2 X2.040 Y1.020 F0.10

; ---------------------------------------------------
; CAD: (2.000, 1.000) -> (2.000, 2.000)
; Case: Connected Corner
M5 T2000
G2 L0.045 F0.10 ; Step into corner
G4 P2000
G1 X2.000 Y1.955 F0.05 ; Rotate to Target
G4 P2000
G2 L0.045 F-0.10 ; Backup
M3 T2000
G2 X2.000 Y1.955 F0.10

; ---------------------------------------------------
; CAD: (2.000, 2.000) -> (4.000, 1.000)
; Case: Connected Corner
M5 T2000
G2 L0.045 F0.10 ; Step into corner
G4 P2000
G1 X3.960 Y1.020 F0.05 ; Rotate to Target
G4 P2000
G2 L0.045 F-0.10 ; Backup
M3 T2000
G2 X3.960 Y1.020 F0.10

; ---------------------------------------------------
; CAD: (4.000, 1.000) -> (3.000, 2.000)
; Case: Connected Corner
M5 T2000
G2 L0.045 F0.10 ; Step into corner
G4 P2000
G1 X3.032 Y1.968 F0.05 ; Rotate to Target
G4 P2000
G2 L0.045 F-0.10 ; Backup
M3 T2000
G2 X3.032 Y1.968 F0.10

; ---------------------------------------------------
; CAD: (3.000, 2.000) -> (2.000, 1.000)
; Case: Connected Corner
M5 T2000
G2 L0.045 F0.10 ; Step into corner
G4 P2000
G1 X2.032 Y1.032 F0.05 ; Rotate to Target
G4 P2000
G2 L0.045 F-0.10 ; Backup
M3 T2000
G2 X2.032 Y1.032 F0.10

; === FINALIZE SEQUENCE ===
M5 T2000 ; Pen UP
G2 L0.045 F0.10 ; Align Center
G4 P2000
G1 X1.000 Y1.000 F0.05 ; Rotate to Park
G4 P2000
G2 X1.000 Y1.000 F0.10 ; Move to Park
G4 P2000
G1 A0.000 F0.05 ; Final rotation

; === END PROGRAM ===

; === ORIGINAL INPUT LINES START ===
; balorobot24.gcode; Имя выходного файла
; 0.0 0.0 0.0 ; Смещение системы координат робота от системы координат чертежа. Где стоит ноль робота на чертеже.
;  
; 2.0 1.0 4.0 1.0
; 4.0 1.0 4.0 2.0
; 4.0 2.0 2.0 1.0
; 2.0 1.0 2.0 2.0
; 2.0 2.0 4.0 1.0
; 4.0 1.0 3.0 2.0
; 3.0 2.0 2.0 1.0
; 1.0 1.0
; === ORIGINAL INPUT LINES END ===
