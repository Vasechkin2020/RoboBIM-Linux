; === GENERATED G-CODE (v3.9.1) ===
; Output Filename: kvadrat50.gcode
; Robot System Coordinate Origin on Map: X=0.000, Y=0.000, Angle=0.0
; Marker Offset: 0.0450
G10 X1.250 Y1.250 A0.000 ; Initial required start point and angle
M5 T2000 ; Pen UP
G1 A0.000 F0.05
G2 L0.045 F-0.10 ; Initial align

; ---------------------------------------------------
; CAD:   (1.250, 1.250) -> (1.750, 1.250)
; ROBOT: (1.250, 1.250) -> (1.750, 1.250) | Ang: 0.00
M5 T2000
G2 X1.250 Y1.250 F0.10 ; Center to Start
G4 P2000
G1 A0.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.705 Y1.250 F0.10

; ---------------------------------------------------
; CAD:   (1.750, 1.250) -> (1.750, 1.750)
; ROBOT: (1.750, 1.250) -> (1.750, 1.750) | Ang: 90.00
M5 T2000
G2 X1.750 Y1.250 F0.10 ; Center to Start
G4 P2000
G1 A90.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.750 Y1.705 F0.10

; ---------------------------------------------------
; CAD:   (1.750, 1.750) -> (1.250, 1.750)
; ROBOT: (1.750, 1.750) -> (1.250, 1.750) | Ang: 180.00
M5 T2000
G2 X1.750 Y1.750 F0.10 ; Center to Start
G4 P2000
G1 A180.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.295 Y1.750 F0.10

; ---------------------------------------------------
; CAD:   (1.250, 1.750) -> (1.250, 1.250)
; ROBOT: (1.250, 1.750) -> (1.250, 1.250) | Ang: -90.00
M5 T2000
G2 X1.250 Y1.750 F0.10 ; Center to Start
G4 P2000
G1 A-90.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.250 Y1.295 F0.10

; === FINALIZE SEQUENCE ===
M5 T2000 ; Pen UP
G2 X1.250 Y1.250 F0.10 ; Align Center to Last Point (Undo Offset)
G4 P1000
G1 A0.000 F0.05 ; Final rotation to 0 degrees

; === END PROGRAM ===

; === ORIGINAL INPUT LINES START (Input backup) ===
; kvadrat50.gcode; Имя выходного файла
; 0.0 0.0 0.0 ; Смещение системы координат робота от системы координат чертежа. Где стоит ноль робота на чертеже.
; 
; 1.25 1.25 1.75 1.25
; 1.75 1.25 1.75 1.75
; 1.75 1.75 1.25 1.75
; 1.25 1.75 1.25 1.25
; === ORIGINAL INPUT LINES END ===
