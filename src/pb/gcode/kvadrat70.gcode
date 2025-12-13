; === GENERATED G-CODE (v3.9.1) ===
; Output Filename: kvadrat70.gcode
; Robot System Coordinate Origin on Map: X=0.000, Y=0.000, Angle=0.0
; Marker Offset: 0.0450
G10 X1.100 Y1.100 A0.000 ; Initial required start point and angle
M5 T2000 ; Pen UP
G1 A0.000 F0.05
G2 L0.045 F-0.10 ; Initial align

; ---------------------------------------------------
; CAD:   (1.100, 1.100) -> (1.800, 1.100)
; ROBOT: (1.100, 1.100) -> (1.800, 1.100) | Ang: 0.00
M5 T2000
G2 X1.100 Y1.100 F0.10 ; Center to Start
G4 P2000
G1 A0.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.755 Y1.100 F0.10

; ---------------------------------------------------
; CAD:   (1.800, 1.100) -> (1.800, 1.800)
; ROBOT: (1.800, 1.100) -> (1.800, 1.800) | Ang: 90.00
M5 T2000
G2 X1.800 Y1.100 F0.10 ; Center to Start
G4 P2000
G1 A90.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.800 Y1.755 F0.10

; ---------------------------------------------------
; CAD:   (1.800, 1.800) -> (1.100, 1.800)
; ROBOT: (1.800, 1.800) -> (1.100, 1.800) | Ang: 180.00
M5 T2000
G2 X1.800 Y1.800 F0.10 ; Center to Start
G4 P2000
G1 A180.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.145 Y1.800 F0.10

; ---------------------------------------------------
; CAD:   (1.100, 1.800) -> (1.100, 1.100)
; ROBOT: (1.100, 1.800) -> (1.100, 1.100) | Ang: -90.00
M5 T2000
G2 X1.100 Y1.800 F0.10 ; Center to Start
G4 P2000
G1 A-90.000 F0.05
G4 P2000
G2 L0.045 F-0.10 ; Back up
M3 T2000
G2 X1.100 Y1.145 F0.10

; === FINALIZE SEQUENCE ===
M5 T2000 ; Pen UP
G2 X1.100 Y1.100 F0.10 ; Align Center to Last Point (Undo Offset)
G4 P1000
G1 A-135.000 F0.05 ; Rotate to Park
G4 P1000
G2 X1.000 Y1.000 F0.10 ; Move to Park Position
G4 P1000
G1 A0.000 F0.05 ; Final rotation to 0 degrees

; === END PROGRAM ===

; === ORIGINAL INPUT LINES START (Input backup) ===
; kvadrat70.gcode; Имя выходного файла
; 0.0 0.0 0.0 ; Смещение системы координат робота от системы координат чертежа. Где стоит ноль робота на чертеже.
; 
; 1.1 1.1 1.8 1.1
; 1.8 1.1 1.8 1.8
; 1.8 1.8 1.1 1.8
; 1.1 1.8 1.1 1.1
; 1.0 1.0
; === ORIGINAL INPUT LINES END ===
