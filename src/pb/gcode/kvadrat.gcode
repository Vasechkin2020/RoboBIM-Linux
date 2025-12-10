; === GENERATED G-CODE (v3.8) ===
; Output Filename: kvadrat.gcode
; Robot System Coordinate Origin on Map: X=0.000, Y=0.000, Angle=0.0
; Marker Offset: 0.0450
G10 X1.000 Y1.000 A0.000 ; Initial required start point and angle in robot's local coordinates
M5 T1000 ; Pen UP
G1 A0.000 F0.05
G2 L0.045 F-0.10 ; Initial align

; ---------------------------------------------------
; CAD:   (1.000, 1.000) -> (2.000, 1.000)
; ROBOT: (1.000, 1.000) -> (2.000, 1.000) | Ang: 0.00
M5 T1000
G2 X1.000 Y1.000 F0.10 ; Center to Start
G4 P200
G1 A0.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X1.955 Y1.000 F0.10

; ---------------------------------------------------
; CAD:   (2.000, 1.000) -> (2.000, 2.000)
; ROBOT: (2.000, 1.000) -> (2.000, 2.000) | Ang: 90.00
M5 T1000
G2 X2.000 Y1.000 F0.10 ; Center to Start
G4 P200
G1 A90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X2.000 Y1.955 F0.10

; ---------------------------------------------------
; CAD:   (2.000, 2.000) -> (1.000, 2.000)
; ROBOT: (2.000, 2.000) -> (1.000, 2.000) | Ang: 180.00
M5 T1000
G2 X2.000 Y2.000 F0.10 ; Center to Start
G4 P200
G1 A180.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X1.045 Y2.000 F0.10

; ---------------------------------------------------
; CAD:   (1.000, 2.000) -> (1.000, 1.000)
; ROBOT: (1.000, 2.000) -> (1.000, 1.000) | Ang: -90.00
M5 T1000
G2 X1.000 Y2.000 F0.10 ; Center to Start
G4 P200
G1 A-90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X1.000 Y1.045 F0.10


; === END PROGRAM (Final commands before Input backup) ===
M5 T1000
G1 A0.000 F0.05 ; Final rotation to 0 degrees (Safe finish in place)
G2 L0.20 F-0.10 ; Back up

; === ORIGINAL INPUT LINES START (Input backup) ===
; kvadrat.gcode; Имя выходного файла
; 0.0 0.0 0.0 ; Смещение системы координат робота от системы координат чертежа. Где стоит ноль робота на чертеже.
; 
; 1.0 1.0 2.0 1.0
; 2.0 1.0 2.0 2.0
; 2.0 2.0 1.0 2.0
; 1.0 2.0 1.0 1.0
; === ORIGINAL INPUT LINES END ===
