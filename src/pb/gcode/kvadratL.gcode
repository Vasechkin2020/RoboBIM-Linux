; === GENERATED G-CODE (v3.6) ===
; Output Filename: kvadratL.gcode
; Robot Position on Map: X=1.000, Y=2.000, Angle=0.0
; Marker Offset: 0.0450
M5 T1000 ; Pen UP
G1 A0.000 F0.05
G2 L0.045 F-0.10 ; Initial align

; ---------------------------------------------------
; CAD:   (1.000, 2.000) -> (2.000, 2.000)
; ROBOT: (0.000, 0.000) -> (1.000, 0.000) | Ang: 0.00
M5 T1000
G2 X0.000 Y0.000 F0.10 ; Center to Start
G4 P200
G1 A0.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X0.955 Y0.000 F0.10

; ---------------------------------------------------
; CAD:   (2.000, 2.000) -> (2.000, 3.000)
; ROBOT: (1.000, 0.000) -> (1.000, 1.000) | Ang: 90.00
M5 T1000
G2 X1.000 Y0.000 F0.10 ; Center to Start
G4 P200
G1 A90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X1.000 Y0.955 F0.10

; ---------------------------------------------------
; CAD:   (2.000, 3.000) -> (1.000, 3.000)
; ROBOT: (1.000, 1.000) -> (0.000, 1.000) | Ang: 180.00
M5 T1000
G2 X1.000 Y1.000 F0.10 ; Center to Start
G4 P200
G1 A180.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X0.045 Y1.000 F0.10

; ---------------------------------------------------
; CAD:   (1.000, 3.000) -> (1.000, 2.000)
; ROBOT: (0.000, 1.000) -> (0.000, 0.000) | Ang: -90.00
M5 T1000
G2 X0.000 Y1.000 F0.10 ; Center to Start
G4 P200
G1 A-90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
M3 T1000
G2 X-0.000 Y0.045 F0.10


; === END PROGRAM (Final commands before Input backup) ===
M5 T1000
G1 A0.000 F0.05 ; Final rotation to 0 degrees (Safe finish in place)

; === ORIGINAL INPUT LINES START (Input backup) ===
; kvadratL.gcode; Имя выходного файла
; 1.0 2.0 0.0 ; Смещение системы координат робота от системы координат чертежа. Где стоит ноль робота на чертеже.
; 1.0 2.0 2.0 2.0
; 2.0 2.0 2.0 3.0
; 2.0 3.0 1.0 3.0
; 1.0 3.0 1.0 2.0
; === ORIGINAL INPUT LINES END ===
