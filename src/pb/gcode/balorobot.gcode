; === GENERATED G-CODE (Debug Version) ===
; Output Filename: balorobot.gcode
; Marker Offset: 0.0450
M5 T1000 ; Pen UP
G1 A0.000 F0.05
G2 L0.045 F-0.10 ; Initial align

; >>> Line: (0.000, 0.000) -> (2.000, 0.000) | Angle: 0.00
M5 T1000 ; Pen UP
G2 X0.000 Y0.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A0.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(2.000,0.000) - Off(0.045,0.000) = C(1.955,0.000)
M3 T1000 ; Pen DOWN
G2 X1.955 Y0.000 F0.10

; >>> Line: (2.000, 0.000) -> (2.000, 1.000) | Angle: 90.00
M5 T1000 ; Pen UP
G2 X2.000 Y0.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(2.000,1.000) - Off(0.000,0.045) = C(2.000,0.955)
M3 T1000 ; Pen DOWN
G2 X2.000 Y0.955 F0.10

; >>> Line: (2.000, 1.000) -> (0.000, 0.000) | Angle: -153.43
M5 T1000 ; Pen UP
G2 X2.000 Y1.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A-153.435 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(0.000,0.000) - Off(-0.040,-0.020) = C(0.040,0.020)
M3 T1000 ; Pen DOWN
G2 X0.040 Y0.020 F0.10

; >>> Line: (0.000, 0.000) -> (0.000, 1.000) | Angle: 90.00
M5 T1000 ; Pen UP
G2 X0.000 Y0.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(0.000,1.000) - Off(0.000,0.045) = C(-0.000,0.955)
M3 T1000 ; Pen DOWN
G2 X-0.000 Y0.955 F0.10

; >>> Line: (0.000, 1.000) -> (2.000, 0.000) | Angle: -26.57
M5 T1000 ; Pen UP
G2 X0.000 Y1.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A-26.565 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(2.000,0.000) - Off(0.040,-0.020) = C(1.960,0.020)
M3 T1000 ; Pen DOWN
G2 X1.960 Y0.020 F0.10

; >>> Line: (2.000, 0.000) -> (1.000, 1.000) | Angle: 135.00
M5 T1000 ; Pen UP
G2 X2.000 Y0.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A135.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(1.000,1.000) - Off(-0.032,0.032) = C(1.032,0.968)
M3 T1000 ; Pen DOWN
G2 X1.032 Y0.968 F0.10

; >>> Line: (1.000, 1.000) -> (0.000, 0.000) | Angle: -135.00
M5 T1000 ; Pen UP
G2 X1.000 Y1.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A-135.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(0.000,0.000) - Off(-0.032,-0.032) = C(0.032,0.032)
M3 T1000 ; Pen DOWN
G2 X0.032 Y0.032 F0.10


; === ORIGINAL INPUT LINES START ===
; balorobot.gcode
; 0.0 0.0 2.0 0.0
; 2.0 0.0 2.0 1.0
; 2.0 1.0 0.0 0.0
; 0.0 0.0 0.0 1.0
; 0.0 1.0 2.0 0.0
; 2.0 0.0 1.0 1.0
; 1.0 1.0 0.0 0.0
; === ORIGINAL INPUT LINES END ===

; === END PROGRAM ===
M5 T1000
G2 X0.000 Y0.000 F0.10 ; Park
