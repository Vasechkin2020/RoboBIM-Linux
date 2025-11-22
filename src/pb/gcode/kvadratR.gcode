; === GENERATED G-CODE (Debug Version) ===
; Output Filename: krvadratR.gcode
; Marker Offset: 0.0450
M5 T1000 ; Pen UP
G1 A0.000 F0.05
G2 L0.045 F-0.10 ; Initial align

; >>> Line: (0.000, 0.000) -> (1.000, 0.000) | Angle: 0.00
M5 T1000 ; Pen UP
G2 X0.000 Y0.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A0.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(1.000,0.000) - Off(0.045,0.000) = C(0.955,0.000)
M3 T1000 ; Pen DOWN
G2 X0.955 Y0.000 F0.10

; >>> Line: (1.000, 0.000) -> (1.000, -1.000) | Angle: -90.00
M5 T1000 ; Pen UP
G2 X1.000 Y0.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A-90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(1.000,-1.000) - Off(0.000,-0.045) = C(1.000,-0.955)
M3 T1000 ; Pen DOWN
G2 X1.000 Y-0.955 F0.10

; >>> Line: (1.000, -1.000) -> (0.000, -1.000) | Angle: 180.00
M5 T1000 ; Pen UP
G2 X1.000 Y-1.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A180.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(0.000,-1.000) - Off(-0.045,0.000) = C(0.045,-1.000)
M3 T1000 ; Pen DOWN
G2 X0.045 Y-1.000 F0.10

; >>> Line: (0.000, -1.000) -> (0.000, 0.000) | Angle: 90.00
M5 T1000 ; Pen UP
G2 X0.000 Y-1.000 F0.10 ; Center to Start Vertex
G4 P200
G1 A90.000 F0.05
G4 P200
G2 L0.045 F-0.10 ; Back up
; Math: V(0.000,0.000) - Off(0.000,0.045) = C(-0.000,-0.045)
M3 T1000 ; Pen DOWN
G2 X-0.000 Y-0.045 F0.10


; === ORIGINAL INPUT LINES START ===
; kvadratR.gcode
; 0.0 0.0 1.0 0.0
; 1.0 0.0 1.0 -1.0
; 1.0 -1.0 0.0 -1.0
; 0.0 -1.0 0.0 0.0
; === ORIGINAL INPUT LINES END ===

; === END PROGRAM ===
M5 T1000
G2 X0.000 Y0.000 F0.10 ; Park
