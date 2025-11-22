; === GENERATED G-CODE ===
; Marker Offset: 0.045
M5 T1000 ; Start with pen UP
G1 A0.000 F0.05
G2 L0.045 F-0.10 ; Initial align to 0,0

; --- Processing Line: (0.000, 0.000) -> (2.000, 0.000) ---
M5 T1000 ; Pen UP
G2 X0.000 Y0.000 F0.10 ; Move Center to Start Vertex
G4 P200
G1 A0.000 F0.05 ; Turn to Line Angle
G4 P200
G2 L0.045 F-0.10 ; Back up to place Marker at Start
M3 T1000 ; Pen DOWN
G2 X1.955 Y0.000 F0.10 ; Draw Line

; --- Processing Line: (2.000, 0.000) -> (2.000, 1.000) ---
M5 T1000 ; Pen UP
G2 X2.000 Y0.000 F0.10 ; Move Center to Start Vertex
G4 P200
G1 A90.000 F0.05 ; Turn to Line Angle
G4 P200
G2 L0.045 F-0.10 ; Back up to place Marker at Start
M3 T1000 ; Pen DOWN
G2 X2.000 Y0.955 F0.10 ; Draw Line

; --- Processing Line: (2.000, 1.000) -> (0.000, 0.000) ---
M5 T1000 ; Pen UP
G2 X2.000 Y1.000 F0.10 ; Move Center to Start Vertex
G4 P200
G1 A-153.435 F0.05 ; Turn to Line Angle
G4 P200
G2 L0.045 F-0.10 ; Back up to place Marker at Start
M3 T1000 ; Pen DOWN
G2 X0.040 Y0.020 F0.10 ; Draw Line


; === END OF PROGRAM ===
M5 T1000
G2 X0.000 Y0.000 F0.10 ; Park Center at 0,0
