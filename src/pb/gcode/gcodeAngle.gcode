; Initialize program
G90 ; Set absolute coordinate mode

G0 F_L0.1 F_R0.1 T3000 ; Linear move 
G4 P1000 ; Pause for ... ms 
G2 L2.0 F0.1 ; Linear move 2.0 m
G4 P1000 ; Pause for ... ms 
;G1 A90.0 F0.1 ; Rotate to angle 90.0°
;G4 P1000 ; Pause for ... ms 

;G9 ; Cicle...