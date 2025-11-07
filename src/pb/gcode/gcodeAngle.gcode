; Initialize program
;G90 ; Set absolute coordinate mode

G2 L1.0 F0.2 ; Linear move 1.0 m
G4 P3000 ; Pause for ... ms 
;G2 L1.0 F0.2 ; Linear move 1.0 m
;G4 P3000 ; Pause for ... ms 
;G2 L1.0 F0.2 ; Linear move 1.0 m
;G4 P3000 ; Pause for ... ms 
;G2 L1.0 F0.2 ; Linear move 1.0 m
;G4 P3000 ; Pause for ... ms 

;G1 A150.0 F0.2 ; Rotate to angle 90.0°
;G4 P2000 ; Pause for ... ms 
;G1 A0.0 F0.2 ; Rotate to angle 90.0°
;G4 P2000 ; Pause for ... ms 
;G1 A-150.0 F0.2 ; Rotate to angle 90.0°
;G4 P2000 ; Pause for ... ms 
;G1 A0.0 F0.2 ; Rotate to angle 90.0°
;G4 P2000 ; Pause for ... ms 
;G2 L1.0 F0.2 ; Linear move 2.0 m
;G4 P1000 ; Pause for ... ms 
;G1 A180.0 F0.2 ; Rotate to angle 90.0°
;G4 P1000 ; Pause for ... ms 
;G2 L1.0 F0.2 ; Linear move 2.0 m
;G4 P1000 ; Pause for ... ms 
;G1 A-90.0 F0.2 ; Rotate to angle 90.0°
;G4 P1000 ; Pause for ... ms 
;G2 L1.0 F0.2 ; Linear move 2.0 m
;G4 P1000 ; Pause for ... ms 
;G1 A0.0 F0.2 ; Rotate to angle 90.0°
;G4 P1000 ; Pause for ... ms 

;G2 L2.0 F0.1 ; Linear move 2.0 m
;G4 P1000 ; Pause for ... ms 
;G0 F_L0.1 F_R0.1 T3000 ; Linear move 
;G4 P1000 ; Pause for ... ms 

;G9 ; Cicle...