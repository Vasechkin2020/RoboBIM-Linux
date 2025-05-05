; Initialize program
G90 ; Set absolute coordinate mode
M3 ; Turn on print module
G0 X0 Y0 A0 ; Rapid move to starting point (0, 0), angle 0°

; Main envelope trajectory
G1 A0 F100 ; Rotate to angle 0° for next move
G1 X1000 Y0 F100 ; Linear move up to (1000, 0)
G1 A90 F100 ; Rotate to angle 90° for next move
G1 X1000 Y500 F100 ; Linear move left to (1000, 500)
G1 A153.43 F100 ; Rotate to angle 153.43° for next move
G1 X500 Y750 F100 ; Linear move to flap vertex (500, 750)
G1 A-153.43 F100 ; Rotate to angle -153.43° for next move
G1 X0 Y500 F100 ; Linear move to left corner (0, 500)
G1 A-90 F100 ; Rotate to angle -90° for next move
G1 X0 Y0 F100 ; Linear move back to start (0, 0)

; First diagonal
G1 A26.57 F100 ; Rotate to angle 26.57° for first diagonal
G1 X1000 Y500 F100 ; Linear move along first diagonal to (1000, 500)

; Second diagonal
G0 X0 Y0 A0 ; Rapid move back to (0, 0) for second diagonal, angle 0°
G1 A0 F100 ; Rotate to angle 0° for next move
G1 X1000 Y0 F100 ; Linear move up to (1000, 0)
G1 A153.43 F100 ; Rotate to angle 153.43° for second diagonal
G1 X0 Y500 F100 ; Linear move along second diagonal to (0, 500)

; Complete program
G4 P1000 ; Pause for 1000 ms before completion
M5 ; Turn off print module
G28 ; Return to home position (0, 0)