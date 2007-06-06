CODEVISION PROGRAMMING
MCU set to 4MHz
no fuses set

CODE: 
speed.c (main)
HEADER FILES:
keypad.h
index.h

to adjust speed:
there is a constant (Kp) near the top of the .c file: increase to increase speed, decrease to decrease speed

TWI(for other mcu): 
index = P: keypad: sends back an index of P and the number pressed
index = S: speed

CAMERA: if its not working properly
comment out the processcamera() in the while(1) main loop
change it to g_speed = 4? 3? whatever (1-4)

HARDWARE:
speed encoder:
pe6 should be either the a or b input(the small, frequent ticks)
the other pins are not implemented
