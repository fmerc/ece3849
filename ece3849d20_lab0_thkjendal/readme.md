# ECE 3849 Lab 0: Buttons, Timers, LCDs, and Interrupts
Tai Kjendal, thkjendal@wpi.edu\

Implements a simple timer on the LCD, demonstrating button, interrupt, and clock functionalities.
Time is displayed in mm:ss:ms. The binary value underneath shows the status of the buttons
(1 = pressed, 0 = unpressed).

Board Functions:
- USR_SW1: pause the timer
- USR_SW2: reset the timer to zero
