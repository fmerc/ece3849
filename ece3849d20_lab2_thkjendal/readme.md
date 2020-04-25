# ECE 3849 Lab 2: RTOS, Spectrum Analyzer
Tai Kjendal, thkjendal@wpi.edu\
May 16, 2020

To read PWM output into ADC1 input, connect Pin PF_2 to Pin AIN3 (GPIO PE_0).
Time scale, voltage scale, and trigger slope are shown at the top of the LCD. 
CPU load is shown at the bottom. Spectrum Mode (FFT) is a simple spectrum analyzer.

Board Functions:
- USR_SW1: toggle FFT Mode
- USR_SW2: toggle Rising/Falling edge trigger
- S1: increment voltage scale
- S2: decrement voltage scale