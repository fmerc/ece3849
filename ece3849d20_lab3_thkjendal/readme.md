# ECE 3849 Lab 3: Advanced I/O
Tai Kjendal, thkjendal@wpi.edu\
April 7, 2020

Adds a frequency counter and audio output functionality to the Lab 2 oscilloscope.
The ADC I/O has also been optimized using DMA, and bumped to a 2 Msps sampling rate.
To read PWM output into ADC1 input, connect Pin PF_2 to Pin AIN3 (GPIO PE_0).
To measure frequency, connect Pin PF_3 to Pin SSI2 (GPIO PD_0).
Time scale, voltage scale, and trigger slope are shown at the top of the LCD. 
PWM period, frequency, and CPU load are displayed at the bottom

Board Functions:
- USR_SW1: decrease PWM signal source period
- USR_SW2: increase PWM signal source period
- S1: increment voltage scale
- S2: decrement voltage scale
- JSEL: play audio waveform