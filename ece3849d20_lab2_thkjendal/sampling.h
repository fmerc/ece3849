/*
 * sampling.h
 *
 *  Created on: Apr 3, 2020
 *      Author: Tai Kjendal
 */

#ifndef SAMPLING_H_
#define SAMPLING_H_

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

#define ADC_BUFFER_SIZE 2048    // size must be a power of 2
#define ADC_TRIGGER_SIZE 128
#define ADC_OFFSET       128
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))    // index wrapping macro

#define VIN_RANGE 3.3       // global voltage input range
#define PIXELS_PER_DIV 20   // determines the pixel per division on the oscilloscope
#define ADC_BITS 12         // the ADC has 12 bits

extern uint32_t gSystemClock;                           // [Hz] system clock frequency
extern volatile int32_t gADCBufferIndex;                // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
extern volatile uint32_t gADCErrors;                    // number of missed ADC deadlines

extern volatile bool spectrumMode;  // whether device is in square mode or spectrum mode
extern volatile uint16_t samples[ADC_TRIGGER_SIZE]; // sample to show on LCD screen
extern volatile bool trigState; // 2 trigger states

extern float fScale;
extern volatile int vState;

// Initialize ADC handling hardware
void ADC_Init(void);
int RisingTrigger(void);
void signal_init(void);
uint32_t cpu_load_count(void);

#endif /* SAMPLING_H_ */
