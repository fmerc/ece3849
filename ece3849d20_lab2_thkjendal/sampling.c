/*
 * sampling.c
 *
 *  Created on: Apr 3, 2020
 *      Author: Tai Kjendal
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sysctl_pll.h"
#include "buttons.h"
#include "sampling.h"

volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;     // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];              // circular buffer
volatile uint32_t gADCErrors = 0;                           // number of missed ADC deadlines
volatile uint16_t samples[ADC_TRIGGER_SIZE];
volatile bool trigState = true; // 2 trigger states

// Initialize ADC handling hardware
void ADC_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    GPIOPinTypeADC(GPIO_PORTP_BASE, GPIO_PIN_0);    // GPIO setup for analog input AIN3

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);     // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0);                           // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);  // specify the "Always" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END |ADC_CTL_CH3);   // in the 0th step, sample channel 3 (AIN3)
                                                                // enable interrupt, and make it the end of sequence
    ADCIntEnable(ADC1_BASE, 0);         // enable sequence 0 interrupt in the ADC1 peripheral
    ADCSequenceEnable(ADC1_BASE, 0);    // enable the sequence.  it is now sampling
//    IntPrioritySet(INT_ADC1SS0, 0);     // set ADC1 sequence 0 interrupt priority
//    IntEnable(INT_ADC1SS0);             // enable ADC1 sequence 0 interrupt in int. controller
}

int RisingTrigger(void) {   // search for edge
    // Step 1
    int index = gADCBufferIndex - LCD_HORIZONTAL_MAX/2;
    int iStop = index - ADC_BUFFER_SIZE/2;

    // Step 2
    switch(trigState) {
    case 0: // positive edge trigger
        for (index; index > iStop; index--) {
            if (    gADCBuffer[ADC_BUFFER_WRAP(index)] <= ADC_OFFSET &&
                    gADCBuffer[ADC_BUFFER_WRAP(index+1)] > ADC_OFFSET)
                break;
        }
        break;

    case 1: // falling edge trigger
        iStop = index + ADC_BUFFER_SIZE/2;
        for (index; index < iStop; index++) {
            if (    gADCBuffer[ADC_BUFFER_WRAP(index)] >= ADC_OFFSET &&
                    gADCBuffer[ADC_BUFFER_WRAP(index+1)] < ADC_OFFSET)
                break;
        }
    }

    // Step 3
    if (index == iStop)    // for loop ran to the end
        index = gADCBufferIndex - LCD_HORIZONTAL_MAX/2;     // set back to previous value

    return index;
}


void signal_init(void) {
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    // configure M0PWM3, at GPIO PF3, BoosterPack 1 header C1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

uint32_t cpu_load_count(void) {
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A);  // start timer in one-shot mode
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

// ISR for sampling ADC
void ADC_ISR(void) {
    ADC1_ISC_R = ADC_ISC_IN0;   // clear ADC1 sequence0 interrupt flag in the ADCISC register [datasheet pg 1085]

    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) {     // check for ADC FIFO overflow
        gADCErrors++;                       // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;       // clear overflow condition
    }

    gADCBuffer[
               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
               ] = ADC1_SSFIFO0_R;      // read sample from the ADC1 sequence 0 FIFO [datasheet pg 1111]
}
