/*
 * sampling.c
 *
 *  Created on: Apr 3, 2020
 *      Author: Tai Kjendal
 */

/* XDCtools Header files */
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdbool.h>
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

// KISS FFT
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#define PI 3.14159265358979f
#define NFFT 1024         // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))
volatile bool spectrumMode = false;

// ADC Sampling
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;     // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];              // circular buffer
volatile uint32_t gADCErrors = 0;                           // number of missed ADC deadlines
volatile uint16_t samples[NFFT];
volatile uint16_t processedBuffer[ADC_TRIGGER_SIZE];
volatile bool trigState = true; // 2 trigger states

volatile uint32_t trigger;
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2}; // voltage scale/div values
float fScale;
volatile int vState = 4;     // 5 voltage scale states

/* Initialize ADC handling hardware (ADC1 sequence 0) */
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
}

/* Find the edge of the waveform */
int RisingTrigger(void) {
    // Step 1
    int index = gADCBufferIndex - LCD_HORIZONTAL_MAX/2;
    int iStop = index - ADC_BUFFER_SIZE/2;

    // Step 2
    switch(trigState) {
    case 0: // positive edge trigger
        for (; index > iStop; index--) {
            if (    gADCBuffer[ADC_BUFFER_WRAP(index)] <= ADC_OFFSET &&
                    gADCBuffer[ADC_BUFFER_WRAP(index+1)] > ADC_OFFSET)
                break;
        }
        break;

    case 1: // falling edge trigger
        iStop = index + ADC_BUFFER_SIZE/2;
        for (; index < iStop; index++) {
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

/* Initialize PWM signal source */
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

/* Configure Timer3 for CPU measurement */
void cpu_clock_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, (gSystemClock/50) - 1);
}

/* Count CPU cycles */
uint32_t cpu_load_count(void) {
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A);  // start timer in one-shot mode
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

/* ISR for ADC sampling */
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


/* Waveform Task: create a waveform to display (high priority) */
void waveformTask_func(UArg arg1, UArg arg2) {
    IntMasterEnable();

    while (true) {
        Semaphore_pend(semWaveform, BIOS_WAIT_FOREVER);

        if (spectrumMode) {
            int i;
            for (i = 0; i < NFFT; i++) {
                samples[i] = gADCBuffer[ADC_BUFFER_WRAP(gADCBufferIndex - NFFT + i)];
            }
        }
        else
            trigger = RisingTrigger(); // search for trigger

        Semaphore_post(semProcessing);
    }
}

/* Processing Task: process stuff (lowest priority) */
void processingTask_func(UArg arg1, UArg arg2) {
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg;                        // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    static float w[NFFT]; // window function
    for (i = 0; i < NFFT; i++) {
        // Blackman window
        w[i] = 0.42f - 0.5f * cosf(2*PI*i/(NFFT-1)) + 0.08f * cosf(4*PI*i/(NFFT-1));
    }

    while (true) {
        Semaphore_pend(semProcessing, BIOS_WAIT_FOREVER);

        if (spectrumMode) {
            fScale = 0.7;
            for (i = 0; i < NFFT; i++) {        // generate an input waveform
                in[i].r = (float)samples[i] * w[i];  // real part of waveform
                in[i].i = 0;                    // imaginary part of waveform
            }

            kiss_fft(cfg, in, out);         // compute FFT

            // convert first 128 bins of out[] to dB for display
            for (i = 0; i < ADC_TRIGGER_SIZE-1; i++) {
                processedBuffer[i] = 160 - roundf(10 * log10f(out[i].r * out[i].r + out[i].i * out[i].i));
            }
        }

        else {
            fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[vState]);

            for (i = 0; i < ADC_TRIGGER_SIZE-1; i++) {// read gADCBuffer measurements into processedBuf
                processedBuffer[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger - LCD_HORIZONTAL_MAX/2) + i];
            }
        }

        Semaphore_post(semWaveform);
        Semaphore_post(semDisplay);
    }
}
