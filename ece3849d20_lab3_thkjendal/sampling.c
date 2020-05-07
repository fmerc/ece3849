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
#include "driverlib/udma.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sysctl_pll.h"

#include "buttons.h"
#include "sampling.h"
#include "audio_waveform.h"

// KISS FFT
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#define PI 3.14159265358979f
#define NFFT 1024         // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))
volatile bool spectrumMode = false;

// ADC Sampling
//volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;     // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];              // circular buffer
volatile uint32_t gADCErrors = 0;                           // number of missed ADC deadlines
volatile uint16_t samples[NFFT];
volatile uint16_t processedBuffer[ADC_TRIGGER_SIZE];
volatile bool trigState = true;

// Time Capture
uint32_t prevCount = 0;
uint32_t timerPeriod = 0;
uint32_t countInterval = 0;
uint32_t countPeriod = 0;
float avgFrequency = 0;

// DMA init
#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64];     // uDMA control table (global)

volatile bool gDMAPrimary = true;   // is DMA occurring in the primary channel?
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2}; // voltage scale/div values
volatile uint32_t trigger;
volatile int vState = 4;
volatile int pwmPeriod = 6000;
float fScale;

// audio
uint32_t gPWMSample = 0;            // PWM sample counter
uint32_t gSamplingRateDivider = 29; // sampling rate divider

/* Configure timer for frequency counter */
void FreqTimer(void) {
    // configure GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);     // use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);   // use maximum prescale value
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);  // 'CaptureA event interrupt' in timer.h
    TimerEnable(TIMER0_BASE, TIMER_A);
}

/* Initialize ADC & DMA handling hardware (ADC1 sequence) */
void DMA_Init(void) {
    // ADC config
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

    // DMA config
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);

    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);

    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);

    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);

    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

    ADCSequenceDMAEnable(ADC1_BASE, 0);         // enable DMA for ADC1 sequence 0
    ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt
    ADCSequenceEnable(ADC1_BASE, 0);    // enable the sequence.  it is now sampling
}

/* Initialize PWM signal source */
void PWM_Init(void) {
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
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwmPeriod);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)pwmPeriod*0.4f));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)pwmPeriod*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);


    // configure M0PWM5, at GPIO PG1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, gen 2, output 5
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PWM_PERIOD/2);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

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

/* Get the index for gADCBuffer */
int32_t getADCBufferIndex(void) {
    int32_t index;
    IArg key;
    key = GateHwi_enter(gateHwi0);
    if (gDMAPrimary) {  // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
    }
    else {              // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
    }
    GateHwi_leave(gateHwi0, key);
    return index;
}

/* Find the edge of the waveform */
int RisingTrigger(void) {
    // Step 1
    int index = getADCBufferIndex();
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
        index = getADCBufferIndex() - LCD_HORIZONTAL_MAX/2;     // set back to previous value

    return index;
}

/* ISR for ADC sampling */
void ADC_ISR(void) {    // DMA (lab 3)
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0);  // clear the ADC1 sequence 0 DMA interrupt flag

    // Check the primary DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
        // restart the primary channel (same as setup)
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
                               (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
        gDMAPrimary = false;    // DMA is currently occurring in the alternate buffer
    }

    // Check the alternate DMA channel for end of transfer, and restart if needed.
    // Also set the gDMAPrimary global.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) {
        // restart the primary channel (same as setup)
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
                               (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
        gDMAPrimary = true;    // DMA is currently occurring in the alternate buffer
    }

    // The DMA channel may be disabled if the CPU is paused by the debugger.
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);  // re-enable the DMA channel
    }
}

/* ISR for for timer capture */
void Capture_ISR(void) {
    TIMER0_ICR_R = TIMER_ICR_CAECINT;   // clear interrupt

    uint32_t count = TimerValueGet(TIMER0_BASE, TIMER_A);   // read 24-bit
    timerPeriod = (count - prevCount) & 0xFFFFFF;           // set timerPeriod
    prevCount = count;                                      // update prevCount

    countPeriod++;            // track # of periods
    countInterval += timerPeriod;  // track interval of countPeriod
}

/* ISR for PWM audio waveform */
void PWM_ISR(void)
{
    PWM0_ISC_R = PWM_0_ISC_INTCNTZERO;  // clear PWM interrupt flag

    int i = (gPWMSample++) / gSamplingRateDivider; // waveform sample index
    PWM0_2_CMPB_R = 1 + gWaveform[i]; // write directly to the PWM compare B register
    if (i >= gWaveformSize) { // if at the end of the waveform array
        PWMIntDisable(PWM0_BASE, PWM_INT_GEN_2); // disable these interrupts
        gPWMSample = 0; // reset sample index so the waveform starts from the beginning
    }
}

/* Waveform Task: create a waveform to display (high priority) */
void waveformTask_func(UArg arg1, UArg arg2) {
    IntMasterEnable();

    while (true) {
        Semaphore_pend(semWaveform, BIOS_WAIT_FOREVER);

        if (spectrumMode) {
            int i;
            int index = getADCBufferIndex();
            for (i = 0; i < NFFT; i++) {
                samples[i] = gADCBuffer[ADC_BUFFER_WRAP(index - NFFT + i)];
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
                processedBuffer[i] = 180 - roundf(10 * log10f(out[i].r * out[i].r + out[i].i * out[i].i));
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

/* Frequency Task: calculate frequency capture */
void frequencyTask_func(UArg arg1, UArg arg2) {
    IArg key;
    uint32_t count, interval;

    while (true) {
        Semaphore_pend(semFrequency, BIOS_WAIT_FOREVER);

        key = GateHwi_enter(gateHwi0);
        interval = countInterval;
        count = countPeriod;
        countInterval = 0;
        countPeriod = 0;
        GateHwi_leave(gateHwi0, key);

        float avgPeriod = (float)interval / count;
        avgFrequency = (float)gSystemClock / avgPeriod;
    }
}

/* Check frequency (run frequencyTask) periodically */
void freq_func(UArg arg) {
    Semaphore_post(semFrequency);
}
