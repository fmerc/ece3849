/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"

/* Lab 1 header files */
#include <stdio.h>
#include <math.h>
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sampling.h"
#include "buttons.h"

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

volatile uint32_t trigger;
float fScale;
volatile int vState = 4;     // 5 voltage scale states
volatile int tState = 11;    // 12 time scale states

// Voltage/time scale constants
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", " 1 V", " 2 V"};
const char * const gTimeScaleStr[] = {"100 ms", "50 ms", "20 ms", "10 ms", "5 ms", "2 ms", "1 ms", "500 us", "200 us", "100 us", "50 us", "20 us"};
const char * const gTriggerStr[] = {"+|", "|-"};

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0;

tContext sContext;

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // CPU measurement
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
//    TimerDisable(TIMER3_BASE, TIMER_BOTH);
//    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
//    TimerLoadSet(TIMER3_BASE, TIMER_A, (gSystemClock/50) - 1);

    // Initialize LCD
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    GrContextInit(&sContext, &g_sCrystalfontz128x128);
    GrContextFontSet(&sContext, &g_sFontFixed6x8);

    // Hardware init
//    ButtonInit();       // Initialize buttons
//    ADC_Init();         // Initialize ADC1 sequence 0
    signal_init();      // Initialize PWM signal source

    count_unloaded = cpu_load_count();  // measure cpu

    /* Start BIOS */
    BIOS_start();

    return (0);
}

// Priority 15, Waveform task creates waveform to display
void waveformTask_func(UArg arg1, UArg arg2) {
    IntMasterEnable();

    while (true) {
        Semaphore_pend(semWaveform, BIOS_WAIT_FOREVER);
        trigger = RisingTrigger(); // search for trigger
        Semaphore_post(semProcessing);
    }
}

// Priority 2, Processing task
void processingTask_func(UArg arg1, UArg arg2) {
    float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2}; // voltage scale/div values
    int i;

    while (true) {
        Semaphore_pend(semProcessing, BIOS_WAIT_FOREVER);

        fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[vState]);
        for (i = 0; i < ADC_TRIGGER_SIZE-1; i++) // read gADCBuffer measurements into samples[]
            samples[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger - LCD_HORIZONTAL_MAX/2) + i];

        Semaphore_post(semWaveform);
        Semaphore_post(semDisplay);
    }
}

// Priority 1, Display task waveform to LCD
void displayTask_func(UArg arg1, UArg arg2) {
    char str[50];       // string buffer
    int i;
    int x, y, y2;

    while (true) {
        Semaphore_pend(semDisplay, BIOS_WAIT_FOREVER);

        // CPU load
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded;

        // Fill screen with black rectangle
        tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // Draw grid
        GrContextForegroundSet(&sContext, ClrBlue);
        for (i = 1; i < 128; i+=21){
            GrLineDraw(&sContext, i, 0, i, 128);
            GrLineDraw(&sContext, 0, i, 128, i);
        }
        GrContextForegroundSet(&sContext, ClrRed);
        GrLineDraw(&sContext, 64, 0, 64, 128);
        GrLineDraw(&sContext, 0, 64, 128, 64);

        // Draw waveform
        GrContextForegroundSet(&sContext, ClrYellow);

        y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(samples[0] - ADC_OFFSET)));

        for (x = 1; x < LCD_HORIZONTAL_MAX - 1; x++) {
            y2 = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(samples[x] - ADC_OFFSET)));
            GrLineDraw(&sContext, x-1, y, x, y2);
            y = y2;
        }

        // Display time scale
        GrContextForegroundSet(&sContext, ClrWhite); // white text
        snprintf(str, sizeof(str), gTimeScaleStr[tState]);
        GrStringDraw(&sContext, str, -1, 0, 0, false);

        // Display voltage scale
        snprintf(str, sizeof(str), gVoltageScaleStr[vState]);
        GrStringDraw(&sContext, str, -1, 50, 0, false);

        // Display trigger
        snprintf(str, sizeof(str), gTriggerStr[trigState]);
        GrStringDraw(&sContext, str, -1, 110, 0, false);

        // Display CPU load %
        snprintf(str, sizeof(str), "CPU load = %.1f%%", cpu_load*100);
        GrStringDraw(&sContext, str, -1, 0, 120, false);

        GrFlush(&sContext); // flush the frame buffer to the LCD

        Semaphore_post(semWaveform);
    }
}

void task0_func(UArg arg1, UArg arg2)
{
    IntMasterEnable();

    while (true) {
/*

        if (fifo_get(buttons)) {
            for (i=0; i<10; i++) {
                if (buttons[i]==('i') && gButtons == 4) { // increment voltage
                    vState = (++vState) % 5;
                } else if (buttons[i]==('j') && gButtons == 8) { // decrement voltage
                    vState = (vState <= 0) ? 4 : vState - 1;
                } else if (buttons[i]==('t') && gButtons == 16) { // change trigger
                    trigState = !trigState;
                } else if (buttons[i]==('u') && gButtons == 2) { // increment time scale
                    tState = (++tState) % 12;
                } else if (buttons[i]==('v') && gButtons == 1) { // decrement time scale
                    tState = (tState <= 0) ? 11 : tState - 1;
                }
            }
        }

*/
    }
}
