/**
 * main.c
 *
 * ECE 3849 Lab 0 Starter Project
 * Gene Bogdanov    10/18/2017
 *
 * This version is using the new hardware for B2017: the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "sampling.h"

extern volatile uint32_t gButtons;
extern volatile int32_t gADCBufferIndex;

uint32_t gSystemClock;          // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", " 1 V", " 2 V"};
const char * const gTimeScaleStr[] = {"100 ms", "50 ms", "20 ms", "10 ms", "5 ms", "2 ms", "1 ms", "500 us", "200 us", "100 us", "50 us", "20 us"};
const char * const gTriggerStr[] = {"+|", "|-"};

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0;

int main(void)
{
    IntMasterDisable();

    // Local variables
    char str[50];       // string buffer
    char buttons[10];   // button presses
    float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2}; // voltage scale/div values
    int vState = 4;     // 5 voltage scale states
    int tState = 11;    // 12 time scale states

    int i;
    int x, y, y2;   // for drawing stuff

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Initialize LCD
    Crystalfontz128x128_Init();                             // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128);  // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8);      // select font

    // Initialize hardware
    ButtonInit();       // Initialize buttons
    ADC_Init();         // Initialize ADC1 sequence 0
    signal_init();      // Initialize PWM signal source

    // Timer for CPU measurement
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, (gSystemClock/50) - 1);

    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    count_unloaded = cpu_load_count();  // get unloaded CPU count

    IntMasterEnable();  // Enable global interrupts

    while (true) {
        // compute CPU load
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded;

        // fill screen with black rectangle
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // draw grid
        GrContextForegroundSet(&sContext, ClrBlue);
        for (i = 1; i < 128; i+=21){
            GrLineDraw(&sContext, i, 0, i, 128);
            GrLineDraw(&sContext, 0, i, 128, i);
        }
        GrContextForegroundSet(&sContext, ClrRed);
        GrLineDraw(&sContext, 64, 0, 64, 128);
        GrLineDraw(&sContext, 0, 64, 128, 64);

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

        // Draw wave
        GrContextForegroundSet(&sContext, ClrYellow);
        int trigger = RisingTrigger(); // search for trigger
        float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[vState]);

        for (i=0; i < ADC_TRIGGER_SIZE-1; i++) // read gADCBuffer measurements into sample[]
            sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger - LCD_HORIZONTAL_MAX/2) + i];

        y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(sample[0] - ADC_OFFSET)));
        for (x = 1; x < LCD_HORIZONTAL_MAX - 1; x++) {
            y2 = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(sample[x] - ADC_OFFSET)));
            GrLineDraw(&sContext, x-1, y, x, y2);
            y = y2;
        }

        // Display time scale
        GrContextForegroundSet(&sContext, ClrWhite); // white text
        snprintf(str, sizeof(str), gTimeScaleStr[tState]);
        GrStringDraw(&sContext, str, /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);

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
    }
}
