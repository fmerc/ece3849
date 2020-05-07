/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */

/* XDCtools Header files */
#include <xdc/std.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <configPkg/package/cfg/rtos_pem4f.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* Lab 1 header files */
#include "grlib/grlib.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sampling.h"
#include "buttons.h"

uint32_t gSystemClock = 120000000;  // [Hz] system clock frequency

// Voltage/time scale constants
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", " 1 V", " 2 V"};
const char * const gTriggerStr[] = {"+|", "|-"};

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0;

tContext sContext;

void LCD_Init(void);

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // Hardware initialization
    LCD_Init();
    cpu_clock_init();
    ButtonInit();
    DMA_Init();
    PWM_Init();
    FreqTimer();

    count_unloaded = cpu_load_count();  // measure CPU

    BIOS_start();   // Start BIOS

    return (0);
}

/* Display Task: draw waveform to LCD (low priority) */
void displayTask_func(UArg arg1, UArg arg2) {
    char str[50];       // string buffer
    int i;
    int x, y, y2;

    while (true) {
        Semaphore_pend(semDisplay, BIOS_WAIT_FOREVER);

        count_loaded = cpu_load_count();        // CPU load
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
        if (!spectrumMode) {
            GrContextForegroundSet(&sContext, ClrRed);
            GrLineDraw(&sContext, 64, 0, 64, 128);
            GrLineDraw(&sContext, 0, 64, 128, 64);
        }

        // Draw waveform
        if (spectrumMode) GrContextForegroundSet(&sContext, ClrDarkOrange);
        else GrContextForegroundSet(&sContext, ClrYellow);

        y = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(processedBuffer[0] - ADC_OFFSET)));
        for (x = 1; x < LCD_HORIZONTAL_MAX - 1; x++) {
            y2 = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(processedBuffer[x] - ADC_OFFSET)));
            GrLineDraw(&sContext, x-1, y, x, y2);
            y = y2;
        }

        GrContextForegroundSet(&sContext, ClrWhite); // white text

        if (spectrumMode) {
            GrStringDraw(&sContext, "20 kHz", -1, 0, 0, false);
            GrStringDraw(&sContext, "20 dB", -1, 50, 0, false);
        } else {
            GrStringDraw(&sContext, "20 us", -1, 0, 0, false);      // Time scale
            snprintf(str, sizeof(str), gVoltageScaleStr[vState]);   // Voltage scale
            GrStringDraw(&sContext, str, -1, 50, 0, false);
            snprintf(str, sizeof(str), gTriggerStr[trigState]);     // Trigger state
            GrStringDraw(&sContext, str, -1, 110, 0, false);
            snprintf(str, sizeof(str), "CPU load = %.1f%%", cpu_load*100);  // CPU load
            GrStringDraw(&sContext, str, -1, 0, 120, false);
            snprintf(str, sizeof(str), "f = %.0f Hz", avgFrequency);  // frequency
            GrStringDraw(&sContext, str, -1, 0, 110, false);
            snprintf(str, sizeof(str), "T = %d", pwmPeriod);  // period
            GrStringDraw(&sContext, str, -1, 0, 100, false);
        }
        GrFlush(&sContext); // flush the frame buffer to the LCD

        Semaphore_post(semWaveform);
    }
}



/* Initialize LCD for display*/
void LCD_Init(void) {
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    GrContextInit(&sContext, &g_sCrystalfontz128x128);
    GrContextFontSet(&sContext, &g_sFontFixed6x8);
}
