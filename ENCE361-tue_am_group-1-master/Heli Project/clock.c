//*****************************************************
//
// clock.c - Sets up the clock and Systick interrupt.
// Functions getGCount and getSample count used allow global
// counters to be used in different files.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#include "clock.h"
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/adc.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "adc.h"

static uint32_t g_count;
static uint32_t prev_adc_count;
static volatile uint32_t g_ulSampCnt;    // Counter for the interrupts

#define BUF_SIZE 8
#define SAMPLE_RATE_HZ 100
#define DISPLAY_RATE_HZ 2
#define SYSTICK_RATE_HZ 48

//******************************************************
// The interrupt handler for the for SysTick interrupt.
//******************************************************
void SysTickIntHandler(void)
{
    // SYSTICK_RATE_HZ is the clock rate of the SysTick clock
    // SAMPLE_RATE_HZ is the sample rate for the ADC

    // If g_count < prev_adc_count, correct for rollover.
    if ((g_count - prev_adc_count) >= (SYSTICK_RATE_HZ / SAMPLE_RATE_HZ))
    {
        ADCProcessorTrigger(ADC0_BASE, 3);
        prev_adc_count = g_count;
        g_ulSampCnt++;
    }
    g_count++;
}

//******************************************************
//Initializes the clock
//******************************************************
void initClock(void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    SYSCTL_XTAL_16MHZ);

    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SYSTICK_RATE_HZ);

    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);

    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

//******************************************************
//Returns the current g_count
//******************************************************
uint32_t getGCount(void)
{
    return g_count;
}

//******************************************************
//Returns the sample count
//******************************************************
uint32_t getSampCnt(void)
{
    return g_ulSampCnt;
}

