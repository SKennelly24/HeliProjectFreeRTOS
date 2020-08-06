//*****************************************************
//
// adc.c - Uses analog to digital conversion to take
// samples of the altitude and store in a circular.
// The height value can be calculated using the values
// from the buffer.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#include "adc.h"
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
#include "clock.h"

//******************************************************
// Constants
//******************************************************
#define ADC_RANGE 1241
#define ONE_HUNDRED_PERCENT 100
#define BUF_SIZE 8
#define SAMPLE_RATE_HZ 100
#define DISPLAY_RATE_HZ 2

//******************************************************
// Globals
//******************************************************
static circBuf_t g_inBuffer; // Buffer of size BUF_SIZE integers (sample values)
static int32_t g_zero_reference;


//******************************************************
// The interrupt handler for the ADC conversion.
//******************************************************
void ADCIntHandler(void)
{
    uint32_t ulValue;

    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

    // Place it in the circular buffer (advancing write index)
    writeCircBuf(&g_inBuffer, ulValue);

    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

//******************************************************
//Initializes the ADC
//******************************************************
void initADC(void)
{
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
    ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    // Register the interrupt handler
    ADCIntRegister(ADC0_BASE, 3, ADCIntHandler);

    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);

    initCircBuf(&g_inBuffer, BUF_SIZE);
}


//******************************************************
// Function to set the current voltage value from the sensor or supply as a
// reference value using a method to find the average over the sampled voltages.
//******************************************************
int32_t setZeroReference(void)
{
    g_zero_reference = getSampleMeanADC();
    return g_zero_reference;
}

//******************************************************
//Given the sample mean ADC returns the altitude percent
//******************************************************
int32_t getAltitudePercent(int32_t sample_mean_adc)
{
    int32_t adc_change;
    //Checks lower boundary for ADC value
    if (g_zero_reference < sample_mean_adc)
    {
        adc_change = 0;
    }
    //Calculates the difference in reference ADC height and sample ADC height
    else
    {
        adc_change = (g_zero_reference - sample_mean_adc);
    }
    //Checks upper boundary of ADC in percentage
    if (((adc_change * ONE_HUNDRED_PERCENT) / ADC_RANGE) > ONE_HUNDRED_PERCENT)
    {
        return ONE_HUNDRED_PERCENT;
    }
    //Converts ADC to percentage
    else
    {
        return ((adc_change * 100) / ADC_RANGE);
    }
}

//******************************************************
//Returns the current sample mean ADC
//******************************************************
int32_t getSampleMeanADC(void)
{
    int32_t sample_mean_adc;
    int32_t sum;
    int16_t i;
    //Reads the circular buffer and calculates the sample mean ADC
    sum = 0;
    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf(&g_inBuffer);
    sample_mean_adc = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
    return sample_mean_adc;
}
