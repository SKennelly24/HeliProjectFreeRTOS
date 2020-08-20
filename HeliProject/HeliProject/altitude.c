/*******************************************************************************
 *
 * altitude.c
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
 *-------------------------------------------------------------------------------
 * ENEL361 Helicopter Project
 * Friday Morning, Group 7
 *
 * Originally written by:
 *  - Manu Hamblyn
 *  - Will Cowper
 *  - Jesse Sheehan
 *
 * Description:
 * This module contains functionality required for calculating the mean altitude
 * as a raw value and as a percentage of the overall height.
 * Functions are provided to initialise, calibrate, update and return
 * the altitude values.
 *
 * Original code from P. J. Bones included and referenced.
 *
 ******************************************************************************/
// Standard modules
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <limits.h>

// Tiva / M4 modules
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"

// Heli modules
#include "altitude.h"
#include "circBuffT.h"
#include "taskDefinitions.h"

// RTOS modules
#include "FreeRTOS.h"
#include "task.h"

// Some default settings
#define ADC_RANGE 1241
#define ONE_HUNDRED_PERCENT 100
#define ALT_BUF_SIZE 16             // Size of the buffer used to store the raw ADC values
#define ADC_BASE ADC0_BASE
#define ADC_PERIPH SYSCTL_PERIPH_ADC0
#define ADC_SEQUENCE 3
#define ADC_STEP 0

// Variable definition
static circBuf_t g_circ_buffer;     // Circular buffer used to store the raw ADC values for calculating the mean.
static uint16_t g_alt_ref;          // Reference altitude. Required for calculating the altitude as a percentage.
static int16_t g_alt_percent;       // Mean altitude as a percentage of full height. Updated when the `void alt_update()` function is called.
static bool g_has_been_calibrated = false;  // Indicates if the altitude has been calibrated.
static int16_t g_conversions = 0;           // Counter to hold the amount of adc conversions


/**
 * (Original Code by P.J. Bones)
 * The handler for the ADC conversion complete interrupt.
 * Writes to the circular buffer.
 */
void alt_adc_int_handler(void)
{
    uint32_t value;

    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC_BASE, ADC_SEQUENCE, &value);

    // Place it in the circular buffer (advancing write index)
    writeCircBuf(&g_circ_buffer, value);

    // Clean up, clearing the interrupt
    ADCIntClear(ADC_BASE, ADC_SEQUENCE);
}


/**
 * (Original code by P.J. Bones)
 * Initialises the ADC module on the Tivaboard.
 */
void alt_init_adc(void)
{
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(ADC_PERIPH);
    while (!SysCtlPeripheralReady(ADC_PERIPH));

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC_BASE, ADC_SEQUENCE, ADC_TRIGGER_PROCESSOR, ADC_STEP);

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCE, ADC_STEP, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);

    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC_BASE, ADC_SEQUENCE);

    // Register the interrupt handler
    ADCIntRegister(ADC_BASE, ADC_SEQUENCE, alt_adc_int_handler);

    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC_BASE, ADC_SEQUENCE);
}

/**
 * (Original code by P.J. Bones)
 * The interrupt handler for the for SysTick interrupt.
 */
void alt_process_adc(void)
{
    if (!g_has_been_calibrated) {
        g_conversions++;
    }
    // Initiate a conversion
    ADCProcessorTrigger(ADC_BASE, ADC_SEQUENCE);
}

void alt_init(void)
{
    // initialise the ADC for the altitude
    alt_init_adc();

    // initialise the circular buffers
    initCircBuf(&g_circ_buffer, ALT_BUF_SIZE);
}

/*
 * Given the sample mean ADC return the altitude as a percentage
 */
int16_t getAltitudePercent(int32_t sample_mean_adc)
{
    int32_t adc_change;
    //Checks lower boundary for ADC value
    if (g_alt_ref < sample_mean_adc)
    {
        adc_change = 0;
    }
    //Calculates the difference in reference ADC height and sample ADC height
    else
    {
        adc_change = (g_alt_ref - sample_mean_adc);
    }
    //Checks upper boundary of ADC in percentage
    if (((adc_change * ONE_HUNDRED_PERCENT) / ADC_RANGE) > ONE_HUNDRED_PERCENT)
    {
        return ONE_HUNDRED_PERCENT;
    }
    //Converts ADC to percentage
    else
    {
        return (int16_t) ((adc_change * MAX_HEIGHT) / ADC_RANGE);
    }
}

/*
 * Called by the update function when enough conversions have happened,
 * this ensures the altitude is calibrated correctly
 */
void alt_calibrate(int32_t alt_raw)
{
    g_alt_ref = alt_raw;
    g_has_been_calibrated = true;
}


void alt_update(void)
{
    int32_t sum;
    uint16_t i;
    int32_t alt_raw;

    // add up all the values in the circular buffer
    sum = 0;

    for (i = 0; i < ALT_BUF_SIZE; i++)
    {
        sum = sum + readCircBuf(&g_circ_buffer);
    }

    // calculate the mean of the data in the circular buffer
    alt_raw = (2 * sum + ALT_BUF_SIZE) / (2 * ALT_BUF_SIZE);

    if (alt_has_been_calibrated()) {
    }
    else if (g_conversions > ALT_BUF_SIZE) {
        alt_calibrate(alt_raw);
        g_has_been_calibrated = true;
    }
    g_alt_percent = getAltitudePercent(alt_raw);
}


int16_t alt_get(void)
{
    return g_alt_percent;
}

bool alt_has_been_calibrated(void)
{
    return g_has_been_calibrated;
}

void alt_reset_calibration_state(void)
{
    g_has_been_calibrated = false;
    g_conversions = 0;
}

/*
 * Initiates the altitude measurement,
 * Gets the current height
 */
void GetAltitude(void *pvParameters)
{
    while (1)
    {
        alt_process_adc();
        alt_update();
        vTaskDelay(TICKS_IN_SECOND / (portTICK_RATE_MS * ALITUDE_MEAS_FREQ)); //  Current frequency is
    }
    // No way to kill this task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}



