/*******************************************************************************
 * 
 * altitude.h
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
 * Written by:
 *  - Manu Hamblyn  <mfb31<@uclive.ac.nz>   95140875
 *  - Will Cowper   <wgc22@uclive.ac.nz>    81163265
 *  - Jesse Sheehan <jps111@uclive.ac.nz>   53366509
 * 
 * Description:
 * This module contains functionality required for calculating the mean altitude
 * as a raw value and as a percentage of the overall height.
 * Functions are provided to initialise, calibrate, update and return
 * the altitude values.
 * 
 ******************************************************************************/

#ifndef ALTITUDE_H_
#define ALTITUDE_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_HEIGHT 100

/**
 * Initialises the altitude module.
 * This must be called before any other functions in the altitude module.
 */
void alt_init(void);

/**
 * Calculates the mean value of the altitude, both the raw resolution and the percentage values. This should be called before one wants to use the altitude values in any other calculations.
 */
void alt_update();

/**
 * Returns the mean altitude as a percentage (usually from 0 - 100). This value can be less than 0 or greater than 100.
 */
int16_t alt_get(void);

/**
 * Returns `true` if the altitude has been calibrated.
 */
bool alt_has_been_calibrated(void);

/*
 * Processes the sampled ADC value
 */
void alt_process_adc(void);

/*
 * Resets the calibration state of the altitude.
 */
void alt_reset_calibration_state(void);

/*
 * Initiates the altitude measurement,
 * Gets the current height
 */
void GetAltitude(void *pvParameters);

#endif /*ALTITUDE_H_*/
