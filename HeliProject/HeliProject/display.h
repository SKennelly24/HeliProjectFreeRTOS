/*******************************************************************************
 *
 * display.h
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
 * -----------------------------------------------------------------------------
 * ENEL361 Helicopter Project
 * Friday Morning, Group 7
 *
 * Written by:
 *  - Manu Hamblyn  <mfb31<@uclive.ac.nz>   95140875
 *  - Will Cowper   <wgc22@uclive.ac.nz>    81163265
 *  - Jesse Sheehan <jps111@uclive.ac.nz>   53366509
 *
 * Created on: 10/04/2019
 *
 * Description:
 * This module contains functions for initialising and updating the display.
 *
 ******************************************************************************/

#ifndef DISPLAY_H_
#define DISPLAY_H_

/**
 * Bytecode for rendering degree symbol on the display
 */
static const int DISP_SYMBOL_DEGREES = 0x60;

#include <stdint.h>
//#include "kernel.h" // kernel not needed using RTOS

/**
 * Initialises the display module.
 * This must be called before any other functions in the display module.
 */
void disp_init(void);

/**
 * Used to cycle through the valid display states
 * Wraps back to state 0 when all states exhausted
 *
 */
void disp_advance_state(void);

/**
 * Clears the display if g_displayState differs from the previous time disp_render() was called
 * Then renders to the display based on the value of g_displayState
 */
//void disp_render(void *pvParameters);

/**
 * New OLED display task
 */
void disp_Values(void *pvParameters);

#endif
