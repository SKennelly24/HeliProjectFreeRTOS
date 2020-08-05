/*******************************************************************************
 *
 * flight_mode.c
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
 *
 *  no longer uses an interrupt for the yaw reference
 *
 * -----------------------------------------------------------------------------
 * ENEL361 Helicopter Project
 * Friday Morning, Group 7
 *
 * 10/05/2019
 *
 * Written by:
 *  - Manu Hamblyn  <mfb31<@uclive.ac.nz>   95140875
 *  - Will Cowper   <wgc22@uclive.ac.nz>    81163265
 *  - Jesse Sheehan <jps111@uclive.ac.nz>   53366509
 *
 * Description:
 * This module contains implements the Operating Mode (or FLight Status)
 * of the helicopter under control of a Finite Sate Machine
 *
 *********************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "altitude.h"
//#include "config.h"
//#include "control.h"
#include "flight_mode.h"
#include "pwm.h"
//#include "setpoint.h"
#include "utils.h"
#include "yaw.h"

/**
 * Duty cycle % to apply to Tail while finding reference
 */
static const int PWM_TAIL_DUTY_YAW_REF = 18;

/**
 * The percentage altitude to hover when in landing state,
 * before finding the reference and zero yaw.
 */
static const int HOVER_ALTITUDE = 10;

/**
 * Holds the current state of the Operating mode (or FLight Status) Finite Sate Machine
 */
static volatile FlightModeState g_mode;

void flight_mode_init(void)
{
    g_mode = LANDED;        //Start in the landed state
}

FlightModeState flight_mode_get(void)
{
    return g_mode;
}

/*
void flight_mode_advance_state(void)
{
    switch (g_mode)
    {
    case LANDED:
        g_mode = TAKE_OFF;
        break;
    case TAKE_OFF:
        g_mode = IN_FLIGHT;
        break;
    case IN_FLIGHT:
        g_mode = LANDING;
        break;
    case LANDING:
        g_mode = LANDED;
        break;
    }
}
*/
