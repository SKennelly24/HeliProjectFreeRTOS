/*******************************************************************************
 *
 * display.c
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
 * Written by:
 *  - Manu Hamblyn  <mfb31<@uclive.ac.nz>   95140875
 *  - Will Cowper   <wgc22@uclive.ac.nz>    81163265
 *  - Jesse Sheehan <jps111@uclive.ac.nz>   53366509
 *
 * Description:
 * This module contains functions for initialising and updating the display.
 *
 ******************************************************************************/

// Standard modules
#include <stdint.h>

// Tiva M4 modules
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"

// Free RTOS modules
#include "FreeRTOS.h"
#include "task.h"

// Heli modules
#include "altitude.h" //commented out for test
#include "display.h"
#include "pwm.h"      //commented out for test
#include "yaw.h"      //commented out for test
#include "taskDefinitions.h"
#include "uart.h"


/**
 * Clear the display.
 */
void disp_clear(void)
{
    OLEDStringDraw("                ", 0, 0);
    OLEDStringDraw("                ", 0, 1);
    OLEDStringDraw("                ", 0, 2);
    OLEDStringDraw("                ", 0, 3);
}


/**
 * Splash screen used during initial calibration while waiting for buffer to fill
 */
void disp_calibration(void)
{
    OLEDStringDraw("ENCE464 Group 18", 0, 0);
    OLEDStringDraw("mfb31", 0, 1);
    OLEDStringDraw("sek40", 0, 2);
    OLEDStringDraw("dle70", 0, 3);
}


/**
 * Task to display yaw and altitude percentage at the same time
 */
void disp_Values(void *pvParameters)
{
    char string[17] = { 0 };

    while (1)
    {
        usnprintf(string, sizeof(string), "Main Duty: %4d%%", pwm_get_main_duty());
        OLEDStringDraw(string, 0, 0);

        usnprintf(string, sizeof(string), "Tail Duty: %4d%%", pwm_get_tail_duty());
        OLEDStringDraw(string, 0, 1);

        usnprintf(string, sizeof(string), "      Yaw: %4d%c", getYaw(), DISP_SYMBOL_DEGREES);
        OLEDStringDraw(string, 0, 2);

        usnprintf(string, sizeof(string), " Altitude: %4d%%", alt_get());
        OLEDStringDraw(string, 0, 3);

        vTaskDelay(TICKS_IN_SECOND / (DISPLAY_FREQ * portTICK_RATE_MS)); // Suspend this task (so others may run) for 1000ms or as close as we can get with the current RTOS tick setting.
    }
}


void disp_init(void)
{
    // Intialise the Orbit OLED display
    OLEDInitialise();
    disp_clear();
}
