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
 *  Hardcoded display values for test
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

#include <stdint.h>

#include "OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"

// Heli Project
#include "altitude.h" //commented out for test
#include "display.h"
#include "pwm.h"      //commented out for test
#include "utils.h"
#include "yaw.h"      //commented out for test

/**
 * Bytecode for rendering degree symbol on the display
 */
//static const int DISP_SYMBOL_DEGREES = 0x60;
/**
 * Enum of all states the display can be in. Cycled by pressing BTN2
 */
enum disp_state
{
    DISP_STATE_CALIBRATION, DISP_STATE_ALL, DISP_STATE_TOTAL
};
typedef enum disp_state DisplayState;


/**
 * Current display state
 */
static uint8_t g_displayState = DISP_STATE_CALIBRATION;


/**
 * Display raw 12-bit adc reading to the display
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
 * Advance display state when BTN2 is pressed
 */
void disp_advance_state(void)
{
    if (++g_displayState >= DISP_STATE_TOTAL)
    {
        g_displayState = DISP_STATE_CALIBRATION + 1;
    }
}


/**
 * Display yaw and altitude percentage at the same time
 * Was disp_all()
 */
void disp_Values(void *pvParameters)
{
    char string[17] = { 0 };

    while (1)
    {

        usnprintf(string, sizeof(string), "Main Duty: %4d%%", pwm_get_main_duty());
        //usnprintf(string, sizeof(string), "Main Duty: %4d%%", get_rand_percent());    // Test only
        OLEDStringDraw(string, 0, 0);

        usnprintf(string, sizeof(string), "Tail Duty: %4d%%", pwm_get_tail_duty());
        //usnprintf(string, sizeof(string), "Tail Duty: %4d%%", get_rand_percent());    // Test only
        OLEDStringDraw(string, 0, 1);

        //usnprintf(string, sizeof(string), "      Yaw: %4d%c", yaw_get(), DISP_SYMBOL_DEGREES);
        //usnprintf(string, sizeof(string), "      Yaw: %4d%c", get_rand_yaw(), DISP_SYMBOL_DEGREES); // Test only
        usnprintf(string, sizeof(string), "      Yaw: %4d%c", yawInDegrees(), DISP_SYMBOL_DEGREES);
        OLEDStringDraw(string, 0, 2);

        usnprintf(string, sizeof(string), " Altitude: %4d%%", alt_get());
        //usnprintf(string, sizeof(string), " Altitude: %4d%%", get_rand_percent()));   // Test only
        OLEDStringDraw(string, 0, 3);

        vTaskDelay(500 / portTICK_RATE_MS); // Suspend this task (so others may run) for 1000ms or as close as we can get with the current RTOS tick setting.
    }
    // No way to kill this blinky task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}

/**
 * Unknown display state fail-safe
 */
void disp_unknown(void)
{
    OLEDStringDraw("Unknown display", 0, 2);
    OLEDStringDraw("state!", 0, 3);
}

/*
 void disp_render(void *pvParameters)
{
    switch (g_displayState)
    {
    case DISP_STATE_CALIBRATION:
        disp_calibration();
        break;
    case DISP_STATE_ALL:
        disp_Values();
        break;
    default:
        disp_unknown();
        break;
    }
}*/


void disp_init(void)
{
    // Intialise the Orbit OLED display
    OLEDInitialise();
    disp_clear();
}
