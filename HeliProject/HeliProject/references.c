/*
 * references.c
 *
 *  Created on: 17/08/2020
 *      Author: sek40
 *
 * ---------------------------------------
 * Implements the setters for changing altitude and yaw reference points and
 * getters for retrieving the current reference points.
 */

// Standard modules
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// Tiva / M4 modules
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "utils/ustdlib.h"

// Heli modules
#include "yaw.h"
#include "buttons.h"
#include "buttonTasks.h"
#include "fsm.h"
#include "pidControl.h"
#include "altitude.h"

// RTOS modules
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/timers.h"

// Yaw increment for button press
#define YAW_CHANGE 15
#define ALT_CHANGE 10

static SemaphoreHandle_t g_altitudeMutex;
static int32_t g_altitudeReference = 0;

static SemaphoreHandle_t g_yawMutex;
static int32_t g_yawReference = 0;

/***********************************Initiliases******************************/
/*
 * Initialises References, yaw and altitude
 */
void initReferences(void)
{
    g_altitudeMutex = xSemaphoreCreateMutex();
    g_yawMutex = xSemaphoreCreateMutex();
    initYaw();
    alt_init();
}


/***********************************Setters and Getters******************************/
/*
 * Sets the altitude reference
 */
void setAltitudeReference(int32_t new_altitude)
{
    if (xSemaphoreTake(g_altitudeMutex, (TickType_t) 10) == true) //Take mutex
    {
        g_altitudeReference = new_altitude;
        xSemaphoreGive(g_altitudeMutex); //give mutex
    }
    //Set control -> altitude reference

}

/*
 * Sets the yaw reference
 */
void setYawReference(int16_t new_yaw)
{
    if (xSemaphoreTake(g_yawMutex, (TickType_t) 10) == true) //Take mutex
    {
        g_yawReference = new_yaw;
        xSemaphoreGive(g_yawMutex); //give mutex
    }
}

int32_t getAltitudeReference(void)
{
    return g_altitudeReference;
}

int32_t getYawReference(void)
{
    return g_yawReference;
}

/*
 * Updates the altitude and yaw references given the button press
 */
void UpdateReferences(int8_t pressed_button)
{
    switch (pressed_button)
    {
    case UP:
        if (g_altitudeReference < (MAX_HEIGHT - ALT_CHANGE))
        { //If not within 10% of max altitude

            setAltitudeReference(g_altitudeReference + ALT_CHANGE);
        }
        else
        {
            setAltitudeReference(MAX_HEIGHT);
        }
        break;
    case DOWN:
        // Checks lower limits of altitude if down button is pressed
        if (g_altitudeReference > ALT_CHANGE)
        {
            setAltitudeReference(g_altitudeReference - ALT_CHANGE);
        }
        else
        {
            setAltitudeReference(0);
        }
        break;
    case RIGHT:
        if (g_yawReference < ((MAX_DEGREE-1) - YAW_CHANGE))
        {
            setYawReference(g_yawReference + YAW_CHANGE);
        } else {
            setYawReference(g_yawReference + YAW_CHANGE - MAX_DEGREE);
        }
        break;
    case LEFT:
        if (g_yawReference > YAW_CHANGE)
        {
            setYawReference(g_yawReference - YAW_CHANGE);
        } else {
            setYawReference(g_yawReference - YAW_CHANGE + MAX_DEGREE);
        }
        break;
    }
}
