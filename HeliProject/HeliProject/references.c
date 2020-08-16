/*
 * references.c
 *
 *  Created on: 17/08/2020
 *      Author: sek40
 */

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

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/timers.h"

#define YAW_CHANGE 15

static SemaphoreHandle_t g_altitudeMutex;
static int32_t g_altitudeReference = 0;

static SemaphoreHandle_t g_yawMutex;
static int32_t g_yawReference = 0;

/***********************************Initiliases******************************/
/*
 * Initialises References
 */
void initReferences(void)
{
    g_altitudeMutex = xSemaphoreCreateMutex();
    g_yawMutex = xSemaphoreCreateMutex();
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
        set_altitude_target( (uint8_t) g_altitudeReference);
        xSemaphoreGive(g_altitudeMutex); //give mutex
    }
    //Set control -> altitude reference

}

/*
 * Sets the yaw reference
 */
bool setYawReference(int16_t new_yaw)
{
    bool worked;
    if (xSemaphoreTake(g_yawMutex, (TickType_t) 10) == true) //Take mutex
    {
        g_yawReference = new_yaw;
        set_yaw_target( (int16_t) g_yawReference);
        xSemaphoreGive(g_yawMutex); //give mutex
        worked = true;
    } else {
        worked = false;
    }
    return worked;
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
        if (g_altitudeReference < (MAX_HEIGHT - 10))
        { //If not within 10% of max altitude

            setAltitudeReference(g_altitudeReference + 10);
        }
        else
        {
            setAltitudeReference(MAX_HEIGHT);
        }
        break;
    case DOWN:
        // Checks lower limits of altitude if down button is pressed
        if (g_altitudeReference > 10)
        {
            setAltitudeReference(g_altitudeReference - 10);
        }
        else
        {
            setAltitudeReference(0);
        }
        break;
    case RIGHT:
        if (g_yawReference < (359 - YAW_CHANGE))
        {
            setYawReference(g_yawReference + YAW_CHANGE);
        } else {
            setYawReference(g_yawReference + YAW_CHANGE - 360);
        }
        break;
    case LEFT:
        if (g_yawReference > YAW_CHANGE)
        {
            setYawReference(g_yawReference - YAW_CHANGE);
        } else {
            setYawReference(g_yawReference - YAW_CHANGE + 360);
        }
        break;
    }
}
