/*
 * pidControl.c
 *
 *  Created on: 12/08/2020
 *  Author: sek40
 *  ----------------------------------------
 *  Implementation of PI controller
 */

// General modules
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Heli modules
#include "pidControl.h"
#include "altitude.h"
#include "yaw.h"
#include "pwm.h"
#include "taskDefinitions.h"
#include "references.h"
#include "utils.h"

//Free RTOS modules
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"


// Main and Tail duty limits
// Rig 1
/*
#define MIN_MAIN_DUTY 5
#define MAX_MAIN_DUTY 55
#define MIN_TAIL_DUTY 5
#define MAX_TAIL_DUTY 55
#define MAX_YAW 360
*/

// Rig 2
/*
#define MIN_MAIN_DUTY 7
#define MAX_MAIN_DUTY 16
#define MIN_TAIL_DUTY 0
#define MAX_TAIL_DUTY 65
#define MAX_YAW 360
*/

/* //Rig 3
#define MIN_MAIN_DUTY 20
#define MAX_MAIN_DUTY 65
#define MIN_TAIL_DUTY 18
#define MAX_TAIL_DUTY 65
#define MAX_YAW 360
*/

/* //Rig 4
#define MIN_MAIN_DUTY 25
#define MAX_MAIN_DUTY 80
#define MIN_TAIL_DUTY 18
#define MAX_TAIL_DUTY 65
#define MAX_YAW 360
*/

// Generic
#define MIN_MAIN_DUTY 20
#define MAX_MAIN_DUTY 65
#define MIN_TAIL_DUTY 18
#define MAX_TAIL_DUTY 65
#define MAX_YAW 360


// *************************************************************
// Proportional and Integral gains.
// (These are either a compromise to work on all (most) rigs or
// Specifically adjusted for a particular rig.)

/*
* Proportional and Integral gains for altitude PI control
*/
static Controller altitudeController = {
                                       .pGain = 0.20, //R1 .20, R2 .07, R3 .39
                                       .iGain = 0.22, //R1 .22, R2 .18, R3 .25
                                       .errorIntegrated = 100.0
};
/*
* Proportional and Integral gains for yaw PI control
*/
static Controller yawController = {
                                       .pGain = 0.41, //R1 .41, R2 .45, R3 .51
                                       .iGain = 0.24, //R1 .23, R2 .45, R3 .25
                                       .errorIntegrated = 0.0
};
// *************************************************************

/*
 * Implements the PI algorithm.
 * Accepts (pointer) controller gains, (double) error value, (enum) controller_choice
 * Returns (double) calculated control value
 */
double pidUpdate(Controller * selectedController, double error) {
    double control;
    double delta_t = (double) 1 / CONTROL_RUN_FREQ;
    selectedController->errorIntegrated += (error * delta_t);
    control = error * selectedController->pGain
            + (selectedController->errorIntegrated * selectedController->iGain);
    return control;
}

/* Calculates error in (between current and desired) altitude and
 * calls PI control with this value.
 * Clamps the new duty (from PI control) within system limits and
 * sets the altitude PWM module value accordingly.
*/
void setAltitudeDuty(void)
{
    int16_t altitudeError;
    int16_t control;
    int16_t newDuty;

    altitudeError = getAltitudeReference() - alt_get();
    control = (int16_t) pidUpdate(&altitudeController, altitudeError);
    newDuty = clamp(control, MIN_MAIN_DUTY, MAX_MAIN_DUTY);
    pwm_set_main_duty(newDuty);
}

/*
 * Calculates error in (between current and desired) yaw and
 * calls PI control with this value..
 * Clamps the new duty (from PI control) within system limits and
 * sets the yaw PWM module value accordingly.
 */
 void setYawDuty(void)
 {
     int16_t yawError;
     int16_t control;
     int16_t newDuty;

      yawError = getYawReference() - getYaw();
      if ((abs(yawError) > (MAX_YAW /2))) {
          yawError = copysign(MAX_YAW - abs(yawError), -yawError);
      }

      control = (int16_t) pidUpdate(&yawController, yawError);
      newDuty = clamp(control, MIN_TAIL_DUTY, MAX_TAIL_DUTY);
      pwm_set_tail_duty(newDuty);
 }


void apply_control(void *pvParameters)
{

    while(1)
    {
        setAltitudeDuty();
        setYawDuty();
        vTaskDelay(TICKS_IN_SECOND / (portTICK_RATE_MS * CONTROL_RUN_FREQ));
        //vTaskDelay(TICKS_IN_SECOND / (CONTROL_RUN_FREQ));

    }
}

void reset_yaw_error(void)
{
    yawController.errorIntegrated = 0;
}


