/*
 * pidControl.c
 *
 *  Created on: 12/08/2020
 *  Author: sek40
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pidControl.h"
#include "altitude.h"
#include "yaw.h"
#include "pwm.h"
#include "utils.h"

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"

#define CONTROL_RUN_FREQ 50
#define MIN_MAIN_DUTY 20
#define MAX_MAIN_DUTY 65
#define MIN_TAIL_DUTY 23
#define MAX_TAIL_DUTY 70
#define MAX_YAW 360

static int16_t YAW_TARGET = 0;    // Degrees
static uint8_t ALT_TARGET = 0;
typedef enum CONTROLLER_CHOICE
{
    ALTITUDE = 0,
    YAW,
} CONTROLLER_CHOICE;

static Controller altitudeController = {
                                        .pGain = 0.6,
                                        .iGain = 0.4,
                                        .errorIntegrated = 100.0
};

static Controller yawController = {
                                        .pGain = 0.51,
                                        .iGain = 0.25,
                                        .errorIntegrated = 0.0
};

double pidUpdate(Controller * selectedController, double error, uint8_t controller_choice) {
    double control;
    double delta_t = (double) 1 / CONTROL_RUN_FREQ;
    selectedController->errorIntegrated += (error * delta_t);
    control = error * selectedController->pGain
            + (selectedController->errorIntegrated * selectedController->iGain);
    return control;
}

void set_altitude_target(uint8_t new_alt_target)
{
    ALT_TARGET = new_alt_target;
}

void set_yaw_target(int16_t new_yaw_target)
{
    YAW_TARGET = new_yaw_target;
}

void setAltitudeDuty(void)
{
    int16_t altitudeError;
    int16_t control;
    int16_t newDuty;

    altitudeError = ALT_TARGET - alt_get();
    control = (int16_t) pidUpdate(&altitudeController, altitudeError, ALTITUDE);
    newDuty = clamp(control, MIN_MAIN_DUTY, MAX_MAIN_DUTY);
    pwm_set_main_duty(newDuty);
}

 void setYawDuty(void)
 {
     int16_t yawError;
     int16_t control;
     int16_t newDuty;

      yawError = YAW_TARGET - getYaw();
      if ((abs(yawError) > (MAX_YAW /2))) {
          yawError = copysign(MAX_YAW - abs(yawError), -yawError);
      }

      control = (int16_t) pidUpdate(&yawController, yawError, YAW);
      newDuty = clamp(control, MIN_TAIL_DUTY, MAX_TAIL_DUTY);
      pwm_set_tail_duty(newDuty);
 }

void apply_control(void *pvParameters)
{

    while(1)
    {

        setAltitudeDuty();
        setYawDuty();
        vTaskDelay(1000 / (portTICK_RATE_MS * CONTROL_RUN_FREQ));
    }
}

void reset_yaw_error(void)
{
    yawController.errorIntegrated = 0;
}


