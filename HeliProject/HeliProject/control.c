/*******************************************************************************
 *
 * control.c
 *
 * ENEL361 Helicopter Project
 * Friday Morning, Group 7
 *
 * Written by:
 *  - Manu Hamblyn  <mfb31<@uclive.ac.nz>   95140875
 *  - Will Cowper   <wgc22@uclive.ac.nz>    81163265
 *  - Jesse Sheehan <jps111@uclive.ac.nz>   53366509
 *
 * Description:
 * Contains PID controllers for yaw and altitude
 *
 ******************************************************************************/
#include <stdint.h>

#include "control.h"
#include "altitude.h"
#include "yaw.h"
#include "pwm.h"
#include "utils.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"

#define CONTROL_RUN_FREQ 100 // run PID control 100 times a second

/**
* The altitude gains.
*/
#define ALT_KP 0.2f;
#define ALT_KI 0.001f;
#define ALT_KD 0.93f;

/**
* The yaw gains.
*/
#define YAW_KP 0.3f;
#define YAW_KI 0.001f;
#define YAW_KD 0.83f;

int16_t volatile YAW_TARGET = 0;    // Degrees
uint8_t volatile ALT_TARGET = 0;    // Percent
bool volatile PID_ACTIVE = false;   // A hack to turn the PID On/Off as needed.

// Idle main duty, allows for faster take off, reducing dependence on integral error (duty cycle %).
static const uint8_t IDLE_MAIN_DUTY = 20;
// Min speed of main rotor, allows for proper anti-clockwise yaw control and clamps descent speed (duty cycle %)
static const uint8_t MIN_MAIN_DUTY = 20;
// Max speed of main motor to stay within spec (duty cycle %)
static const uint8_t MAX_MAIN_DUTY = 65;    //was 70

// Min speed of tail rotor, prevents wear on motor by idling it instead of completely powering off during large C-CW movements (duty cycle %)
// also reduces the time taken to spool up motor during sudden large C-CW->CW movements.
static const uint8_t MIN_TAIL_DUTY = 5;
// Max speed of tail motor to stay within spec (duty cycle %)
static const uint8_t MAX_TAIL_DUTY = 65;    //was 70

// Clamps for Kp and Kd gains for each rotor (duty cycle %)
static const uint8_t MAIN_GAIN_CLAMP = 10;  //was 10
static const uint8_t TAIL_GAIN_CLAMP = 15;  //was 10

// clamp for Ki growth for large errors (error)
static const uint8_t INTEGRAL_TAIL_CLAMP = 20; //was 30
static const uint8_t INTEGRAL_MAIN_CLAMP = 5;

void set_altitude_target(uint8_t new_alt_target)
{
    ALT_TARGET = new_alt_target;
}

void set_yaw_target(int16_t new_yaw_target)
{
    YAW_TARGET = new_yaw_target;
}

void set_PID_ON(void)
{
    PID_ACTIVE = true;
}

void set_PID_OFF(void)
{
    PID_ACTIVE = false;
}


void control_update_altitude(void *pvParameters)
{
    // temp variables used to calculate new gain and direction
    float Pgain = 0;
    float Igain = 0;
    float Dgain = 0;

    int16_t cumulative = 0;
    int16_t error = 0;
    int16_t lastError = 0;
    int8_t duty = 0;                // Percent
    int8_t new_Duty = 0;

    while (1)
    {

    if (PID_ACTIVE)
    {
        // the difference between what we want and what we have (as a percentage)
        error = ALT_TARGET - (int16_t)alt_get();

        // P control, *kp;
        Pgain = error * ALT_KP;
        Pgain = clamp(Pgain, -MAIN_GAIN_CLAMP, MAIN_GAIN_CLAMP);

        // I control
        // only accumulate error if we are not motor duty limited (limits overshoot)
        if (duty > MIN_MAIN_DUTY && duty < MAX_MAIN_DUTY) {
            cumulative += clamp(error, -INTEGRAL_MAIN_CLAMP, INTEGRAL_MAIN_CLAMP);; // Clamp integral growth for large errors
        }
        Igain = cumulative * ALT_KI;

        // D control, clamped to 10%
        Dgain = (error - lastError) * ALT_KD;
        lastError = error;
        Dgain = clamp(Dgain, -MAIN_GAIN_CLAMP, MAIN_GAIN_CLAMP);

        // Calculate new motor duty percentage gain
        new_Duty = IDLE_MAIN_DUTY + Pgain + Igain + Dgain;

        // clamp motor to be within spec
        new_Duty = clamp(new_Duty, MIN_MAIN_DUTY, MAX_MAIN_DUTY);

        // set the motor duty
        pwm_set_main_duty(new_Duty);

        // update the duty
        duty = new_Duty;
    }
    vTaskDelay(1 / (CONTROL_RUN_FREQ * portTICK_RATE_MS));
    //Not the right approach but using to get started
    }
}


void control_update_yaw(void *pvParameters)
{
    // temp variables used to calculate new gain and direction
    float Pgain = 0;
    float Igain = 0;
    float Dgain = 0;

    int16_t cumulative = 0;
    int16_t error = 0;
    int16_t lastError = 0;
    int8_t duty = 0;                // Percent
    int8_t new_Duty = 0;

    while(1)
    {

        if (PID_ACTIVE)
            {
            // the difference between what we want and what we have (in degrees)
            error = (YAW_TARGET - yawInDegrees());    // Update our target

            // P control with +- 10% clamp
            Pgain = error * YAW_KP;
            Pgain = clamp(Pgain, -TAIL_GAIN_CLAMP, TAIL_GAIN_CLAMP);

            // I control, only accumulate error if we are not motor duty limited (limits overshoot)
            //if (duty > MIN_TAIL_DUTY && duty < MAX_TAIL_DUTY) //change
            //{
            //    cumulative += clamp(error, -INTEGRAL_TAIL_CLAMP, INTEGRAL_TAIL_CLAMP);; // Clamp integral growth for large errors
            //}
            Igain = cumulative * YAW_KI;

            // D control with clamp
            Dgain = (error - lastError) * YAW_KD; // Control is called with fixed frequency so time delta can be ignored.
            lastError = error;
            Dgain = clamp(Dgain, -TAIL_GAIN_CLAMP, TAIL_GAIN_CLAMP);

            // Calculate new motor duty percentage gain
            new_Duty = Pgain + Igain + Dgain;

            // clamp motor to be within spec
            new_Duty = clamp(new_Duty, MIN_TAIL_DUTY, MAX_TAIL_DUTY);

            // set the motor duty
            pwm_set_tail_duty(new_Duty);

            // update the duty
            duty = new_Duty;
            }

        vTaskDelay(1 / (CONTROL_RUN_FREQ * portTICK_RATE_MS));
        //Not the right approach but using to get started
    }
}
