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
#include "flight_mode.h"
#include "utils.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"

#define CONTROL_RUN_FREQ 30 // run PID control 30 times a second

int16_t volatile YAW_TARGET = 0; // Degrees
uint8_t volatile ALT_TARGET = 0; // Percent

//struct control_state_s
//{
  /**
   * Gains, errors and duty for the associated motor
   */
 // float kp;
//  float ki;
//  float kd;
//  int16_t lastError;
//  int16_t cumulative;
//};

/**
 * Represents the internal state of the control system.
 */
//typedef struct control_state_s ControlState;

// Idle main duty, allows for faster take off, reducing dependence on integral error (duty cycle %).
static const uint8_t IDLE_MAIN_DUTY = 25;

// Min speed of main rotor, allows for proper anti-clockwise yaw control and clamps descent speed (duty cycle %)
static const uint8_t MIN_MAIN_DUTY = 20;
// Max speed of main motor to stay within spec (duty cycle %)
static const uint8_t MAX_MAIN_DUTY = 70;

// Min speed of tail rotor, prevents wear on motor by idling it instead of completely powering off during large C-CW movements (duty cycle %)
// also reduces the time taken to spool up motor during sudden large C-CW->CW movements.
static const uint8_t MIN_TAIL_DUTY = 1;
// Max speed of tail motor to stay within spec (duty cycle %)
static const uint8_t MAX_TAIL_DUTY = 70;

// Clamps for Kp and Kd gains for each rotor (duty cycle %)
static const uint8_t MAIN_GAIN_CLAMP = 10;
static const uint8_t TAIL_GAIN_CLAMP = 10;

// clamp for integral growth for large errors (error)
static const uint8_t INTEGRAL_TAIL_CLAMP = 30;
static const uint8_t INTEGRAL_MAIN_CLAMP = 5;

//static ControlState g_control_altitude;
//static ControlState g_control_yaw;

static bool g_enable_altitude;
static bool g_enable_yaw;

/**
 * Helper function to create a ControlState struct from a ControlGains struct.
 */
/*
ControlState control_get_state_from_gains(ControlGains t_gains)
{
    return (ControlState){
        t_gains.kp, // Kp
        t_gains.ki, // Ki
        t_gains.kd, // Kd
        0, // lastError
        0, // cumulative error
        0}; // current duty% of motor
}

void control_init(ControlGains t_altitude_gains, ControlGains t_yaw_gains)
{
    // initialise the control states of the altitude and yaw
    g_control_altitude = control_get_state_from_gains(t_altitude_gains);
    g_control_yaw = control_get_state_from_gains(t_yaw_gains);
}
*/

void set_altitude_target(uint8_t new_alt_target)
{
    ALT_TARGET = new_alt_target;
}

void set_yaw_target(int16_t new_yaw_target)
{
    YAW_TARGET = new_yaw_target;
}

void control_update_altitude(void *pvParameters)
{
    // temp variables used to calculate new gain and direction
    float Pgain = 0;
    float Igain = 0;
    float Dgain = 0;

    float kp = 0;
    float ki = 0;
    float kd = 0;

    int16_t cumulative = 0;
    int16_t error = 0;
    int16_t lastError = 0;
    int8_t duty = 0;                // Percent
    int8_t new_Duty = 0;
    int8_t alt_target = 0;

    while (1)
    {

/*
    if (!g_enable_altitude)
    {
        return;
    }
*/


    // the difference between what we want and what we have (as a percentage)
    int16_t error = ALT_TARGET - alt_get();

    // P control, *kp;
    Pgain = error * kp;
    Pgain = clamp(Pgain, -MAIN_GAIN_CLAMP, MAIN_GAIN_CLAMP);

    // I control
    // only accumulate error if we are not motor duty limited (limits overshoot)
    if (duty > MIN_MAIN_DUTY && duty < MAX_MAIN_DUTY) {
        cumulative += clamp(error, -INTEGRAL_MAIN_CLAMP, INTEGRAL_MAIN_CLAMP);; // Clamp integral growth for large errors
    }
    Igain = cumulative * ki;

    // D control, clamped to 10%
    Dgain = (error - lastError) * kd;
    lastError = error;
    Dgain = clamp(Dgain, -MAIN_GAIN_CLAMP, MAIN_GAIN_CLAMP);

    // Calculate new motor duty percentage gain
    new_Duty = IDLE_MAIN_DUTY + Pgain + Igain + Dgain;

    // clamp motor to be within spec
    new_Duty = clamp(new_Duty, MIN_MAIN_DUTY, MAX_MAIN_DUTY);

    // update the duty
    duty = new_Duty;

    // set the motor duty
    pwm_set_main_duty(duty);

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

    float kp = 0;
    float ki = 0;
    float kd = 0;

    int16_t cumulative = 0;
    int16_t error = 0;
    int16_t lastError = 0;
    int8_t duty = 0;                // Percent
    int8_t new_Duty = 0;

    while(1)
    {

        // the difference between what we want and what we have (in degrees)
        error = (YAW_TARGET - yawInDegrees());    // Update our target

        // P control with +- 10% clamp
        Pgain = error*kp;
        Pgain = clamp(Pgain, -TAIL_GAIN_CLAMP, TAIL_GAIN_CLAMP);

        // I control, only accumulate error if we are not motor duty limited (limits overshoot)
        if (duty > MIN_TAIL_DUTY && duty < MAX_TAIL_DUTY)
        {
            cumulative += clamp(error, -INTEGRAL_TAIL_CLAMP, INTEGRAL_TAIL_CLAMP);; // Clamp integral growth for large errors
        }
        Igain = cumulative*ki;

        // D control with +- 10% clamp
        Dgain = (error - lastError)*kd; // Control is called with fixed frequency so time delta can be ignored.
        lastError = error;
        Dgain = clamp(Dgain, -TAIL_GAIN_CLAMP, TAIL_GAIN_CLAMP);

        // Calculate new motor duty percentage gain
        new_Duty = Pgain + Igain + Dgain;

        // clamp motor to be within spec
        new_Duty = clamp(new_Duty, MIN_TAIL_DUTY, MAX_TAIL_DUTY);

        // update the duty
        duty = new_Duty;

        // set the motor duty
        pwm_set_tail_duty(duty);

        //new_yaw_target = yaw_target;

        vTaskDelay(1 / (CONTROL_RUN_FREQ * portTICK_RATE_MS));
        //Not the right approach but using to get started
    }
}
/*
void control_enable_yaw(bool t_enabled)
{
    g_enable_yaw = t_enabled;
    if (!g_enable_yaw)
    {
        g_control_yaw.cumulative = 0;
        g_control_yaw.duty = 0;
        g_control_yaw.lastError = 0;
        pwm_set_tail_duty(0);
    }
    else
    {
        pwm_set_tail_duty(g_control_yaw.duty);
    }
}

void control_enable_altitude(bool t_enabled)
{
    g_enable_altitude = t_enabled;
    if (!g_enable_altitude)
    {
        g_control_altitude.cumulative = 0;
        g_control_altitude.duty = 0;
        g_control_altitude.lastError = 0;
        pwm_set_main_duty(0);
    }
    else
    {
        pwm_set_main_duty(g_control_altitude.duty);
    }
}
*/
