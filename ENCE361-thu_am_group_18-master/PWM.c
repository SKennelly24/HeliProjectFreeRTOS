/* ****************************************************************
 * PWM.c
 *
 * Module for Pulse Width Modulation
 * Comprised of initialisers, setting functions and controllers for
 * main and tail rotors
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * Based off PWMGen.c - P.J. Bones, UCECE, 2018
 * ***************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "PWM.h"
#include "DEFINES.h"

// ************************* GLOBALS *******************************
// ************************* EXTERNAL ******************************
// Desired Yaw/Alt Values
volatile int TARGET_YAW = 0;
volatile int TARGET_ALT = 0;

// Current Duty Cycles Initialized To 0
volatile int MAIN_DUTY = 0;
volatile int TAIL_DUTY = 0;

// ************************* INTERNAL ******************************
// Previous Yaw/Alt Errors For Derivative Control
volatile int prev_error_yaw = 0;
volatile int prev_error_alt = 0;

// Integral Error Sums
volatile int integral_error_yaw = 0;
volatile int integral_error_alt = 0;

// ***********************  FUNCTIONS **************************

/* Initialise the PWM modules and pins */
void initPWM(void)
{
    // Initialise Main PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);                         // Enables PWM Module 0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);                        // Enables GPIO Port C peripheral
    GPIOPinConfigure(GPIO_PC5_M0PWM7);                                  // Links pin PC5 to Module 0, PWM 7
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);                        // Configures pin type to be PWM
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3,                               // Configures PWM module properties
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);                                 // Enables PWM on PWM Module 0, Generator 3
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);                    // Sets output state to false - no output until true

    // Initialise Tail PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);                         // Enables PWM Module 1 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                        // Enables GPIO Port F peripheral
    GPIOPinConfigure(GPIO_PF1_M1PWM5);                                  // Links pin PF1 to Module 1, PWM 5
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);                        // Configures pin type to be PWM
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2,                               // Configures PWM module properties
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);                                 // Enables PWM on PWM Module 1, Generator 2
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);                    // Sets output state to false - no output until true
}

/* Sets Duty Cycles for main and tail rotor */
void setPWM(void)
{
    uint32_t Period = SysCtlClockGet() / PWM_FIXED_FREQ;                // Equates to 80000 clock cycles = 0.004s = (1/250)Hz

    // MAIN
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, Period);                      // Sets PWM period
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, Period * MAIN_DUTY / 100);   // Sets pulse width (Duty Cycle)

    // TAIL
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, Period);                      // Sets PWM period
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, Period * TAIL_DUTY / 100);   // Sets pulse width (Duty Cycle)
}

/* Controls helicopter when in 'FLYING' mode */
void flightController(int Yaw, int8_t Alt)
{
    int time_step = SysCtlClockGet() / 1000;                            // 1ms time step

    IntMasterDisable();                                                 // Disable interrupts to prevent values changing while being read
    double yaw_error = (TARGET_YAW - Yaw);                              // Difference between desired and actual yaw
    double alt_error = (TARGET_ALT - Alt);                              // Difference between desired and actual percentage altitude
    IntMasterEnable();                                                  // Re-enable interrupts

    // ********** GAINS **********
    double Kp_T = 1.3;                                                  // Tail proportional gain
    double Ki_T = 0.0001;                                               // Tail integral gain
    double Kd_T = 1.6;                                                  // Tail derivative gain

    double Kp_M = 1;                                                    // Main proportional gain
    double Ki_M = 0.009;                                                // Main integral gain
    double Kd_M = 2.3;                                                  // Main derivative gain

    // ********** CONTROLLERS **********
    double alt_control = 30 + (Kp_M * alt_error)                        // This will become the new duty cycle of the main rotor
            - (Kd_M * (prev_error_alt - alt_error) / time_step)
            + (Ki_M * integral_error_alt);
    double yaw_control = (alt_control * 40) / 50 + (Kp_T * yaw_error)   // This will become the new duty cycle of the tail rotor
            - (Kd_T * (prev_error_yaw - yaw_error) / time_step)
            + (Ki_T * integral_error_yaw);

    prev_error_yaw = yaw_error;                                         // Stores yaw error to be used in next derivative iteration of controller
    prev_error_alt = alt_error;                                         // Stores alt error to be used in next derivative iteration of controller

    integral_error_yaw += yaw_error;                                    // Sums up the total yaw error over the entire flight
    integral_error_alt += alt_error;                                    // Sums up the total alt error over the entire flight

    if (yaw_error != 0)                                                 // If yaw error exists, update tail Duty Cycle
    {
        TAIL_DUTY = yaw_control;
    }

    if (TAIL_DUTY >= PWM_DUTY_MAX_T)                                    // If tail Duty Cycle is out of range, update to be limit
    {
        TAIL_DUTY = PWM_DUTY_MAX_T;
    }
    else if (TAIL_DUTY <= PWM_DUTY_MIN_T)
    {
        TAIL_DUTY = PWM_DUTY_MIN_T;
    }

    if (alt_error != 0)                                                 // If alt error exists, update main Duty Cycle
    {
        MAIN_DUTY = alt_control;
    }

    if (MAIN_DUTY >= PWM_DUTY_MAX_M)                                    // If main Duty Cycle is out of range, update to be limit
    {
        MAIN_DUTY = PWM_DUTY_MAX_M;
    }
    else if (MAIN_DUTY <= PWM_DUTY_MIN_M)
    {
        MAIN_DUTY = PWM_DUTY_MIN_M;
    }

    setPWM();                                                           // Set main and tail Duty Cycles

    SysCtlDelay(time_step);                                             // Wait for 1ms
}

/* Controls helicopter when in 'LANDING' mode */
void landingController(int Yaw, int8_t Alt)
{
    TARGET_ALT = 0;                                                     // Sets desired alt to be 0 (%)
    TARGET_YAW = 0;                                                     // Sets desired yaw to be 0 (deg)

    int time_step = SysCtlClockGet() / 1000;                            // 1ms time step

    IntMasterDisable();                                                 // Disable interrupts to prevent them from changing while being read
    double yaw_error = (TARGET_YAW - Yaw);                              // Difference between desired and actual yaw
    double alt_error = (TARGET_ALT - Alt);                              // Difference between desired and actual alt
    IntMasterEnable();                                                  // Re-enable interrupts

    // ********** GAINS **********
    double Kp_T = 1;                                                    // Tail proportional gain
    double Ki_T = 0.01;                                                 // Tail integral gain
    double Kd_T = 2.4;                                                  // Tail derivative gain

    double Kp_M = 1;                                                    // Main proportional gain
    double Ki_M = 0.0009;                                               // Main integral gain
    double Kd_M = 0.4;                                                  // Main derivative gain

    // ********** CONTROLLERS **********
    double yaw_control = (Kp_T * yaw_error)                             // Becomes the new tail Duty Cycle
            - (Kd_T * (prev_error_yaw - yaw_error) / time_step)
            + (Ki_T * integral_error_yaw);

    double alt_control = (Kp_M * alt_error)                             // Becomes the new main Duty Cycle
            - (Kd_M * (prev_error_alt - alt_error) / time_step)
            + (Ki_M * integral_error_alt);

    prev_error_yaw = yaw_error;                                         // Stores yaw error to be used in next derivative iteration of controller
    prev_error_alt = alt_error;                                         // Stores alt error to be used in next derivative iteration of controller

    integral_error_yaw += yaw_error;                                    // Sums up the total yaw error over the entire flight
    integral_error_alt += alt_error;                                    // Sums up the total alt error over the entire flight

    if (yaw_error != 0)                                                 // If yaw error exists, update tail Duty Cycle
    {
        TAIL_DUTY = yaw_control;
    }

    if (TAIL_DUTY >= PWM_DUTY_MAX_T)                                    // If tail Duty Cycle out of range, set to limits
    {
        TAIL_DUTY = PWM_DUTY_MAX_T;
    }
    else if (TAIL_DUTY <= PWM_DUTY_MIN_T)
    {
        TAIL_DUTY = PWM_DUTY_MIN_T;
    }

    if (alt_error != 0)                                                 // If altitude error exists, update main Duty Cycle
    {
        MAIN_DUTY = alt_control;
    }

    if (MAIN_DUTY >= PWM_DUTY_MAX_M)                                    // If main Duty Cycle out of range, set to limits
    {
        MAIN_DUTY = PWM_DUTY_MAX_M;
    }
    else if (MAIN_DUTY <= PWM_DUTY_MIN_M_L)
    {
        MAIN_DUTY = PWM_DUTY_MIN_M_L;
    }

    setPWM();                                                           // Updates Duty Cycle

    SysCtlDelay(time_step);                                             // 1ms delay

}

/* Disables output on the PWM pins */
void disablePWM(void)
{
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);                    // Disables main PWM
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);                    // Disables tail PWM
}

/* Enables output on the PWM pins */
void enablePWM(void)
{
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);                     // Enables main PWM
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);                     // Enables tail PWM
}
