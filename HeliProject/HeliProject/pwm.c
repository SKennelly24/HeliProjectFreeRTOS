/*******************************************************************************
 *
 * pwmGen.c
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
 * -------------------------------------------------------------------------------
 * ENEL361 Helicopter Project
 * Friday Morning, Group 7
 *
 * 09/05/2019
 *
 * Written by:
 *  - Manu Hamblyn  <mfb31<@uclive.ac.nz>   95140875
 *  - Will Cowper   <wgc22@uclive.ac.nz>    81163265
 *  - Jesse Sheehan <jps111@uclive.ac.nz>   53366509
 *
 * Description:
 * This module contains prototypes for initialising PWM, changing duty cycle and
 * returning current duty cycle, for Main and Tail rotors.
 *
 * This module reuses code from pwmGen.c by P.J. Bones as
 * used in Lab3 ENCE361-19S1 =>
 * pwmGen.c - Example code which generates a single PWM
 *    output on J4-05 (M0PWM7) with duty cycle fixed and
 *    the frequency controlled by UP and DOWN buttons in
 *    the range 50 Hz to 400 Hz.
 *
 * P.J. Bones   UCECE
 * Last modified:  7.2.2018
 *
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

#include "pwm.h"
#include "utils.h"

//General PWM defines
#define PWM_RATE 200 //The PWM frequency for both the main and tail rotors.
#define PWM_DIVIDER_CODE SYSCTL_PWMDIV_16 // PWM divider code for a 80 MHz system clock.
#define PWM_DIVIDER 16 //The PWM divider for a 80 MHz system clock.

//PWM configuration for the main motor
#define PWM_MAIN_BASE PWM0_BASE //PWM Module 0
#define PWM_MAIN_GEN PWM_GEN_3 //PWM Generator 3
#define PWM_MAIN_OUTNUM PWM_OUT_7 //PWM output 7 (corresponds to generator 3)
#define PWM_MAIN_OUTBIT PWM_OUT_7_BIT //PWM Output 7
#define PWM_MAIN_PERIPH_PWM SYSCTL_PERIPH_PWM0 //PWM peripheral module 0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC //PWM GPIO peripheral 0
#define PWM_MAIN_GPIO_BASE GPIO_PORTC_BASE //PWM GPIO Port C
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7 //PWM GPIO Config for PC5 Module 0 PWM 7
#define PWM_MAIN_GPIO_PIN GPIO_PIN_5 //PWM GPIO Pin 5

//PWM configuration for the tail rotor
#define PWM_TAIL_BASE PWM1_BASE //PWM Module 1
#define PWM_TAIL_GEN PWM_GEN_2 //PWM Generator 2
#define PWM_TAIL_OUTNUM PWM_OUT_5 //PWM output 5 for generator 2
#define PWM_TAIL_OUTBIT PWM_OUT_5_BIT //PWM output bit 5 for generator 2
#define PWM_TAIL_PERIPH_PWM SYSCTL_PERIPH_PWM1 //PWM peripheral for PWM 1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF //PWM peripheral GPIO base F
#define PWM_TAIL_GPIO_BASE GPIO_PORTF_BASE //PWM GPIO base F
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5 //PWM module 1, pwm 5
#define PWM_TAIL_GPIO_PIN GPIO_PIN_1 //GPIO pin 1


static int8_t g_main_duty; //Stores the current duty cycle for the main rotor.
static int8_t g_tail_duty; //Stores the current duty cycle for the tail rotor.
static uint32_t g_pwm_period; // Stores the PWM period. This is used for some duty cycle calculations.


//-------------------------------------------------------------------------------

/*
 * Initialises the tail and main PWM
 */
void pwm_init(void)
{
    // setup the PWM period. It is based on the system clock
    g_pwm_period = SysCtlClockGet() / (PWM_DIVIDER * PWM_RATE);

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    //while (!SysCtlPeripheralReady(PWM_DIVIDER_CODE)); // busy-wait until ready // Halts here if enabled

    // initialise the main rotor
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);

    while (!SysCtlPeripheralReady(PWM_MAIN_PERIPH_PWM));
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    while (!SysCtlPeripheralReady(PWM_MAIN_PERIPH_GPIO));  // busy-wait until Main PWM and GPIO ready
    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);
    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, g_pwm_period);
    pwm_set_main_duty(0);                  // Set Main rotor PWM to 0 (i.e. off)
    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    // initialise the tail rotor
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);

    while (!SysCtlPeripheralReady(PWM_TAIL_PERIPH_PWM));
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    while (!SysCtlPeripheralReady(PWM_TAIL_PERIPH_GPIO));  // busy-wait until Tail PWM and GPIO ready
    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);
    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, g_pwm_period);
    pwm_set_tail_duty(0);                  // Set Tail rotor PWM to 0 (i.e. off)
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

/*
 * Sets up the main duty to the given value
 */
void pwm_set_main_duty(int8_t t_duty)
{
    g_main_duty = clamp(t_duty, 0, 100);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,g_pwm_period * g_main_duty / 100);
}

/*
 * Returns the current PWM duty
 */
int8_t pwm_get_main_duty(void)
{
    return g_main_duty;
}

/*
 * Sets the tail duty to the given value
 */
void pwm_set_tail_duty(int8_t t_duty)
{
    g_tail_duty = clamp(t_duty, 0, 100);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,g_pwm_period * g_tail_duty / 100);
}

/*
 * Returns the current PWM duty
 */
int8_t pwm_get_tail_duty(void)
{
    return g_tail_duty;
    //return get_rand_percent();    // Test only
}
