//*****************************************************
//
// pwmGen.c - Example code which generates a single PWM
//     output on J4-05 (M0PWM7) with duty cycle fixed and
//     the frequency controlled by UP and DOWN buttons in
//     the range 50 Hz to 400 Hz.
// 2017: Modified for Tiva and using straightforward, polled
//     button debouncing implemented in 'buttons4' module.
//
// P.J. Bones   UCECE
// Last modified:  7.2.2018
//
// pwmTailGen.c - This code was based off the pwmGen.c example
//      code. We have changed the code only generate a single
//      PWM signal on Tiva board pin J4-05 = PF1 (M1PWM5). This
//      is the same PWM output as the helicopter tail rotor.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#include "pwmTailGen.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

//******************************************************
// Constants
//******************************************************

// PWM configuration
#define PWM_TAIL_START_RATE_HZ  200
#define PWM_TAIL_FIXED_DUTY     0
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        1

//  PWM Hardware Details M1PWM5 (gen 3)
//  ---Tail Rotor PWM: PF1, J4-05
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

//******************************************************
// Function to set the freq, duty cycle of M0PWM7
//******************************************************
void
setTailPWM (uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / PWM_TAIL_START_RATE_HZ;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
        ui32Period * ui32Duty / 100);
}

//******************************************************
// initialisePWM for tail rotor.
// M0PWM7 (J4-05, PC5) is used for the main rotor motor.
//******************************************************
void
initialiseTailRotorPWM (void)
{
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setTailPWM(PWM_TAIL_FIXED_DUTY);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

//******************************************************
// Turns on the tail rotor so a duty cycle can be passed to it
//******************************************************
void
turnOnTailPWM(void)
{
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}





