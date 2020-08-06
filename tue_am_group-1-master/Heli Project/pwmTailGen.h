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
// pwmTailGen.h - This code was based off the pwmGen.c example
//      code. We have changed the code only generate a single
//      PWM signal on Tiva board pin J4-05 = PF1 (M1PWM5). This
//      is the same PWM output as the helicopter tail rotor.
//
//      This .h file contains all functions in pwmTailGen.c.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#include <stdint.h>
#include <stdbool.h>

//******************************************************
// initialisePWM for tail rotor.
// M0PWM7 (J4-05, PC5) is used for the main rotor motor.
//******************************************************
void
initialiseTailRotorPWM (void);

//******************************************************
// Function to set the freq, duty cycle of M0PWM7
//******************************************************
void
setTailPWM (uint32_t ui32Duty);

//******************************************************
// Turns on the tail rotor so a duty cycle can be passed to it
//******************************************************
void
turnOnTailPWM(void);
