#ifndef PWM_H
#define PWM_H

/* ****************************************************************
 * PWM.h
 *
 * Header file for the PWM Functions
 * Comprises of initialisers, setting functions and controllers for
 * main and tail rotors
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/


extern volatile int TARGET_YAW;                                     // Target Yaw | L/R Buttons change this.
extern volatile int TARGET_ALT;                                     // Target Alt | U/D Buttons change this.

extern volatile int MAIN_DUTY;                                      // Initial Main Duty Cycle
extern volatile int TAIL_DUTY;                                      // Initial Tail Duty Cycle

// *********************** PROTOTYPES *****************************

// ****************************************************************
// initPWM: Initialises the Peripherals, Ports & Pins
// associated with the PWM function
void initPWM(void);

// ****************************************************************
// setPWM: Sets/updates the main and tail rotor Duty Cycles
void setPWM(void);

// ****************************************************************
// landingController: Controller for the helicopter when in 'FLYING' mode
void flightController(int Yaw, int8_t Alt);
// @param   Yaw - Current yaw of the helicopter, in +/- degrees from reference
// @param   Alt - Current alt of the helicopter, in % of max height

// ****************************************************************
// landingController: Controller for the helicopter when in 'LANDING' mode
void landingController(int Yaw, int8_t Alt);
// @param   Yaw - Current yaw of the helicopter, in +/- degrees from reference
// @param   Alt - Current alt of the helicopter, in % of max height

// ****************************************************************
// disablePWM: Disables the PWM function
void disablePWM(void);

// ****************************************************************
// enablePWM: Enables the PWM function
void enablePWM(void);

#endif /*PWM_H*/
