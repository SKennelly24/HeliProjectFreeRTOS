#ifndef BUTTONS_H
#define BUTTONS_H

/* ****************************************************************
 * BUTTONS.h
 *
 * Header file for the buttons module
 * Supports buttons on the Tiva/Orbit.
 * Comprises of initialisers and button checks
 *
 * Author: P.J. Bones UCECE
 * Edited: Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/

#include <stdint.h>
#include <stdbool.h>

// ****************************************************************
// Constants
// ****************************************************************

extern volatile int TARGET_YAW;     // Link the external variable TARGET_YAW
extern volatile int TARGET_ALT;     // Link the external variable TARGET_ALT

// Debounce algorithm: A state machine is associated with each button.
// A state change occurs only after NUM_BUT_POLLS consecutive polls have
// read the pin in the opposite condition, before the state changes and
// a flag is set.  Set NUM_BUT_POLLS according to the polling rate.

// ****************************************************************
// initButtons: Initialise the variables associated with the set of buttons
// defined by the constants above.
void initButtons (void);

// ****************************************************************
// updateButtons: Function designed to be called regularly. It polls all
// buttons once and updates variables associated with the buttons if
// necessary.  It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
void updateButtons (void);

// ****************************************************************
// checkButton: Function returns the new button state if the button state
// (PUSHED or RELEASED) has changed since the last call, otherwise returns
// NO_CHANGE.  The argument butName should be one of constants in the
// enumeration butStates, excluding 'NUM_BUTS'. Safe under interrupt.
uint8_t checkButton (uint8_t butName);
// @param   butname - Name of the button to compare the state of (UP/DOWN/LEFT/RIGHT)

// ****************************************************************
// buttonsCheck: checks if buttons associated with altitude and yaw have
// been pushed and increments accordingly
void buttonsCheck(void);

#endif /*BUTTONS_H*/
