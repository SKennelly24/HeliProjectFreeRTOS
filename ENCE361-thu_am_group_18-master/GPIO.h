#ifndef GPIO_H
#define GPIO_H

/* ****************************************************************
 * GPIO.h
 *
 * Header file for the gpio module
 * Configures Peripherals, Ports, and Pins
 * for Switches and Reset & Ref Pins
 *
 * Author: Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/
// ****************************************************************
// initReset: Initialises the virtual reset 'button' (PA6)
void initReset(void);

// ****************************************************************
// resetHandler: Handles the reset interrupt - Restarts board
void resetHandler(void);

// ****************************************************************
// initSW1: Initialises SW1 (PA6) on the Orbit Booster Thing
void initSW1(void);

// ****************************************************************
// initRefPin: Initialises Yaw Reference Pin (PC4)
void initRefPin(void);

// ****************************************************************
// refPinActive: Returns HIGH when PC4 is LOW, else LOW
int refPinActive(void);
// @return  !(State of pin PC4)

// ****************************************************************
// SW1Active: Returns HIGH when PA7 is HIGH, else LOW
int SW1Active(void);
// @return  State of pin PA7

// ****************************************************************
// initGPIO: Calls all init functions above
void initGPIO(void);

#endif /* GPIO_H */
