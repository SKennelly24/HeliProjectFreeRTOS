/* ****************************************************************
 * GPIO.c
 *
 * Module to configure GPIO elements
 * Configures Peripherals, Ports, and Pins
 * for Switches and Reset & Ref Pins
 *
 * Author: Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "ustdlib.h"
#include "GPIO.h"
#include "DEFINES.h"

/* Handles the reset interrupt */
void resetHandler(void)
{
    GPIOIntClear(RST_BUT_PORT_BASE, RST_BUT_PIN);                       // Clears the interrupt
    SysCtlReset();                                                      // Resets the TIVA Board
}

/* Initialises the virtual reset 'button' */
void initReset(void)
{
    SysCtlPeripheralEnable(RST_BUT_PERIPH);                             // Enables reset button peripheral

    GPIOPadConfigSet(RST_BUT_PORT_BASE, RST_BUT_PIN, GPIO_STRENGTH_2MA, // Configures pull-up properties of pin
                     GPIO_PIN_TYPE_STD_WPU);
    GPIODirModeSet(RST_BUT_PORT_BASE, RST_BUT_PIN, GPIO_DIR_MODE_IN);   // Sets pin type to be an input

    GPIOIntTypeSet(RST_BUT_PORT_BASE, RST_BUT_PIN, GPIO_FALLING_EDGE);  // Sets an interrupt to be triggered on a falling edge
    GPIOIntRegister(RST_BUT_PORT_BASE, resetHandler);                   // Sets interrupt handler
    GPIOIntEnable(RST_BUT_PORT_BASE, RST_BUT_PIN);                      // Enables interrupt

    IntEnable(RST_BUT_INT);                                             // Enables interrupt
}

/* Initialises Switch 1 */
void initSW1(void)
{
    SysCtlPeripheralEnable(SW1_PERIPH);                                 // Enables Switch peripheral
    GPIODirModeSet(SW1_PORT_BASE, SW1_PIN, GPIO_DIR_MODE_IN);           // Sets pin type to be an input
    GPIOPadConfigSet(SW1_PORT_BASE, SW1_PIN, GPIO_STRENGTH_2MA,         // Configures pull down properties
                     GPIO_PIN_TYPE_STD_WPD);
}

/* Initialises the yaw reference pin */
void initRefPin(void)
{
    SysCtlPeripheralEnable(REF_PERIPH);                                 // Enables reference pin peripheral
    GPIOPinTypeGPIOInput(REF_PORT_BASE, REF_PIN);                       // Sets pin type to be an input
}

/* Checks if the reference pin is active (Active LOW) */
int refPinActive(void)
{
    return !(GPIOPinRead(REF_PORT_BASE, REF_PIN));                      // Ref pin is active LOW, so when active, returns HIGH/'1', else LOW/'0'
}

/* Checks if Switch 1 is active (Active HIGH) */
int SW1Active(void)
{
    return GPIOPinRead(SW1_PORT_BASE, SW1_PIN);                         // SW1 is active HIGH, so when active, returns HIGH/'1', else LOW/'0'
}

/* Calls all initialise functions above */
void initGPIO(void)
{
    initReset();                                                        // Initialises Reset button
    initSW1();                                                          // Initialises Switch 1
    initRefPin();                                                       // Initialises reference pin
}
