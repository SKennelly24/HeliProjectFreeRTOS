// *******************************************************
// buttons.c
//
// Modified for ENCE464-20S2 Group 18
// 2020_07_28 by:
//  - Derrick Edward
//  - Sarah Kennelley
//  - Manu Hamblyn
//
// -----------------------------------------------------------------
// Support for a set of FOUR specific buttons on the Tiva/Orbit.
// ENCE361 sample code.
// The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
// LEFT and RIGHT on the Tiva.
//
// Initially written by P.J. Bones UCECE
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "inc/tm4c123gh6pm.h"  // Board specific defines (for PF0)
#include "buttons.h"

// *******************************************************
// Globals to module
// *******************************************************
static bool but_state[NUM_BUTS];    // Corresponds to the electrical state
static uint8_t but_count[NUM_BUTS];
static bool but_flag[NUM_BUTS];
static bool but_normal[NUM_BUTS];   // Corresponds to the electrical state

void initButtons(void)
{
    int i;

    // UP button (active HIGH)
    SysCtlPeripheralEnable(UP_BUT_PERIPH);
    GPIOPinTypeGPIOInput(UP_BUT_PORT_BASE, UP_BUT_PIN);
    GPIOPadConfigSet(UP_BUT_PORT_BASE, UP_BUT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    but_normal[UP] = UP_BUT_NORMAL;

    // DOWN button (active HIGH)
    SysCtlPeripheralEnable(DOWN_BUT_PERIPH);
    GPIOPinTypeGPIOInput(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN);
    GPIOPadConfigSet(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    but_normal[DOWN] = DOWN_BUT_NORMAL;

    // LEFT button (active LOW)
    SysCtlPeripheralEnable(LEFT_BUT_PERIPH);
    GPIOPinTypeGPIOInput(LEFT_BUT_PORT_BASE, LEFT_BUT_PIN);
    GPIOPadConfigSet(LEFT_BUT_PORT_BASE, LEFT_BUT_PIN, GPIO_STRENGTH_2MA,
    GPIO_PIN_TYPE_STD_WPU);
    but_normal[LEFT] = LEFT_BUT_NORMAL;

    // RIGHT button (active LOW)
    // Note that PF0 is one of a handful of GPIO pins that need to be
    // "unlocked" before they can be reconfigured.  This also requires
    //      #include "inc/tm4c123gh6pm.h"
    SysCtlPeripheralEnable(RIGHT_BUT_PERIPH);
    //---Unlock PF0 for the right button:
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= GPIO_PIN_0; //PF0 unlocked
    GPIO_PORTF_LOCK_R = GPIO_LOCK_M;
    GPIOPinTypeGPIOInput(RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN);
    GPIOPadConfigSet(RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN, GPIO_STRENGTH_2MA,
    GPIO_PIN_TYPE_STD_WPU);
    but_normal[RIGHT] = RIGHT_BUT_NORMAL;

    //SW1 Switch (active HIGH)
    SysCtlPeripheralEnable(SWITCH_1_PERIPH);
    GPIOPinTypeGPIOInput(SWITCH_1_PORT_BASE, SWITCH_1_PIN);
    GPIOPadConfigSet(SWITCH_1_PORT_BASE, SWITCH_1_PIN, GPIO_STRENGTH_2MA,
    GPIO_PIN_TYPE_STD_WPD);
    but_normal[SW1] = UP_BUT_NORMAL;

    for (i = 0; i < NUM_BUTS; i++)
    {
        but_state[i] = but_normal[i];
        but_count[i] = 0;
        but_flag[i] = false;
    }
}

void updateButtons(void)
{
    bool but_value[NUM_BUTS];
    int i;

    // Read the pins; true means HIGH, false means LOW
    but_value[UP] = (GPIOPinRead(UP_BUT_PORT_BASE, UP_BUT_PIN) == UP_BUT_PIN);
    but_value[DOWN] = (GPIOPinRead(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN) == DOWN_BUT_PIN);
    but_value[LEFT] = (GPIOPinRead(LEFT_BUT_PORT_BASE, LEFT_BUT_PIN) == LEFT_BUT_PIN);
    but_value[RIGHT] = (GPIOPinRead(RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN) == RIGHT_BUT_PIN);
    but_value[SW1] = (GPIOPinRead(SWITCH_1_PORT_BASE, SWITCH_1_PIN) == SWITCH_1_PIN);
    // Iterate through the buttons, updating button variables as required

    for (i = 0; i < NUM_BUTS; i++)
    {
        if (but_value[i] != but_state[i])
        {
            but_count[i]++;
            if (but_count[i] >= NUM_BUT_POLLS)
            {
                but_state[i] = but_value[i];
                but_flag[i] = true;    // Reset by call to checkButton()
                but_count[i] = 0;
            }
        }
        else
            but_count[i] = 0;
    }
}

uint8_t checkButton(uint8_t butName)
{
    if (but_flag[butName])
    {
        but_flag[butName] = false;
        if (but_state[butName] == but_normal[butName])
            return RELEASED;
        else
            return PUSHED;
    }
    return NO_CHANGE;
}

