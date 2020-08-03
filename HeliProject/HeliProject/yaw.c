/*******************************************************************************
 *
 * yaw.c
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
* ****************************************************************
 * QUAD_DECODE.c
 *
 * Module for software quadrature decoding using finite state machine
 * Helicopter rotates clockwise (yaw increases) & anticlockwise (yaw decreases)
 * Includes Finite State Machine, Initialiser and Degree conversion function
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/

#include <defines.h>
#include <stdint.h>
#include <stdbool.h>
#include <yaw.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "utils/ustdlib.h"

// ************************* GLOBALS *****************************************
int A_B = NULL;
int YAW = 0;
int yawDeg = 0;

// ********************** QUADRATURE DECODING FUNCTIONS **********************
/* Initialises the pins used for quadrature decoding */
void initQuadDecode(void)
{
    SysCtlPeripheralEnable(PHASE_AB_PERIPH);
    GPIOPinTypeQEI(PHASE_AB_PORT_BASE, PHASE_A_PIN | PHASE_B_PIN);      // Sets pin types to be Quad Decoding pins (Just makes Phase B HIGH = 2 instead of 1)

    GPIOIntRegister(PHASE_AB_PORT_BASE, QDIntHandler);                  // Sets QDIntHandler to be function to handle interrupt
    GPIOIntTypeSet(PHASE_AB_PORT_BASE, PHASE_A_INT_PIN,                 // Sets Phase A interrupt on both rising and falling edges
                   GPIO_BOTH_EDGES);
    GPIOIntTypeSet(PHASE_AB_PORT_BASE, PHASE_B_INT_PIN,                 // Sets Phase B interrupt on both rising and falling edges
                   GPIO_BOTH_EDGES);
    GPIOIntEnable(PHASE_AB_PORT_BASE, PHASE_A_INT_PIN                   // Enables interrupts
                  | PHASE_B_INT_PIN);
}

/* Calculates the current state */
int currentQuad(void)
{
    return GPIOPinRead(PHASE_AB_PORT_BASE, PHASE_A_PIN)
            - GPIOPinRead(PHASE_AB_PORT_BASE, PHASE_B_PIN);
}

/* Changes global yaw depending on current state of quad decode pins */
void QDIntHandler(void)
{

    if (A_B == STATE_A)
    {                                         // Phase 1: A = 0, B = 0
        if (currentQuad() == STATE_B)         // AntiClockwise
        {
            YAW--;
            A_B = STATE_B;
        }
        else if (currentQuad() == STATE_D)   // Clockwise
        {
            YAW++;
            A_B = STATE_D;
        }
    }
    else if (A_B == STATE_D)
    {                                        // Phase 2: A = 0, B = 2
        if (currentQuad() == STATE_A)        // AntiClockwise:  A = 0, B = 0
        {
            YAW--;
            A_B = STATE_A;
        }
        else if (currentQuad() == STATE_C)   // Clockwise:  A = 1, B = 2
        {
            YAW++;
            A_B = STATE_C;
        }
    }
    else if (A_B == STATE_C)
    {                                        // Phase 3: A = 1, B = 2
        if (currentQuad() == STATE_D)        // AntiClockwise: A = 0, B = 2
        {
            YAW--;
            A_B = STATE_D;
        }
        else if (currentQuad() == STATE_B)   // Clockwise: A = 1, B = 0
        {
            YAW++;
            A_B = STATE_B;
        }
    }
    else if (A_B == STATE_B)
    {                                        // Phase 4: A = 1, B = 0
        if (currentQuad() == STATE_C)        // AntiClockwise: A = 1, B = 2
        {
            YAW--;
            A_B = STATE_C;
        }
        else if (currentQuad() == STATE_A)   // Clockwise: A = 0, B = 0
        {
            YAW++;
            A_B = STATE_A;
        }
    }
    else
    {
        A_B = currentQuad();
    }

    if (YAW >= MAX_YAW || YAW <= MIN_YAW)
    {
        YAW = 0;
    }

    // Clears interrupts
    GPIOIntClear(PHASE_AB_PORT_BASE, PHASE_A_INT_PIN);
    GPIOIntClear(PHASE_AB_PORT_BASE, PHASE_B_INT_PIN);
    return;
}

/* Calculates yaw in degrees from slots in yaw disc */
int yawInDegrees(void)
{
    IntMasterDisable();
    yawDeg = YAW * 360 / MAX_YAW;
    IntMasterEnable();

    if (yawDeg >= 180)
    {
        yawDeg = (yawDeg - 360);
    }
    else if (yawDeg <= -180)
    {
        yawDeg = 360 + yawDeg;
    }

    return yawDeg;
}

// ****************************************************************************
