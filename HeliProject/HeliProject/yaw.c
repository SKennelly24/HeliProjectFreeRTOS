/*******************************************************************************
 *
 * yaw.c
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelly
 *  - Manu Hamblyn
 *
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
#define YAW_GPIO_BASE       GPIO_PORTB_BASE //Sets the base for pins J1-03 (PB0, channel A) and J1-04 (PB1, channel B)
#define YAW_PIN0_GPIO_PIN   GPIO_INT_PIN_0
#define YAW_PIN1_GPIO_PIN   GPIO_INT_PIN_1

#define YAW_REFERENCE_BASE GPIO_PORTC_BASE
#define YAW_REFERENCE_PIN GPIO_INT_PIN_4

#define STATE_A             0                       // A = 0, B = 0
#define STATE_B             1                       // A = 1, B = 0
#define STATE_C             -1                      // A = 1, B = 2
#define STATE_D             -2                      // A = 0, B = 2

#define MAX_YAW             448                     // Number of slots in yaw disc * 4 - Equates to one full rotation CW
#define MIN_YAW             -448                    // Number of slots in yaw disc * 4 - Equates to one full rotation CCW

#define PHASE_AB_PERIPH     SYSCTL_PERIPH_GPIOB     // Peripheral For Both Phases
#define PHASE_AB_PORT_BASE  GPIO_PORTB_BASE         // Port Base For Both Phases
#define PHASE_A_PIN         GPIO_PIN_0              // Phase A Pin
#define PHASE_B_PIN         GPIO_PIN_1              // Phase B Pin
#define PHASE_A_INT_PIN     GPIO_INT_PIN_0          // Interrupt On Phase A Pin
#define PHASE_B_INT_PIN     GPIO_INT_PIN_1          // Interrupt On Phase B Pin

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
int32_t g_referenceYaw;


// ********************** QUADRATURE DECODING FUNCTIONS **********************
//
// Interrupt for to check if the helicopter has found the zero yaw reference
//
void referenceInterrupt(void)
{
    YAW = 0;
    GPIOIntClear(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
}


//********************************************************
// Initializes the quadrature decoders used to calculate the yaw
//********************************************************
void initReferenceYaw(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // YAW_GPIO_BASE holds the value for Port B base
    GPIOIntRegister(YAW_REFERENCE_BASE, referenceInterrupt);

    GPIOPinTypeGPIOInput(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);

    GPIOIntTypeSet(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN,GPIO_FALLING_EDGE);

    GPIOIntEnable(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
    g_referenceYaw = -1;
}


//********************************************************
// Initialization functions for the clock (incl. SysTick), ADC, display, quadrature.
//********************************************************
void initQuadratureGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // YAW_GPIO_BASE holds the value for Port B base
    GPIOIntRegister(YAW_GPIO_BASE, QDIntHandler);

    // YAW_PIN0_GPIO_PIN, YAW_PIN0_GPIO_PIN have the value for pin 0 and pin 1
    GPIOPinTypeGPIOInput(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN,GPIO_BOTH_EDGES);

    GPIOIntEnable(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    // set initial quadrature conditions
    YAW = GPIOPinRead(YAW_GPIO_BASE,
    YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);
}


void initYaw(void)
{
    initQuadratureGPIO();
    initReferenceYaw();
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
    int32_t yawDeg = YAW * 360 / MAX_YAW;
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
