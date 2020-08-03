/* ****************************************************************
 * FINAL.c
 *
 * Top level program for the Helicopter project
 * Includes initialisations, main loop, and display function
 * of program
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  2.05.2019
 *
 * *****************************************************************/

// Include all of the required files/modules
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "QUAD_DECODE.h"
#include "UART.h"
#include "ADC.h"
#include "PWM.h"
#include "BUTTONS.h"
#include "GPIO.h"
#include "DEFINES.h"

// Initialise global variables
volatile int ENABLE = INITIAL;                                                  // Used to enable/disable the helicopter
volatile int MODE = LANDED;                                                     // Heli state: Landed/Finding Ref/Flying/Landing
volatile int REF_FOUND  = 0;                                                    // Indicates if the reference has been found
volatile int GROUNDED   = 1;                                                    // Indicates if the heli is grounded/landed


/* Handler for the system tick interrupt */
void SysTickIntHandler(void)
{
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);                                          // Triggers the ADC interrupt
    updateButtons();                                                            // Checks if any buttons have been pressed

    // Triggers a slow tick at 4Hz
    static uint8_t tickCount = 0;
    const uint8_t ticksPerSlow = SAMPLE_RATE_HZ
            / SLOWTICK_RATE_HZ;

    if (++tickCount >= ticksPerSlow)
    {  // Signal a slow tick
        tickCount = 0;
        slowTick = true;
    }
}

/* Initialise internal clock and set SysTickInterrupt*/
void initClock(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN          // Set clock frequency to 20MHz
                   | SYSCTL_XTAL_16MHZ);
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);                        // Set SysTick frequency
    SysTickIntRegister(SysTickIntHandler);                                      // Set SysTick interrupt handler
    SysTickIntEnable();                                                         // Enable interrupt
    SysTickEnable();                                                            // Enable SysTick
}

/* Initialise all modules */
void init(void)
{
    initClock();                                                                // Initialise the internal clock
    initButtons();                                                              // Initialise the UP/DOWN/LEFT/RIGHT buttons
    initCircBuf(&g_inBuffer, BUF_SIZE);                                         // Initialise the circular buffer
    initADC();                                                                  // Initialise the ADC module
    OLEDInitialise();                                                           // Initialise the OLED display on the Orbit Booster Thing
    initQuadDecode();                                                           // Initialise the pins/interrupts for quadrature decoding
    initUART();                                                                 // Initilaise the UART/serial communication module
    initPWM();                                                                  // Initialise pins/modules used for rotor PWM
    initGPIO();                                                                 // Initialise virtual reset button, yaw reference pin, and switch

    IntMasterEnable();                                                          // Enable interrupts to the processor.
}

/* Displays strings on the OLED Display of the Orbit Booster */
void display(int8_t percent)
{
    char string[17];                                                            // Initialise string of 16 characters + end char

    usnprintf(string, sizeof(string), "Alt = %3d%c      ", percent, '%');
    OLEDStringDraw(string, 0, 0);
    usnprintf(string, sizeof(string), "Yaw = %3d       ", yawInDegrees());
    OLEDStringDraw(string, 0, 1);
    usnprintf(string, sizeof(string), "Main = %3d%c     ", MAIN_DUTY, '%');
    OLEDStringDraw(string, 0, 2);
    usnprintf(string, sizeof(string), "Tail = %3d%c      ", TAIL_DUTY, '%');
    OLEDStringDraw(string, 0, 3);
}

/* Rotates the heli until yaw reference point is found */
void findRef(void){
    if (refPinActive()){                                                        // If reference point is found, set yaw = 0, enable heli
        REF_FOUND = 1;
        ENABLE = ENABLED;
        YAW = 0;

        MAIN_DUTY = 2;
        TAIL_DUTY = 2;
    } else {
        TAIL_DUTY = 25;
        MAIN_DUTY = 15;
        setPWM();
    }
}

//*****************************************************************************
// MAIN PROGRAM LOOP
//*****************************************************************************
int main(void)
{
    init();                                                                     // Initialize all modules

    int16_t ground_level = calculateMean();                                     // Sets ground level/reference
    int8_t altitude = percentageHeight(ground_level, calculateMean());          // Calculates the initial height percentage

    while (1)                                                                   // Main program loop
    {
        altitude = percentageHeight(ground_level, calculateMean());             // Calculates percentage height
        display(altitude);                                                      // Display info on Orbit OLED Display

        if (slowTick)
        {
            transmit(altitude, yawInDegrees());                                 // Serial UART output
        }

        if (MODE == LANDED)                                                     // If current mode is 'LANDED'
        {
            if (ENABLE == INITIAL && !SW1Active())                              // Ensures the heli mode switch has to be 'off' before heli can be enabled
            {
                ENABLE = DISABLED;
            }
            if (ENABLE == DISABLED && SW1Active())                              // If Switch 1 is active (HIGH), change state
            {
                enablePWM();                                                    // Enable PWM outputs
                if (REF_FOUND == 0)                                             // If yaw reference point has not been found, change state to FIND_REF
                {
                    MODE = FIND_REF;
                }
                else                                                            // If yaw reference point has been found, change state to flying
                {
                    MODE = FLYING;
                }
            }

        }

        if (MODE == FIND_REF)                                                   // If current mode is 'FIND_REF'
        {
            findRef();                                                          // Call to the findRef function
            setPWM();                                                           // Sets PWM signals
            if (REF_FOUND == 1)                                                 // If reference point has been found change state
            {
                if (SW1Active())                                                // If Switch 1 is still active (HIGH), change state to 'FLYING'
                {
                    MODE = FLYING;
                }
                else                                                            // If Switch 1 has been changed to LOW, change state to 'LANDING'
                {
                    MODE = LANDING;
                }
            }

        }

        if (MODE == FLYING)                                                     // If current state is 'FLYING'
        {
            if (SW1Active())                                                    // If Switch 1 is HIGH
            {
                buttonsCheck();                                                 // Checks button changes and updates target yaw and altitude accordingly
                flightController(yawInDegrees(), altitude);                     // Run PID controller function
            }
            else                                                                // If Switch 1 has been changed to LOW, change state to 'LANDING'
            {
                MODE = LANDING;
            }

        }

        if (MODE == LANDING)                                                    // If curent mode is 'LANDING'
        {
            if (altitude == 0 && yawInDegrees() == 0)                           // If reference position has been reached (0% Alt, 0 deg Yaw)
            {
                MODE = LANDED;                                                  // Change mode to 'LANDED'
                disablePWM();                                                   // Disable PWM outputs
                ENABLE = DISABLED;                                              // Change enable state to disabled
            }
            else                                                                // If ref position has not been reached
            {
                landingController(yawInDegrees(), altitude);                    // Run PID controller function
            }
        }
    }
}

