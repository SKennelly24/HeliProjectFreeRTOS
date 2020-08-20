/**
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 *----------------------------------------------
 * Main routine for heli control:
 *  calls initialisation routines,
 *  displays OLED splash screen,
 *  starts FreeRTOS.
 * ---------------------------------------------
 */

// Standard modules
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// Tiva / M4 modules
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "utils/ustdlib.h"

// Heli modules
#include "display.h"
#include "uart.h"
#include "pwm.h"
#include "buttonTasks.h"
#include "fsm.h"
#include "taskDefinitions.h"
#include "references.h"
#include "utils.h"

// RTOS modules
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/timers.h"

#define SPLASH_SCREEN_WAIT_TIME 3

/*
 * Initialises clock, interrupts
 * and everything for each task
 */
void initialise(void)
{
    // disable all interrupts
    IntMasterDisable();

    // Set the clock rate to 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    SYSCTL_XTAL_16MHZ);

    //Initialisation things for tasks         // Altitude and ADC
    disp_init();        // Display
    uart_init();        // UART
    pwm_init();         // PWM (overwrites LED)
    initButtonTasks();  //
    initFSM();          // Finite state machine          // Yaw system (state machine, counters, zero reference)
    initReferences();   // Set up Altitude, Yaw, Button & Queue references

    // Enable interrupts to the processor.
    IntMasterEnable();
}

int main(void)

{
    initialise();

    // Render splash screen for a few seconds
    disp_calibration();
    utils_wait_for_seconds(SPLASH_SCREEN_WAIT_TIME);
    disp_advance_state();

    // Initialise tasks
    createTasks();
    vTaskStartScheduler();  // Start FreeRTOS!!

    // Should never get here since the RTOS should never "exit".
    while (1);
}

