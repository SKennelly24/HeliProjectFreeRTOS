/**
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 * Starts with Simple LED blinking example for Tiva launchpad
 *  provided by Andre Renard
 * Then adds working and ported (for RTOS) modules from
 *  ENCE361-20S1 Fri_am_group 7 provided with permission of group
 *  memebers Jesse Shehan, Will Cowper, Manu Hamblyn
 *
 * mfb31 2020_07_28 Add display
 *
 * based on Simple LED blinking example for the Tiva Launchpad
 */

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
#include "utils.h"
#include "altitude.h"
#include "uart.h"
#include "pwm.h"
#include "yaw.h"
#include "buttons.h"

/*
 #
 #include "clock.h"
 #include "control.h"
 #include "config.h"

 #include "input.h"
 #include "kernel.h"

 #include "setpoint.h"
 #include "flight_mode.h"

 */

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"

#define QUEUE_SIZE 16

#define BUTTON_QUEUE_FREQ 50
#define ALITUDE_MEAS_FREQ 10
#define YAW_MEAS_FREQ 10
#define CHECK_QUEUE_FREQ 10

// Global constants .. bad but needed
/**
 * The amount of time to display the splash screen (in seconds)
 */
static const uint32_t SPLASH_SCREEN_WAIT_TIME = 3;

static QueueHandle_t g_buttonQueue;
static SemaphoreHandle_t g_buttonMutex;
static SemaphoreHandle_t g_changeStateMutex;

static int32_t g_altitudeReference;
static int32_t g_yawReference;
static int8_t g_heliState;

typedef enum HELI_STATE
{
    LANDED = 0, TAKEOFF, FLYING, LANDING,
} HELI_STATE;



/*
 * Initiates the altitude measurement,
 * Gets the current height
 */
void GetAltitude(void *pvParameters)
{
    while (1)
    {
        alt_process_adc();
        int32_t height = alt_update();
        vTaskDelay(1 / (ALITUDE_MEAS_FREQ * portTICK_RATE_MS)); //  Current frequency is
    }
    // No way to kill this task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}


/*
 * Initialise the button queue
 */
void initButtonQueue(void)
{
    g_buttonQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
    g_buttonMutex = xSemaphoreCreateMutex();

}


/*
 * Check the button state if it has been pushed then
 * Queue it if it has been or in the case of the switch released aswell
 */
void CheckQueueButton(uint8_t button)
{
    //printf("Checking button %d", button);
    uint8_t buttonState;
    buttonState = checkButton(button);
    if (buttonState == PUSHED || ((button == SW1) && button == RELEASED))
    {
        if (xSemaphoreTake(g_buttonMutex, (TickType_t) 10) == true) //Take mutex
        {
            xQueueSendToBack(g_buttonQueue, (void * ) &button, (TickType_t ) 0); //queue
            xSemaphoreGive(g_buttonMutex); //give mutex
        }
    }
}


/*
 * Updates the buttons,
 * If the helicopter is flying then queue the buttons,
 * otherwise queue the switch pushes
 */
void QueueButtonPushes(void *pvParameters)
{
    while (1)
    {
        updateButtons();
        if (g_heliState == FLYING)
        {
            CheckQueueButton(UP);
            CheckQueueButton(DOWN);
            CheckQueueButton(LEFT);
            CheckQueueButton(RIGHT);
        }
        else
        {
            CheckQueueButton(SW1);
        }
        vTaskDelay(1 / (BUTTON_QUEUE_FREQ * portTICK_RATE_MS));
    }
}


/*
 * Updates the altitude and yaw references given the button press
 */
void UpdateReferences(int8_t pressed_button)
{
    switch (pressed_button)
    {
    case UP:
        if (g_altitudeReference < (MAX_HEIGHT - 10))
        { //If not within 10% of max altitude
            g_altitudeReference = g_altitudeReference + 10;
        }
        else
        {
            g_altitudeReference = MAX_HEIGHT;
        }
        break;
    case DOWN:
        // Checks lower limits of altitude if down button is pressed
        if (g_altitudeReference > 10)
        {
            g_altitudeReference = g_altitudeReference - 10;
        }
        else
        {
            g_altitudeReference = 0;
        }
        break;
    case RIGHT:
        if (g_yawReference <= 164)
        {
            g_yawReference = g_yawReference + 15;
        }
        else
        {
            g_yawReference = -345 + g_yawReference;
        }
        break;
    case LEFT:
        if (g_yawReference >= -165)
        {
            g_yawReference = g_yawReference - 15;
        }
        else
        {
            g_yawReference = 345 + g_yawReference;
        }
        break;
    }
}


/*
 * Return the target yaw value in degrees
 */
int32_t get_yaw_target(void)
{
    return (g_yawReference); //might need to cast to ??
}

/*
 * Returns the target altitude value in percent
 */
int32_t get_altitude_target(void)
{
    return (g_altitudeReference); //might need to cast to ??
}

/*
 * Returns the helicopter current state (of FSM)
 */
int8_t get_state(void)
{
    return (g_heliState);
}

/*
 * Changes the helicopter state to the given state
 */
void changeState(int8_t state_num)
{
    if (xSemaphoreTake(g_changeStateMutex, (TickType_t) 10) == true) //Take mutex
    {
        g_heliState = state_num;
        xSemaphoreGive(g_changeStateMutex); //give mutex
    }
}


/* Given the pressed button from the queue
 * changes the state if the switch was pressed
 * otherwise updates the altitude and yaw references
 */
void ButtonUpdates(int8_t pressed_button)
{
    if (pressed_button == SW1)
    {
        switch (g_heliState)
        {
        case FLYING:
            changeState(LANDING);
        case LANDED:
            changeState(TAKEOFF);
        }
    }
    else
    {
        if (g_heliState == FLYING) // do you need the mutex for equality?
        {
            UpdateReferences(pressed_button);
        }
    }
}


/*
 * Checks if there is anything on the button queue
 * if there has been either changes the state or references
 */
void CheckButtonQueue(void *pvParameters)
{
    int8_t pressed_button = -1;
    while (1)
    {
        //printf("Checking Button Queue");
        if (xSemaphoreTake(g_buttonMutex, (TickType_t) 10) == true) //takes the mutex if it can
        {
            if ((uxQueueMessagesWaiting(g_buttonQueue)) > 0)
            {
                xQueueReceive(g_buttonQueue, &pressed_button, (TickType_t) 0);
                UpdateReferences(pressed_button);
            }
            xSemaphoreGive(g_buttonMutex); //Gives it back
        }
        vTaskDelay(1 / (CHECK_QUEUE_FREQ * portTICK_RATE_MS));
    }
}


/*
 * Initialises things for the FSM
 */
void initFSM(void)
{
    g_heliState = FLYING;
    g_altitudeReference = 10;
    g_yawReference = 10;
    g_changeStateMutex = xSemaphoreCreateMutex();
}

void flight_mode_update(void *pvParameters)
{
    // If state is TAKEOFF, find yaw reference, advance state,
    if (g_heliState == TAKEOFF)
    {
        if (1) //Check if this is the right conditionals g_referenceYaw = -1 && YAW != 0
        {
            // if we are in TAKE_OFF mode and both the yaw and altitude have been calibrated,
            // then we advance to FLYING mode

            changeState(FLYING);
            //UpdateReferences(pressed_button);
        }
        else
        {
            // if we are in TAKE_OFF mode and both of the yaw and altitude have not been calibrated,
            // then we turn off the main rotor and turn on the tail rotor
            pwm_set_main_duty(0);
            pwm_set_tail_duty(0);

            // the yaw reference will be calibrated via an interrupt
        }
    }
}

    // If state is LANDING, set yaw to zero, altitude to HOVER_ALTITUDE,
    //  once settled set altitude to zero.
    // Once settled at zero altitude, deactivate PID controls, reset
    //  calibration, set yaw and altitude setpoints to zero, advance state

/*
     if (g_mode == LANDING)
    {
        // Is current yaw within tolerance?
        if (yaw_is_settled_around(0))
        {
            if (alt_is_settled_around(0))
            {

                // if the angle is +/- 3 degrees of zero and our altitude is zero or lower,
                // then we reset the entire helicopter state and put it back in LANDED mode

                control_enable_yaw(false);
                control_enable_altitude(false);

                yaw_reset_calibration_state();
                alt_reset_calibration_state();

                setpoint_set_yaw(0);
                setpoint_set_altitude(0);

                flight_mode_advance_state();
            }
            else
            {
                // Is yaw and altitude settled for HOVER_ALTITUDE?
                if (alt_is_settled_around(HOVER_ALTITUDE)
                        && yaw_is_settled_around(0))
                {
                    // if the angle is +/- 3 degrees of zero and our altitude is around 5%,
                    // then we set the desired altitude to 0%
                    setpoint_set_altitude(0);
                }
                else
                {
                    if (setpoint_get_altitude() != 0)
                    {
                        // if the angle is +/- 3 degrees of zero and our altitude is not around 0% and
                        // our desired altitude has not been set to 0%,
                        // then we set our desired altitude to be 5%
                        setpoint_set_altitude (HOVER_ALTITUDE);
                    }
                }
            }
        }
        // NO, get to zero yaw
        else
        {
            // if the angle is not +/- 3 degrees of zero
            // then we set our desired angle to be 0 degrees
            setpoint_set_yaw(0);
        }
    }
}
*/

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

    //Initialisation to things for tasks
    alt_init();     // Altitude and ADC
    disp_init();    // Display
    uart_init();    // UART
    pwm_init();     // PWM (overwrites LED)
    initButtonQueue();
    initButtons();
    initFSM();
    initYaw();

    // Enable interrupts to the processor.
    IntMasterEnable();
}


/*
 * Creates all the FREERTOS tasks
 */
void createTasks(void)
{
    if (pdTRUE != xTaskCreate(GetAltitude, "Get Altitude", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE
            != xTaskCreate(disp_Values, "Display Update", 512, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(uart_update, "UART send", 512, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(QueueButtonPushes, "Queue Button Pushes", 32, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(CheckButtonQueue, "Check Button Queue", 32, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }
}


int main(void)
{
    initialise();

    // Render splash screen for a couple of seconds
    disp_calibration();
    utils_wait_for_seconds(SPLASH_SCREEN_WAIT_TIME);
    disp_advance_state();

    // Initialise tasks
    createTasks();
    vTaskStartScheduler();  // Start FreeRTOS!!

    // Should never get here since the RTOS should never "exit".
    while (1);
}
