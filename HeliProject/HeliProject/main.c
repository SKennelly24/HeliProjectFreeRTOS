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
//#define YAW_MEAS_FREQ 10
#define CHECK_QUEUE_FREQ 10

// Global constants .. bad but needed
/**
 * The amount of time to display the splash screen (in seconds)
 */
static const uint32_t SPLASH_SCREEN_WAIT_TIME = 3;

static QueueHandle_t g_button_queue;
static SemaphoreHandle_t g_button_mutex;
static int32_t g_altitudeReference;
static int32_t g_yawReference;
static int8_t g_heliState;

typedef enum HELI_STATE
{
    LANDED = 0,
    TAKEOFF,
    FLYING,
    LANDING,
}HELI_STATE;


/*
 * Initiates the alitude measurement,
 * Gets the current height
 */
void GetAltitude(void *pvParameters)
{
    while (1) {
        alt_process_adc();
        int32_t height = alt_update();
        vTaskDelay(1 / (ALITUDE_MEAS_FREQ * portTICK_RATE_MS));  //  Current frequency is
    }
    // No way to kill this task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}

/*
 * Gets the current yaw by calling the interrupt handler?
 */
/*void GetYaw(void *pvParameters)
{
    while (1) {
        QDIntHandler();//Just calling the interrupt handler does not seem ok
        int32_t yaw = yawInDegrees();
        vTaskDelay(1 / (YAW_MEAS_FREQ * portTICK_RATE_MS));
    }
}*/
//Unneccessary task you only need to get the yaw when you need it it should all be dealt with in its own file

/*
 * Initialise the button queue
 */
void initButtonQueue(void)
{
    g_button_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
    g_button_mutex = xSemaphoreCreateMutex();

}

/*
 * Check the button state if it has been pushed then
 * queue it if it has been or in the case of the switch released aswell
 */
void CheckQueueButton(uint8_t button)
{
    //printf("Checking button %d", button);
    uint8_t buttonState;
    buttonState = checkButton(button);
    if (buttonState == PUSHED || ((button == SW1) && button == RELEASED))
    {
        if (xSemaphoreTake(g_button_mutex, (TickType_t) 10) == true) //Take mutex
        {
            xQueueSendToBack(g_button_queue, (void *) &button, (TickType_t) 0); //queue
            xSemaphoreGive(g_button_mutex); //give mutex
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
    while(1){
        updateButtons();
        if (g_heliState == FLYING) {
            CheckQueueButton(UP);
            CheckQueueButton(DOWN);
            CheckQueueButton(LEFT);
            CheckQueueButton(RIGHT);
        } else {
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
    switch(pressed_button) {
        case UP:
            if (g_altitudeReference < (MAX_HEIGHT - 10)) { //If not within 10% of max altitude
                g_altitudeReference = g_altitudeReference + 10;
            } else {
                g_altitudeReference = MAX_HEIGHT;
            }
            break;
        case DOWN:
            // Checks lower limits of altitude if down button is pressed
            if (g_altitudeReference > 10) {
               g_altitudeReference = g_altitudeReference - 10;
            } else {
               g_altitudeReference = 0;
            }
            break;
        case RIGHT:
            if (g_yawReference <= 164) {
               g_yawReference = g_yawReference + 15;
            } else {
               g_yawReference = -345 + g_yawReference;
            }
            break;
        case LEFT:
            if (g_yawReference >= -165) {
                g_yawReference = g_yawReference - 15;
            } else {
                g_yawReference = 345 + g_yawReference;
            }
            break;
    }
}

/*
 * Changes the helicopter state to the given state
 */
void changeState(int8_t state_num)
{
    g_heliState = state_num;
}

/* Given the pressed button from the queue
 * changes the state if the switch was pressed
 * otherwise updates the altitude and yaw references
 */
void ButtonUpdates(int8_t pressed_button)
{
    if (pressed_button == SW1)
    {
        switch(g_heliState) {
            case FLYING:
                changeState(LANDING);
            case LANDED:
                changeState(TAKEOFF);
        }
    } else {
        if (g_heliState == FLYING)
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
    while(1) {
        //printf("Checking Button Queue");
        if (xSemaphoreTake(g_button_mutex, (TickType_t) 10) == true) //takes the mutex if it can
        {
            if ((uxQueueMessagesWaiting(g_button_queue)) > 0)
            {
                xQueueReceive(g_button_queue, &pressed_button, (TickType_t) 0);
                UpdateReferences(pressed_button);

            }
            xSemaphoreGive(g_button_mutex); //Gives it back
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
    g_altitudeReference = 0;
    g_yawReference = 0;
}

/*
 * Initialises clock, interrupts
 * and everything for each task
 */
void initialise(void)
{
    // disable all interrupts
    IntMasterDisable();

    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_16MHZ);

    //Initialisation ot things for tasks
    alt_init();     // Altitude and ADC
    disp_init();    // Display
    uart_init();    // UART
    pwm_init();     // PWM (overwrites LED)
    initQuadDecode(); //Yaw
    initButtonQueue();
    initButtons();

    // Enable interrupts to the processor.
    IntMasterEnable();
}

/*
 * Creates all the FREERTOS tasks
 */
void createTasks(void)
{
    if (pdTRUE != xTaskCreate(GetAltitude, "Get Altitude", 128, NULL, 4, NULL)) {
       while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(disp_Values, "Display Update", 512, NULL, 4, NULL)) {
       while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(uart_update, "UART send", 512, NULL, 4, NULL)) {
       while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    /*if (pdTRUE != xTaskCreate(GetYaw, "Get Yaw", 128, NULL, 4, NULL)) {
       while(1);   // Oh no! Must not have had enough memory to create the task.
    }*/

    if (pdTRUE != xTaskCreate(QueueButtonPushes, "Queue Button Pushes", 32, NULL, 4, NULL)) {
       while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(CheckButtonQueue, "Check Button Queue", 32, NULL, 4, NULL)) {
       while(1);   // Oh no! Must not have had enough memory to create the task.
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
    while(1);
}
