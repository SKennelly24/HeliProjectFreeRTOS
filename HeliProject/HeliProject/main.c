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
#include "control.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"

#define QUEUE_SIZE 16

#define BUTTON_QUEUE_FREQ 25
#define ALITUDE_MEAS_FREQ 10
#define CHECK_QUEUE_FREQ 10
#define FSM_FREQ 50

// Global constants .. bad but needed
typedef enum HELI_STATE
{
    LANDED = 0, TAKEOFF, FLYING, LANDING, HOVER
} HELI_STATE;



static const uint32_t SPLASH_SCREEN_WAIT_TIME = 3;

static QueueHandle_t g_buttonQueue;
static SemaphoreHandle_t g_buttonMutex;

static SemaphoreHandle_t g_changeStateMutex;
static int8_t g_heliState = LANDED;

static SemaphoreHandle_t g_altitudeMutex;
static int32_t g_altitudeReference = 0;

static SemaphoreHandle_t g_yawMutex;
static int32_t g_yawReference = 0;


/*
 * Initiates the altitude measurement,
 * Gets the current height
 */
void GetAltitude(void *pvParameters)
{
    while (1)
    {
        alt_process_adc();
        alt_update();
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
    if (buttonState == PUSHED || ((button == SW1) && (buttonState == RELEASED)))
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
        CheckQueueButton(SW1);
        if (g_heliState == FLYING)
        {
            CheckQueueButton(UP);
            CheckQueueButton(DOWN);
            CheckQueueButton(LEFT);
            CheckQueueButton(RIGHT);
        }
        vTaskDelay(1 / (BUTTON_QUEUE_FREQ * portTICK_RATE_MS));
    }
}

/*
 * Sets the altitude reference
 */
void setAltitudeReference(int32_t new_altitude)
{
    if (xSemaphoreTake(g_altitudeMutex, (TickType_t) 10) == true) //Take mutex
    {
        g_altitudeReference = new_altitude;
        set_altitude_target( (uint8_t) g_altitudeReference);
        xSemaphoreGive(g_altitudeMutex); //give mutex
    }
    //Set control -> altitude reference

}

/*
 * Sets the yaw reference
 */
void setYawReference(int32_t new_yaw)
{
    if (xSemaphoreTake(g_yawMutex, (TickType_t) 10) == true) //Take mutex
    {
        g_yawReference = new_yaw;
        set_yaw_target( (int16_t) g_yawReference);
        xSemaphoreGive(g_yawMutex); //give mutex
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

            setAltitudeReference(g_altitudeReference + 10);
        }
        else
        {
            setAltitudeReference(MAX_HEIGHT);
        }
        break;
    case DOWN:
        // Checks lower limits of altitude if down button is pressed
        if (g_altitudeReference > 10)
        {
            setAltitudeReference(g_altitudeReference - 10);
        }
        else
        {
            setAltitudeReference(0);
        }
        break;
    case RIGHT:
        if (g_yawReference <= 164)
        {
            setYawReference(g_yawReference + 15);
        }
        else
        {
            setYawReference(-345 + g_yawReference);
        }
        break;
    case LEFT:
        if (g_yawReference >= -165)
        {
            setYawReference(g_yawReference - 15);
        }
        else
        {
            setYawReference(345 + g_yawReference);
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
                break;
            case LANDED:
                changeState(TAKEOFF);
                break;
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
                ButtonUpdates(pressed_button);
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
    g_changeStateMutex = xSemaphoreCreateMutex();
    g_altitudeMutex = xSemaphoreCreateMutex();
    g_yawMutex = xSemaphoreCreateMutex();
}

void flight_mode_FSM(void *pvParameters)
{
    // If state is TAKEOFF, find yaw reference, advance state,
    while(1) {
        switch (g_heliState)
        {
        case (TAKEOFF):
            if (alt_get() >= 10)
            {
                changeState(FLYING);
                setYawReference(0);
            }
            else if (yaw_has_been_calibrated() && alt_has_been_calibrated()) //Check if yaw and reference is calibrated
            {
                set_PID_ON();               //
                setAltitudeReference(10);   //
                setYawReference(0);
            }
            else
            {
                //Set the rotors to move so it can find the yaw reference
                //Suggest pwm_main = % and tail = %
                pwm_set_main_duty(15);
                pwm_set_tail_duty(33);
            }
            break;
        case (LANDING):
            if (alt_get() == 0 && yawInDegrees() == 0)
            {
                changeState(LANDED);
                set_PID_OFF();
                pwm_set_main_duty(0);
                pwm_set_tail_duty(0);
                //
            }
            else
            {
                setAltitudeReference(0);
                setYawReference(0);
            }
            break;
        case (FLYING):
            //Turn on motors and do shit
            //flight controls active!
            break;
        case(HOVER):
            //Will setup new mode 07/08/2020
            setAltitudeReference(MAX_HEIGHT / 2);
            break;
        case (LANDED):
            alt_reset_calibration_state();
            yaw_reset_calibration_state();
            //Turn off the motors?
            //set the pwm_main and pwm_tail duty to zero
            //set the pwm_main and pwm_tail duty to zero
            set_PID_OFF();
            pwm_set_main_duty(0);
            pwm_set_tail_duty(0);
            break;

            vTaskDelay(1 / (FSM_FREQ * portTICK_RATE_MS));
        }
    }

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

    if (pdTRUE != xTaskCreate(disp_Values, "Display Update", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(uart_update, "UART send", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(QueueButtonPushes, "Queue Button Pushes", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(CheckButtonQueue, "Check Button Queue", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(control_update_altitude, "Altitude PID", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(control_update_yaw, "Yaw PID", 128, NULL, 4, NULL))
    {
       while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(flight_mode_FSM, "FSM", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

}

// UART sender task
void uart_update(void *pvParameters)
{
    //static const int UART_INPUT_BUFFER_SIZE = 40;
    /**
     * Buffer settings for UART
     */
    char g_buffer[100] = {0};        //fixed the buffer size to 40

    while (1) {
            // originals commented out and modified copies for test
            //uint16_t target_yaw = setpoint_get_yaw();
            int16_t target_yaw = g_yawReference;//get_rand_yaw();
            //uint16_t actual_yaw = yaw_get();
            int16_t actual_yaw = yawInDegrees();

            //int16_t target_altitude = setpoint_get_altitude();
            int16_t target_altitude = g_altitudeReference;//(int16_t) get_rand_percent();
            int16_t actual_altitude = (int16_t) alt_get();
            uint8_t main_rotor_duty = pwm_get_main_duty();
            //uint8_t main_rotor_duty = (int8_t) get_rand_percent();
            uint8_t tail_rotor_duty = pwm_get_tail_duty();
            //uint8_t tail_rotor_duty = (int8_t) get_rand_percent();
            uint8_t operating_mode = g_heliState;
            //uint8_t operating_mode = IN_FLIGHT;
            usprintf(g_buffer, "t_y:%d, y:%d, t_a:%d, a:%d, st:%d, m_d:%d, t_d:%d\r\n", target_yaw, actual_yaw, target_altitude, actual_altitude, operating_mode, main_rotor_duty, tail_rotor_duty);
            uart_send(g_buffer);
            vTaskDelay(250 / portTICK_RATE_MS);  // Suspend this task (so others may run) for 500ms or as close as we can get with the current RTOS tick setting.
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
