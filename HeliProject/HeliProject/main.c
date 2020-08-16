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
#include "pidControl.h"
//#include "control.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/timers.h"

#define QUEUE_SIZE 16

#define BUTTON_QUEUE_FREQ 25
#define ALITUDE_MEAS_FREQ 10
#define CHECK_QUEUE_FREQ 10
#define FSM_FREQ 50
#define TIME_BETWEEN_DOUBLE 1000
#define YAW_SETTLE_RANGE 10
#define YAW_CHANGE 15

// Global constants .. bad but needed
typedef enum HELI_STATE
{
    LANDED = 0,
    TAKEOFF,
    FLYING,
    LANDING,
    SPIN_360
} HELI_STATE;

typedef enum TIMER_STATE
{
    NOT_STARTED = 0,
    NOT_FINISHED
}TIMER_STATE;


static const uint32_t SPLASH_SCREEN_WAIT_TIME = 3;

static QueueHandle_t g_buttonQueue;
static SemaphoreHandle_t g_buttonMutex;

static SemaphoreHandle_t g_changeStateMutex;
static int8_t g_heliState = LANDED;

static SemaphoreHandle_t g_altitudeMutex;
static int32_t g_altitudeReference = 0;

static SemaphoreHandle_t g_yawMutex;
static int32_t g_yawReference = 0;

//Global timers
static SemaphoreHandle_t g_upTimerMutex;
static uint8_t g_upTimerFinished = NOT_STARTED;

static SemaphoreHandle_t g_rightTimerMutex;
static uint8_t g_rightTimerFinished = NOT_STARTED;

static TimerHandle_t upTimer;
static TimerHandle_t rightTimer;

TaskHandle_t PIDTaskHandle;

//0 no timer started, 1 timer started not finished yet, 2 timer started and finished need to queue button

/***********************************Prototypes*****************************************/
void FinishTimer(TimerHandle_t finishedTimer);

/***********************************Changing mutex values******************************/
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
 * Change the timer state to the given state
 */
void ChangeTimerState(uint8_t newState, SemaphoreHandle_t * timerMutex, uint8_t * timerState)
{
    if (xSemaphoreTake(*timerMutex, (TickType_t) 10) == true) //Take mutex
    {
        *timerState = newState;
        xSemaphoreGive(*timerMutex); //give mutex
    }
}

/*
 * Sets the yaw reference
 */
bool setYawReference(int16_t new_yaw)
{
    bool worked;
    if (xSemaphoreTake(g_yawMutex, (TickType_t) 10) == true) //Take mutex
    {
        g_yawReference = new_yaw;
        set_yaw_target( (int16_t) g_yawReference);
        xSemaphoreGive(g_yawMutex); //give mutex
        worked = true;
    } else {
        worked = false;
    }
    return worked;
}

/*
 * Queue the button which has been pushed
 */
void QueueButton(uint8_t button)
{
    if (xSemaphoreTake(g_buttonMutex, (TickType_t) 10) == true) //Take mutex
    {
        xQueueSendToBack(g_buttonQueue, (void * ) &button, (TickType_t ) 0); //queue
        xSemaphoreGive(g_buttonMutex); //give mutex
    }
}

/***********************************Initialises ******************************************/
/*
 * Initialise the button queue
 */
void initButtonQueue(void)
{
    g_buttonQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
    g_buttonMutex = xSemaphoreCreateMutex();
}
/*
 * Initialise timers for button presses
 */
void createTimers(void)
{
    g_upTimerMutex = xSemaphoreCreateMutex();
    upTimer = xTimerCreate("Up timer", TIME_BETWEEN_DOUBLE, pdFALSE, (void *) 0, FinishTimer);

    g_rightTimerMutex = xSemaphoreCreateMutex();
    rightTimer = xTimerCreate("Right timer", TIME_BETWEEN_DOUBLE, pdFALSE, (void *) 0, FinishTimer);
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
    createTimers();

    // Enable interrupts to the processor.
    IntMasterEnable();
}

/***********************************Tasks and helper functions************************************************/

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

//Button tasks and helper functions

/*
 * When the timer finishes this is called,
 * changes the timer state to ensure button is then queued
 */
void FinishTimer(TimerHandle_t finishedTimer)
{
    if (finishedTimer == upTimer ) {
        if (g_upTimerFinished != NOT_STARTED) {
            ChangeTimerState(NOT_STARTED, &g_upTimerMutex, &g_upTimerFinished);
        }
    } else {
        if (g_rightTimerFinished != NOT_STARTED) {
            ChangeTimerState(NOT_STARTED, &g_rightTimerMutex, &g_rightTimerFinished);
        }
    }

}

/*
 * Starts the timer
 */
void StartTimer(TimerHandle_t * startTimer, SemaphoreHandle_t * timerMutex, uint8_t * timerState)
{
    ChangeTimerState(NOT_FINISHED, timerMutex, timerState);
    if (xTimerStart(*startTimer, 0) != pdPASS)
    {
        //Couldn't start
    }

}

/*
 * Changes the state if button is double pressed,
 * Queues button if the button wasn't pressed twice,
 * Starts the timer if the button has just been pressed
 */
void ChangeUpButtonState(void)
{
    if ((g_upTimerFinished == NOT_FINISHED)) {
        setAltitudeReference(MAX_HEIGHT / 2);
        ChangeTimerState(NOT_STARTED, &g_upTimerMutex, &g_upTimerFinished);
    } else {
        QueueButton(UP);
        StartTimer(&upTimer, &g_upTimerMutex, &g_upTimerFinished);
    }
}

/*
 * Changes the state if button is double pressed,
 * Queues button if the button wasn't pressed twice,
 * Starts the timer if the button has just been pressed
 */
void ChangeRightButtonState(void)
{
    if ((g_rightTimerFinished == NOT_FINISHED)) {
        changeState(SPIN_360);
        ChangeTimerState(NOT_STARTED, &g_rightTimerMutex, &g_rightTimerFinished);
    } else {
        QueueButton(RIGHT);
        StartTimer(&rightTimer, &g_rightTimerMutex, &g_rightTimerFinished);
    }
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
    if ((button == UP) && (buttonState == PUSHED)) {
        ChangeUpButtonState();
    } else if ((button == RIGHT) && (buttonState == PUSHED)) {
        ChangeRightButtonState();
    }
    else if (buttonState == PUSHED || ((button == SW1) && (buttonState == RELEASED)))
    {
        QueueButton(button);
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
        vTaskDelay(1000 / (BUTTON_QUEUE_FREQ * portTICK_RATE_MS));
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
        if (g_yawReference < (359 - YAW_CHANGE))
        {
            setYawReference(g_yawReference + YAW_CHANGE);
        } else {
            setYawReference(g_yawReference + YAW_CHANGE - 360);
        }
        break;
    case LEFT:
        if (g_yawReference > YAW_CHANGE)
        {
            setYawReference(g_yawReference - YAW_CHANGE);
        } else {
            setYawReference(g_yawReference - YAW_CHANGE + 360);
        }
        break;
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
        vTaskDelay(1000 / (CHECK_QUEUE_FREQ * portTICK_RATE_MS));
    }
}

/*
 * Returns true if the given yaw is within in the settle range,
 * else return false if the yaw is not
 */
bool yawInSettleRange(int16_t currentYaw)
{
    bool inRange;
    int16_t upperBound = g_yawReference + YAW_SETTLE_RANGE;
    int16_t lowerBound = g_yawReference - YAW_SETTLE_RANGE;
    if ((currentYaw > lowerBound) && (currentYaw < upperBound))
    {
        inRange = true;
    } else {
        inRange = false;
    }
    return inRange;
}

/*
 * Sets the yaw reference to next spin target
 */
void setSpinTarget(currentYaw) {
    int16_t target = currentYaw + 90;
    if (target > -1)
    {
        setYawReference(target);
    } else {
        setYawReference(target + 360);
    }
}
/*
 * Sets the first yaw for the heli to spin back to,
 * sets the first target and four subsequent targets.
 *
 */
void spin360(void)
{
    int16_t currentYaw = getYaw();
    static int16_t firstYaw;
    static uint8_t targets_acquired;

    if (targets_acquired == 0)
    {
        firstYaw = g_yawReference;
        setSpinTarget(g_yawReference);
        targets_acquired++;
    } else if ((firstYaw == g_yawReference) && yawInSettleRange(currentYaw))
    {
           reset_yaw_error();
           changeState(FLYING);
    } else if ((targets_acquired == 3) && yawInSettleRange(currentYaw)) {
        setYawReference(firstYaw);
    }
    else if ((targets_acquired < 4) && yawInSettleRange(currentYaw))
    {
        setSpinTarget(g_yawReference);
        targets_acquired++;
    }
}

/*
 * First rotate to the yaw reference by setting main and tail,
 * then when altitude and yaw is calibrated put PID on and
 * set altitude reference 10, yaw reference to 0,
 * when these values have been reached move into flight mode
 */
void TakeOffSequence(void)
{
    if (alt_get() >= 10)
    {
        changeState(FLYING);
        setYawReference(0);
    }
    else if (yaw_has_been_calibrated() && alt_has_been_calibrated()) //Check if yaw and reference is calibrated
    {
        vTaskResume(PIDTaskHandle);
        setAltitudeReference(10);   //
        setYawReference(0);
    }
    else
    {
        //Set the rotors to move so it can find the yaw reference
        //Suggest pwm_main = % and tail = %
        vTaskSuspend(PIDTaskHandle);
        pwm_set_main_duty(25); //15
        pwm_set_tail_duty(5); //33

    }
}



/*
 * Set the altitude and yaw references to 0,
 * If they are met turn off the main and tail motors
 */
void LandingSequence(void)
{
    if (alt_get() == 0 && getYaw() == 0)
    {
        changeState(LANDED);
        vTaskSuspend(PIDTaskHandle);
        pwm_set_main_duty(0);
        pwm_set_tail_duty(0);
    }
    else
    {
        setAltitudeReference(0);
        setYawReference(0);
    }
}

/*
 * The task for the FSM
 */
void flight_mode_FSM(void *pvParameters)
{
    // If state is TAKEOFF, find yaw reference, advance state,
    while(1) {
        switch (g_heliState)
        {
        case (TAKEOFF):
            TakeOffSequence();
            break;
        case (LANDING):
            LandingSequence();
            break;
        case (FLYING):
            vTaskResume(PIDTaskHandle);
            break;
        case (LANDED):
            //Reset calibration on yaw and altitude
            alt_reset_calibration_state();
            yaw_reset_calibration_state();

            //Turn off PID and set the pwm_main and pwm_tail duty to zero
            vTaskSuspend(PIDTaskHandle);
            pwm_set_main_duty(0);
            pwm_set_tail_duty(0);
            break;
        case (SPIN_360):
            spin360();
            break;
        }
        vTaskDelay(1000 / (FSM_FREQ * portTICK_RATE_MS));
    }
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

    if (pdTRUE!= xTaskCreate(apply_control, "PID", 128, NULL, 4, &PIDTaskHandle))
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
            int16_t actual_yaw = getYaw();

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

