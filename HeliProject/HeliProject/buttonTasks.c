/**
 * buttonTasks.c
 *
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 * Provides the tasks and all the helper functions to deal with
 * the button presses and sets the yaw and altitude references
 * appropriately
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
#include "yaw.h"
#include "buttons.h"
#include "buttonTasks.h"
#include "fsm.h"
#include "altitude.h"
#include "taskDefinitions.h"
#include "references.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/timers.h"

#define QUEUE_SIZE 16
#define TIME_BETWEEN_DOUBLE 1000

typedef enum TIMER_STATE
{
    NOT_STARTED = 0,
    NOT_FINISHED
}TIMER_STATE;

/*****************************Globals to hold FREERtos queues and sempahores**************/
static QueueHandle_t g_buttonQueue;
static SemaphoreHandle_t g_buttonMutex;

//Global timers
static SemaphoreHandle_t g_upTimerMutex;
static uint8_t g_upTimerFinished = NOT_STARTED;

static SemaphoreHandle_t g_rightTimerMutex;
static uint8_t g_rightTimerFinished = NOT_STARTED;

static TimerHandle_t upTimer;
static TimerHandle_t rightTimer;

/****************************Prototypes****************************************/
void FinishTimer(TimerHandle_t finishedTimer);

/****************************Initialises values****************************************/
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

void initButtonTasks(void)
{
    initButtonQueue();
    createTimers();
}

/****************************Altering semaphore values****************************************/

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


/****************************Timer helper functions****************************************/

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


/****************************Button helper functions****************************************/
/*
 * Changes the state if button is double pressed,
 * Queues button if the button wasn't pressed twice,
 * Starts the timer if the button has just been pressed
 */
void ChangeUpButtonState(void)
{
    if ((g_upTimerFinished == NOT_FINISHED)) {
        ChangeTimerState(NOT_STARTED, &g_upTimerMutex, &g_upTimerFinished);
        setAltitudeReference(MAX_HEIGHT / 2);
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
        ChangeTimerState(NOT_STARTED, &g_rightTimerMutex, &g_rightTimerFinished);
        changeState(SPIN_360);
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




/* Given the pressed button from the queue
 * changes the state if the switch was pressed
 * otherwise updates the altitude and yaw references
 */
void ButtonUpdates(int8_t pressed_button)
{
    if (pressed_button == SW1)
    {
        switch (getState())
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
        if (getState() == FLYING) // do you need the mutex for equality?
        {
            UpdateReferences(pressed_button);
        }
    }
}

/****************************FreeRTOS Tasks****************************************/
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
        if (getState() == FLYING)
        {
            CheckQueueButton(UP);
            CheckQueueButton(DOWN);
            CheckQueueButton(LEFT);
            CheckQueueButton(RIGHT);
        }
        vTaskDelay(1000 / (BUTTON_QUEUE_FREQ * portTICK_RATE_MS));
    }
}
