/**
 * fsm.c
 *
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 *------------------------------------------------------------
 * Implements helper functions and tasks for the FSM
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
#include "pwm.h"
#include "yaw.h"
#include "references.h"
#include "fsm.h"
#include "taskDefinitions.h"
#include "pidControl.h"
#include "altitude.h"
#include "uart.h"

// RTOS modules
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/timers.h"

#define MAX_YAW 360
#define YAW_SETTLE_RANGE 10

static SemaphoreHandle_t g_changeStateMutex;
static int8_t g_heliState = LANDED;         //Set LANDED on initialisation

/***********************************Getter and setter for state******************************/
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
 * Returns the current state of the FSM
 */
uint8_t getState(void)
{
    return g_heliState;
}

/***********************************Initialises ******************************************/

/*
 * Initialises things for the FSM
 */
void initFSM(void)
{
    g_changeStateMutex = xSemaphoreCreateMutex();

}
/***********************************Helper functions for spin mode******************************************/
/*
 * Returns true if the given yaw is within in the settle range,
 * else return false if the yaw is not
 */
bool yawInSettleRange(int16_t currentYaw)
{
    bool inRange;
    int32_t yawReference = getYawReference();
    int16_t upperBound = yawReference + YAW_SETTLE_RANGE;
    int16_t lowerBound = yawReference - YAW_SETTLE_RANGE;
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
void setSpinTarget(int16_t currentYaw, int16_t add_amount) {
    int16_t target = currentYaw + add_amount;
    if (target > -1 && target < MAX_YAW)
    {
        setYawReference(target);
    } else if (target < 0){
        setYawReference(target + MAX_YAW);
    } else {
        setYawReference(target - MAX_YAW);
    }
}

/***********************************Functions for different modes******************************************/
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
    int32_t yawReference = getYawReference();

    if (targets_acquired == 0)
    {
        firstYaw = yawReference;
        setSpinTarget(firstYaw, 75);
        targets_acquired++;
    } else if ((firstYaw == yawReference) && yawInSettleRange(currentYaw))
    {
           reset_yaw_error();
           changeState(FLYING);
           targets_acquired = 0;
    } else if ((targets_acquired == 3) && yawInSettleRange(currentYaw)) {
        setYawReference(firstYaw);
    }
    else if ((targets_acquired < 4) && yawInSettleRange(currentYaw))
    {
        setSpinTarget(yawReference, 90);
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
        startPIDTask();
        setAltitudeReference(10);   //
        setYawReference(0);
    }
    else
    {
        //Set the rotors to move so it can find the yaw reference
        //Suggest pwm_main = % and tail = %
        suspendPIDTask();
        pwm_set_main_duty(25);      // R1 15, R2 6, R3 25, R4 25
        pwm_set_tail_duty(5);       // R1 5, R2 32, R3 5, R4 5

    }
}


/*
 * Set the altitude and yaw references to zero duty,
 * If they are met turn off main and tail rotors
 */
void LandingSequence(void)
{
    if (alt_get() == 0 && getYaw() == 0)
    {
        changeState(LANDED);
        suspendPIDTask();
        pwm_set_main_duty(0);
        pwm_set_tail_duty(0);
    }
    else
    {
        setAltitudeReference(0);
        setYawReference(0);
    }
}
/***********************************FSM task******************************************/

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
            startPIDTask();
            break;
        case (LANDED):
            //Reset calibration on yaw and altitude
            alt_reset_calibration_state();
            yaw_reset_calibration_state();

            //Turn off PID and set the pwm_main and pwm_tail duty to zero
            suspendPIDTask();
            pwm_set_main_duty(0);
            pwm_set_tail_duty(0);
            break;
        case (SPIN_360):
            spin360();
            break;
        }
        vTaskDelay(TICKS_IN_SECOND / (FSM_FREQ * portTICK_RATE_MS));
    }
}




