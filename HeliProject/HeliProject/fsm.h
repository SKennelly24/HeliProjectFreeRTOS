/**
 * fsm.h
 *
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 *-------------------------------------------------------------
 * Provides the helper function and task prototypes for the FSM
 */

#ifndef FSM_H_
#define FSM_H_

#include "FreeRTOS.h"
#include "task.h"

/*
 * Definitions for the possible heli states
 */
typedef enum HELI_STATE
{
    LANDED = 0,
    TAKEOFF,
    FLYING,
    LANDING,
    SPIN_360
} HELI_STATE;


/*
 * Initialises the state of the FSM
 */
void initFSM(void);

/*
 * Changes the helicopter state to the given state
 */
void changeState(int8_t state_num);

/*
 * Returns the current state of the FSM
 */
uint8_t getState(void);

/*
 * FSM (Free RTOS) task
 */
void flight_mode_FSM(void *pvParameters);

#endif /* FSM_H_ */
