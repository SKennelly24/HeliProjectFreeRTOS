/****************************************************************
 * buttonTasks.h
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

#ifndef BUTTONTASKS_H_
#define BUTTONTASKS_H_

/*
 * Initialises everything needed for the button tasks
 */
void initButtonTasks(void);

/*
 * Updates the buttons,
 * If the helicopter is flying then queue the buttons,
 * otherwise queue the switch pushes
 */
void QueueButtonPushes(void *pvParameters);

/*
 * Checks if there is anything on the button queue
 * if there has been either changes the state or references
 */
void CheckButtonQueue(void *pvParameters);


#endif /* BUTTONTASKS_H_ */
