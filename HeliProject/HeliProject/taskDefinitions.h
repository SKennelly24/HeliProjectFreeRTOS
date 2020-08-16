/**
 * taskDefinitions.h
 *
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 * Provides the functions to suspend, resume tasks and create them
 */

#ifndef TASKDEFINITIONS_H_
#define TASKDEFINITIONS_H_

#define BUTTON_QUEUE_FREQ 25
#define CHECK_QUEUE_FREQ 10
#define ALITUDE_MEAS_FREQ 10
#define CONTROL_RUN_FREQ 50
#define UART_FREQ 5
#define FSM_FREQ 50
#define DISPLAY_FREQ 5

/*
 * Creates all the tasks for FreeRTOS
 */
void createTasks(void);

/*
 * Suspends the PID control task
 */
void suspendPIDTask(void);

/*
 * Resumes the PID control task
 */
void startPIDTask(void);


#endif /* TASKDEFINITIONS_H_ */
