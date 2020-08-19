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

//Frequencies of tasks
#define ALITUDE_MEAS_FREQ 50
#define CONTROL_RUN_FREQ 50
#define FSM_FREQ 20
#define BUTTON_QUEUE_FREQ 25
#define CHECK_QUEUE_FREQ 10
#define UART_FREQ 4
#define DISPLAY_FREQ 1

#define TICKS_IN_SECOND 1000

//Priority of tasks
#define QUEUE_BUTTON_PRIORITY 4
#define CHECK_QUEUE_PRIORITY 4
#define MEAS_ALTITUDE_PRIORITY 4
#define PID_CONTROL_PRIORITY 4
#define UART_PRIORITY 4
#define FSM_PRIORITY 4
#define DISPLAY_PRIORITY 4

//Stack size of tasks
#define UART_STACK_SIZE 512 //68 needed for normal, 352 needed for CPU load analysis
#define CHECK_QUEUE_STACK_SIZE 64 //uses 34 bytes
#define QUEUE_BUTTON_STACK_SIZE 64 //uses 46 bytes
#define DISPLAY_STACK_SIZE 64 //uses 39 bytes
#define ALTITUDE_STACK_SIZE 32 //uses 19 bytes
#define PID_STACK_SIZE 128 //uses 53 bytes
#define FSM_STACK_SIZE 64 //uses 19 bytes


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
