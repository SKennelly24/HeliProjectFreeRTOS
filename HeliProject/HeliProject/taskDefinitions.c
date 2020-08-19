/**
 * taskDefinitions.c
 *
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 * Provides the functions to suspend, resume tasks and create them
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
#include "taskDefinitions.h"
#include "altitude.h"
#include "display.h"
#include "uart.h"
#include "buttonTasks.h"
#include "pidControl.h"
#include "fsm.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/timers.h"

TaskHandle_t PIDTaskHandle; //Hold handle so task can be resumed and suspended
/*
 * Creates all the FREERTOS tasks
 */
void createTasks(void)
{
    if (pdTRUE != xTaskCreate(GetAltitude, "Get Altitude", ALTITUDE_STACK_SIZE, NULL, MEAS_ALTITUDE_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(disp_Values, "Display Update", DISPLAY_STACK_SIZE, NULL, DISPLAY_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(uart_update, "UART send", UART_STACK_SIZE, NULL, UART_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(QueueButtonPushes, "Queue Button Pushes", QUEUE_BUTTON_STACK_SIZE, NULL, QUEUE_BUTTON_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(CheckButtonQueue, "Check Button Queue", CHECK_QUEUE_STACK_SIZE, NULL, CHECK_QUEUE_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(apply_control, "PID", PID_STACK_SIZE, NULL, PID_CONTROL_PRIORITY, &PIDTaskHandle))
    {
       while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(flight_mode_FSM, "FSM", FSM_STACK_SIZE, NULL, FSM_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

}


/*
 * Suspends the PID task
 */
void suspendPIDTask(void)
{
    vTaskSuspend(PIDTaskHandle);
}

/*
 * Starts the PID task
 */
void startPIDTask(void)
{
    vTaskResume(PIDTaskHandle);
}
