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

void load_cpu_stats(void *pvParameters)
{
    while (1)
    {
        static char runtime_stats_buffer[512];
        vTaskGetRunTimeStats(runtime_stats_buffer);
        //printf(runtime_stats_buffer);
        vTaskDelay(1000 / (LOAD_FREQ * portTICK_RATE_MS));
    }
}
/*
 * Creates all the FREERTOS tasks
 */
void createTasks(void)
{
    if (pdTRUE != xTaskCreate(GetAltitude, "Get Altitude", 128, NULL, MEAS_ALTITUDE_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(disp_Values, "Display Update", 128, NULL, DISPLAY_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(uart_update, "UART send", 512, NULL, UART_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(QueueButtonPushes, "Queue Button Pushes", 128, NULL, QUEUE_BUTTON_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(CheckButtonQueue, "Check Button Queue", 128, NULL, CHECK_QUEUE_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(apply_control, "PID", 128, NULL, PID_CONTROL_PRIORITY, &PIDTaskHandle))
    {
       while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE!= xTaskCreate(flight_mode_FSM, "FSM", 128, NULL, FSM_PRIORITY, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }

    /*if (pdTRUE!= xTaskCreate(load_cpu_stats, "CPU_LOAD_STATS", 128, NULL, 4, NULL))
    {
        while (1);   // Oh no! Must not have had enough memory to create the task.
    }*/

}



void suspendPIDTask(void)
{
    vTaskSuspend(PIDTaskHandle);
}

void startPIDTask(void)
{
    vTaskResume(PIDTaskHandle);
}
