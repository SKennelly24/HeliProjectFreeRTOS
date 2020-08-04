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

/*
#
#include "clock.h"
#include "control.h"
#include "config.h"

#include "input.h"
#include "kernel.h"

#include "setpoint.h"
#include "flight_mode.h"

*/

// RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"

#define QUEUE_SIZE 16

// Global constants .. bad but needed
/**
 * The amount of time to display the splash screen (in seconds)
 */
static const uint32_t SPLASH_SCREEN_WAIT_TIME = 3;

QueueHandle_t g_button_queue;
SemaphoreHandle_t g_button_mutex;
int8_t g_last_press = -1;


// Altitude task
void GetAltitude(void *pvParameters)
{

    while (1) {
        alt_process_adc();
        int32_t height = alt_update();
        vTaskDelay(100 / portTICK_RATE_MS);  //  Current frequency is
    }
    // No way to kill this task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}

// Yaw task
void GetYaw(void *pvParameters)
{
    while (1) {
        QDIntHandler();
        int32_t yaw = yawInDegrees();
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void initButtonQueue(void)
{
    g_button_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
    g_button_mutex = xSemaphoreCreateMutex();

}

void CheckQueueButton(uint8_t button)
{
    //printf("Checking button %d", button);
    uint8_t buttonState;
    buttonState = checkButton(button);
    if (buttonState == PUSHED)
    {

        if (xSemaphoreTake(g_button_mutex, (TickType_t) 10) == true)
        {
            xQueueSendToBack(g_button_queue, (void *) &button, (TickType_t) 0);
            printf("Pushed and Queued %d", button);
            xSemaphoreGive(g_button_mutex);
        }
    }
}
void QueueButtonPushes(void *pvParameters)
{
    while(1){
        updateButtons();
        CheckQueueButton(UP);
        CheckQueueButton(DOWN);
        CheckQueueButton(LEFT);
        CheckQueueButton(RIGHT);
        vTaskDelay(20 / portTICK_RATE_MS);  //  Current frequency is 50Hz
    }
}

void CheckButtonQueue(void *pvParameters)
{
    int8_t pressed_button = -1;
    while(1) {
        //printf("Checking Button Queue");
        if (xSemaphoreTake(g_button_mutex, (TickType_t) 10) == true)
        {
            if ((uxQueueMessagesWaiting(g_button_queue)) > 0) //Not sure if you need the token for this
            {
                xQueueReceive(g_button_queue, &pressed_button, (TickType_t) 0);
                //printf("Pressed %d button", pressed_button);

            }
            xSemaphoreGive(g_button_mutex);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}


int main(void)
{
    // disable all interrupts
    IntMasterDisable();

    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_16MHZ);

    // For LED blinky task - initialize GPIO port F and then pin #1 (red) for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // activate internal bus clocking for GPIO port F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) ; // busy-wait until GPIOF's bus clock is ready

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1); // PF_1 as output
    // doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); // off by default

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // PF_2 as output
    // doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); // off by default

    //Initialisation
    alt_init();     // Altitude and ADC
    disp_init();    // Display
    uart_init();    // UART
    pwm_init();     // PWM (overwrites LED)
    initQuadDecode(); //Yaw
    initButtonQueue();
    initButtons();

    // Enable interrupts to the processor.
    IntMasterEnable();

    // Render splash screen for a couple of seconds
    disp_calibration();
    utils_wait_for_seconds(SPLASH_SCREEN_WAIT_TIME);
    disp_advance_state();

    // Initialise tasks
    if (pdTRUE != xTaskCreate(GetAltitude, "Get Altitude", 128, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(disp_Values, "Display Update", 512, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(uart_update, "UART send", 512, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(GetYaw, "Get Yaw", 128, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(QueueButtonPushes, "Queue Button Pushes", 32, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(CheckButtonQueue, "Check Button Queue", 32, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    vTaskStartScheduler();  // Start FreeRTOS!!

    // Should never get here since the RTOS should never "exit".
    while(1);
}
