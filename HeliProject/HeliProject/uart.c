/*******************************************************************************
 *
 * uart.c
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
 * -------------------------------------------------------------------------------
 * ENEL361 Helicopter Project
 * Friday Morning, Group 7
 *
 * Written by:
 *  - Manu Hamblyn  <mfb31<@uclive.ac.nz>   95140875
 *  - Will Cowper   <wgc22@uclive.ac.nz>    81163265
 *  - Jesse Sheehan <jps111@uclive.ac.nz>   53366509
 *
 * This file contains portions of code that written by P.J. Bones. These portions are noted in the comments.
 *
 * Description:
 * This module contains functions and constants that facilitate sending data via
 * USB UART to the host computer.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"

// RTOS
#include "FreeRTOS.h"
#include "task.h"

//Heli
#include "altitude.h"
//#include "config.h"
//#include "flight_mode.h"
#include "pwm.h"
//#include "setpoint.h"
#include "uart.h"
#include "yaw.h"
#include "utils.h"

/**
 * Enum for status of flight_mode
 * Finite State Machine
 */
enum flight_mode_state_e { LANDED,
                                  TAKE_OFF,
                                  IN_FLIGHT,
                                  LANDING };
typedef enum flight_mode_state_e FlightModeState;

/**
 * Define hardware settings for the UART
 */
static const int UART_BAUD_RATE = 57600;
static const int UART_USB_BASE = UART0_BASE;
static const uint32_t UART_USB_PERIPH_UART = SYSCTL_PERIPH_UART0;
static const uint32_t UART_USB_PERIPH_GPIO = SYSCTL_PERIPH_GPIOA;
static const int UART_USB_GPIO_BASE = GPIO_PORTA_BASE;
static const int UART_USB_GPIO_PIN_RX = GPIO_PIN_0;
static const int UART_USB_GPIO_PIN_TX = GPIO_PIN_1;

// buffer settings
static const int UART_INPUT_BUFFER_SIZE = 40;   // Old implementation
static char *g_buffer = NULL;                   // Old implementation

void uart_init(void)
{
    g_buffer = malloc(UART_INPUT_BUFFER_SIZE);

    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    while (!SysCtlPeripheralReady(UART_USB_PERIPH_UART));
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    while (!SysCtlPeripheralReady(UART_USB_PERIPH_GPIO));

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), UART_BAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

/**
 * (Original Code by P.J. Bones)
 * Formats the string to send via UART and then sends it
 */
void uart_send(const char *t_buffer)
{
    // write the buffer out
    while (*t_buffer)
    {
        UARTCharPut(UART_USB_BASE, *t_buffer);
        t_buffer++;
    }
}

//moved task into main
/*
void uart_flight_data_update(KernelTask* t_task)
{
    uint16_t target_yaw = setpoint_get_yaw();
    uint16_t actual_yaw = yaw_get();

    int16_t target_altitude = setpoint_get_altitude();
    int16_t actual_altitude = alt_get();
    uint8_t main_rotor_duty = pwm_get_main_duty();
    uint8_t tail_rotor_duty = pwm_get_tail_duty();
    uint8_t operating_mode = flight_mode_get();

    // format the outgoing data
#if !CONFIG_DIRECT_CONTROL
    usprintf(g_buffer, "Y%u\ty%u\tA%d\ta%d\tm%u\tt%u\to%u\r\n", target_yaw, actual_yaw, target_altitude, actual_altitude, main_rotor_duty, tail_rotor_duty, operating_mode);
#else
    usprintf(g_buffer, "y%u\ta%d\tm%u\tt%u\to%u\r\n", actual_yaw, actual_altitude, main_rotor_duty, tail_rotor_duty, operating_mode);
#endif

    // send it
    uart_send(g_buffer);
}
*/

// UART sender task
void uart_update(void *pvParameters)
{
    //static const int UART_INPUT_BUFFER_SIZE = 40;
    /**
     * Buffer settings for UART
     */
    char g_buffer[40] = {0};        //fixed the buffer size to 40

    while (1) {
            // originals commented out and modified copies for test
            //uint16_t target_yaw = setpoint_get_yaw();
            uint16_t target_yaw = get_rand_yaw();
            //uint16_t actual_yaw = yaw_get();
            uint16_t actual_yaw = yawInDegrees();

            //int16_t target_altitude = setpoint_get_altitude();
            int16_t target_altitude = (int16_t) get_rand_percent();
            int16_t actual_altitude = (int16_t) alt_get();
            //uint8_t main_rotor_duty = pwm_get_main_duty();
            uint8_t main_rotor_duty = (int8_t) get_rand_percent();
            //uint8_t tail_rotor_duty = pwm_get_tail_duty();
            uint8_t tail_rotor_duty = (int8_t) get_rand_percent();
            //uint8_t operating_mode = flight_mode_get();
            uint8_t operating_mode = IN_FLIGHT;

// format the outgoing data
//#if !CONFIG_DIRECT_CONTROL
    usprintf(g_buffer, "Y%u\ty%u\tA%d\ta%d\tm%u\tt%u\to%u\r\n", target_yaw, actual_yaw, target_altitude, actual_altitude, main_rotor_duty, tail_rotor_duty, operating_mode);
//#else
//    usprintf(g_buffer, "y%u\ta%d\tm%u\tt%u\to%u\r\n", actual_yaw, actual_altitude, main_rotor_duty, tail_rotor_duty, operating_mode);
//#endif

    // send it
    uart_send(g_buffer);
    vTaskDelay(500 / portTICK_RATE_MS);  // Suspend this task (so others may run) for 500ms or as close as we can get with the current RTOS tick setting.
    }
}

/*
void uart_kernel_data_update(KernelTask* t_task)
{
    uint8_t num_tasks;
    KernelTask* kernel_tasks = kernel_get_tasks(&num_tasks);

    int i;
    for (i = 0; i < num_tasks; i++)
    {
        KernelTask task = kernel_tasks[i];
        usprintf(g_buffer, "%s,%u,%u,%u\t", task.name, task.duration_micros, task.period_micros, task.frequency);
        uart_send(g_buffer);

    }

    uart_send("\r\n");
}
*/
