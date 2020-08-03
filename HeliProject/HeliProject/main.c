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

/*
#
#include "clock.h"
#include "control.h"
#include "config.h"

#include "input.h"
#include "kernel.h"

#include "setpoint.h"
#include "flight_mode.h"
#include "yaw.h"
*/


// RTOS
#include "FreeRTOS.h"
#include "task.h"

// Global constants .. bad but needed
/**
 * The amount of time to display the splash screen (in seconds)
 */
static const uint32_t SPLASH_SCREEN_WAIT_TIME = 3;

/**
 * Enum for status of flight_mode
 * Finite State Machine
 */
enum flight_mode_state_e { LANDED,
                                  TAKE_OFF,
                                  IN_FLIGHT,
                                  LANDING };
typedef enum flight_mode_state_e FlightModeState;

// Blinky Red function
void BlinkRedLED(void *pvParameters)
{
    unsigned int whichLed = (unsigned int)pvParameters; /* While pvParameters is technically a pointer, a pointer is nothing
                                                         * more than an unsigned integer of size equal to the architecture's
                                                         * memory address bus width, which is 32-bits in ARM.  We're abusing
                                                         * the parameter then to hold a simple integer value.  Could also have
                                                         * used this as a pointer to a memory location holding the value, but
                                                         * our method uses less memory.
                                                         */


    const uint8_t whichBit = 1 << whichLed; // TivaWare GPIO calls require the pin# as a binary bitmask, not a simple number.
    // Alternately, we could have passed the bitmask into pvParameters instead of a simple number.

    uint8_t currentValue = 0;

    while (1) {
        currentValue ^= whichBit; // XOR keeps flipping the bit on / off alternately each time this runs.
        GPIOPinWrite(GPIO_PORTF_BASE, whichBit, currentValue);
        vTaskDelay(1000 / portTICK_RATE_MS);  // Suspend this task (so others may run) for 1000ms or as close as we can get with the current RTOS tick setting.
    }
    // No way to kill this blinky task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}


// Altitude task
void GetAltitude(void *pvParameters)
{

    while (1) {
        alt_process_adc();
        int32_t height = alt_update();
        vTaskDelay(100 / portTICK_RATE_MS);  //  Current frequency is
    }
    // No way to kill this blinky task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}

// OLED Display Updater task
void disp_Values(void *pvParameters)
{
    while (1) {
        char string[17];

        usnprintf(string, sizeof(string), "Main Duty: %4d%%", pwm_get_main_duty());
        //usnprintf(string, sizeof(string), "Main Duty: %4d%%", get_rand_percent());    // Test only
        OLEDStringDraw(string, 0, 0);

        usnprintf(string, sizeof(string), "Tail Duty: %4d%%", pwm_get_tail_duty());
        //usnprintf(string, sizeof(string), "Tail Duty: %4d%%", get_rand_percent());    // Test only
        OLEDStringDraw(string, 0, 1);

        //usnprintf(string, sizeof(string), "      Yaw: %4d%c", yaw_get(), DISP_SYMBOL_DEGREES);
        usnprintf(string, sizeof(string), "      Yaw: %4d%c", get_rand_yaw(), DISP_SYMBOL_DEGREES); // Test only
        OLEDStringDraw(string, 0, 2);

        //usnprintf(string, sizeof(string), " Altitude: %4d%%", alt_get());
        usnprintf(string, sizeof(string), " Altitude: %4d%%", alt_get());   // Test only
        OLEDStringDraw(string, 0, 3);

        vTaskDelay(2000 / portTICK_RATE_MS);  // Suspend this task (so others may run) for 1000ms or as close as we can get with the current RTOS tick setting.
        }
        // No way to kill this blinky task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}

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
            uint16_t actual_yaw = get_rand_yaw();

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

    // Enable interrupts to the processor.
    IntMasterEnable();

    // Render splash screen for a couple of seconds
    disp_render(NULL);
    utils_wait_for_seconds(SPLASH_SCREEN_WAIT_TIME);
    disp_advance_state();

    // Initialise tasks
    /*
     * Bye bye blinky, we have other tasks working, you are not needed anymore.
     * Blame PWM it trashed you.
     *
    if (pdTRUE != xTaskCreate(BlinkRedLED, "Blink Red", 32, (void *)1, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }
    */
    if (pdTRUE != xTaskCreate(GetAltitude, "Get Altitude", 128, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(disp_Values, "Display Update", 512, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }

    if (pdTRUE != xTaskCreate(uart_update, "UART send", 512, NULL, 4, NULL)) {
        while(1);   // Oh no! Must not have had enough memory to create the task.
    }


    vTaskStartScheduler();  // Start FreeRTOS!!

    // Should never get here since the RTOS should never "exit".
    while(1);
}
