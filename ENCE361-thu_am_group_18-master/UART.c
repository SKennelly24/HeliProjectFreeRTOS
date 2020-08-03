/* ****************************************************************
 * UART.c
 *
 * Module for the Universal Asynchronous Receiver Transmitter
 * UART with 20-byte FIFO buffers for Tx
 * Includes Initialiser and Send function
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * Based off uartDemo.c - P.J. Bones, UCECE, 2018
 * ***************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "ustdlib.h"
#include "UART.h"
#include "DEFINES.h"

// ************************* GLOBALS *******************************
volatile uint8_t slowTick = false;
char serialString[MAX_STR_LEN + 1];

// *********************** UART FUNCTIONS **************************
/* Initialises UART */
void initUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);                        // Enables UART peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                        // Enables GPIO peripheral
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);          // Sets pin type to be UART
    GPIOPinConfigure(GPIO_PA0_U0RX);                                    // Sets PA0 to be serial recieve pin (not used)
    GPIOPinConfigure(GPIO_PA1_U0TX);                                    // Sets PA1 to be serial transmit pin

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), BAUD_RATE,        // Sets serial properties (baud rate, parity bit, stop bit, etc.)
    UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    UART_CONFIG_PAR_NONE);

    UARTFIFOEnable(UART0_BASE);                                         // Enables First-In-First-Out property
    UARTEnable(UART0_BASE);                                             // Enables UART
}

/* Transmits strings over serial */
void UARTSend(char *pucBuffer)
{
    while (*pucBuffer)
    {
        UARTCharPut(UART0_BASE, *pucBuffer);                            // Sends each char in the string individually
        pucBuffer++;
    }
}

/* Calls UARTSend with each line to transmit */
void transmit(int8_t height, int yaw)
{
    IntMasterDisable();                                                 // Disbales all interrupts to prevent global variables changing whilst being read
    slowTick = false;                                                   // Sets slowTick back to false

    // Prints the following strings in order:
    usnprintf(serialString, sizeof(serialString), "Yaw (deg): %d [%d]\r\n",
              yaw, TARGET_YAW);
    UARTSend(serialString);

    usnprintf(serialString, sizeof(serialString), "Alt: %d%c [%d]\r\n",
              height, '%', TARGET_ALT);
    UARTSend(serialString);

    usnprintf(serialString, sizeof(serialString), "M PWM: %d%c\r\n",
              MAIN_DUTY, '%');
    UARTSend(serialString);

    usnprintf(serialString, sizeof(serialString), "R PWM: %d%c\r\n",
              TAIL_DUTY, '%');
    UARTSend(serialString);

    if (MODE == LANDED){
        usnprintf(serialString, sizeof(serialString), "LANDED\r\n");
    }else if (MODE == FIND_REF){
        usnprintf(serialString, sizeof(serialString), "FINDING REF\r\n");
    }else if (MODE == FLYING){
        usnprintf(serialString, sizeof(serialString), "FLYING\r\n");
    }else if (MODE == LANDING){
        usnprintf(serialString, sizeof(serialString), "LANDING\r\n");
    }
    UARTSend(serialString);

    usnprintf(serialString, sizeof(serialString), "-----------------\r\n");
    UARTSend(serialString);
    IntMasterEnable();                                                  // Re-Enable interrupts
}
