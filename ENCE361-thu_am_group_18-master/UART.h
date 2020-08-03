#ifndef UART_H
#define UART_H

/* ****************************************************************
 * UART.h
 *
 * Header file for the Universal Asynchronous Receiver Transmitter
 * UART with 20-byte FIFO buffers for Tx
 * Includes Initialiser and Send function
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/

// ********************** GLOBALS *****************************
extern volatile uint8_t slowTick;

extern volatile int MAIN_DUTY;
extern volatile int TAIL_DUTY;
extern volatile int TARGET_ALT;
extern volatile int TARGET_YAW;
extern volatile int ENABLE;
extern volatile int MODE;

// ********************** PROTOTYPES *****************************
// ****************************************************************
// initUART: Initialises the Peripherals, Ports & Pins
// associated with the UART function
void initUART(void);

// ****************************************************************
// UARTSend: Transmit a string via UART0
void UARTSend(char *pucBuffer);

// ****************************************************************
// transmit: Transmit all helicopter information through UART
void transmit(int8_t height, int yaw);

#endif /*UART_H*/
