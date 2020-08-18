/*******************************************************************************
 *
 * uart.h
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
 * This file reference portions of code that written by P.J. Bones.
 * These portions are noted in the comments.
 *
 * Description:
 * This module contains prototypes that facilitate sending data via
 * USB UART to the host computer.
 *
 ******************************************************************************/

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

/**
 * (Original Code by P.J. Bones)
 * initialiseUSB_UART - 8 bits, 1 stop bit, no parity
 */
void uart_init(void);

/**
 * (Original Code by P.J. Bones)
 * Formats the string to send via UART and then sends it
 */
void uart_send(const char *t_buffer);

/**
 * Task prototype to transmit the heli status information via UART.
 */
void uart_update(void *pvParameters);

#endif /* UART_H_ */
