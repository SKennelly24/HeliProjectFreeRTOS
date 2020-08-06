//*****************************************************
//
// clock.c - Sets up the clock and Systick interrupt.
// Functions getGCount and getSample count used allow global
// counters to be used in different files.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdint.h>

#define DISPLAY_RATE_HZ 2

//******************************************************
// The interrupt handler for the for SysTick interrupt.
//******************************************************
void SysTickIntHandler(void);

//******************************************************
//Initializes the clock
//******************************************************
void initClock(void);

//******************************************************
//Returns the current g_count
//******************************************************
uint32_t getGCount(void);

//******************************************************
//Returns the sample count
//******************************************************
uint32_t getSampCnt(void);

#endif /* CLOCK_H_ */
