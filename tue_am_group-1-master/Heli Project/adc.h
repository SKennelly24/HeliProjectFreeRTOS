//*****************************************************
//
// adc.c - Uses analog to digital conversion to take
// samples of the altitude and store in a circular.
// The height value can be calculated using the values
// from the buffer.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************
#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

//****************************************************************************
//Initializes the ADC
//****************************************************************************
void initADC(void);

//*****************************************************************************
// Function to set the current voltage value from the sensor or supply as a
// reference value using a method to find the average over the sampled voltages.
//*****************************************************************************
int32_t setZeroReference(void);

//****************************************************************************
//Given the sample mean ADC returns the altitude percent
//****************************************************************************
int32_t getAltitudePercent(int32_t sample_mean_adc);

//****************************************************************************
//Returns the current sample mean ADC
//****************************************************************************
int32_t getSampleMeanADC(void);

#endif /* ADC_H_ */
