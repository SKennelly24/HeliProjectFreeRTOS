// *******************************************************
// pidControl.h
//
// Functions to control the altitude and Yaw with a PID control on the main
// and tail duty
//
// Initially written by P.J. Bones UCECE
//
// Modified by
// - Sarah Kennelly
// - Derrick Edward
// - Manu Hamblyn
//
//*****************************************************************************

#ifndef PIDCONTROL_H_
#define PIDCONTROL_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct {
    double pGain;
    double iGain;
    double errorIntegrated;
}Controller;

void apply_control(void *pvParameters);

void set_yaw_target(int16_t new_yaw_target);
void set_altitude_target(uint8_t new_alt_target);

void set_PID_ON(void);
void set_PID_OFF(void);

#endif /* PIDCONTROL_H_ */
