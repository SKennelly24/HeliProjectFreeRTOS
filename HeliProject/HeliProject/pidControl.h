// *******************************************************
// pidControl.h
//
// Header file for the PID control
// Sets the main and tail duty using a PID control
// to get to the desired yaw and altitude
//
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

/*
 * Task for setting the duty of the main and tail rotor,
 * to achieve the altitude and yaw references
 */
void apply_control(void *pvParameters);

/*
 * Sets the yaw target to the given value
 */

void set_yaw_target(int16_t new_yaw_target);
/*
 * Sets the altitude target to the given value
 */
void set_altitude_target(uint8_t new_alt_target);

/*
 * Resets the intergrated yaw error
 */
void reset_yaw_error(void);

#endif /* PIDCONTROL_H_ */
