#ifndef QUAD_DECODE_H
#define QUAD_DECODE_H

/* ****************************************************************
 * QUAD_DECODE.h
 *
 * Header file for Software quadrature decoding using finite state machine
 * Helicopter rotates clockwise (yaw increases) & anticlockwise (yaw decreases)
 * Includes Finite State Machine, Initialiser and Degree conversion function
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/

// ************************ GLOBALS *******************************
extern int A_B;
extern int YAW;
extern int yawDeg;

// ********************** PROTOTYPES *****************************

// ****************************************************************
// initQuadDecode: Initialises the Peripherals, Ports & Pins
// associated with the Quadrature decoding function
void initQuadDecode(void);

// ****************************************************************
// currentQuad: Reads the difference in Pins and returns the value
int currentQuad(void);
// @return  Current state of the quad pins (which indictes which phase is leading)

// ****************************************************************
// QDIntHandler: The Finite state machine for quadrature decoding
void QDIntHandler(void);

// ****************************************************************
// yawInDegrees: Converts between Raw Yaw to Yaw in degrees
int yawInDegrees(void);
// @return  Current heli yaw in +/- degrees from reference point

#endif /*QUAD_DECODE_H*/
