#ifndef YAW_H
#define YAW_H

/*******************************************************************************
 *
 * yaw.h
 *
 * Modified for ENCE464-20S2 Group 18
 * 2020_07_28 by:
 *  - Derrick Edward
 *  - Sarah Kennelley
 *  - Manu Hamblyn
 *
 * ****************************************************************
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
// ********************** PROTOTYPES *****************************
// ****************************************************************
// Initialises Yaw
// associated with the Quadrature decoding function
void initYaw(void);

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
