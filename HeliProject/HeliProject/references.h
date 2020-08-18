/**
 * taskDefinitions.h
 *
 * ENCE464-20S2 Group 18 RTOS Heli project
 * Derrick Edward
 * Sarah Kennelley
 * Manu Hamblyn
 *
 * ---------------------------------------------------------------------------
 * Provides the function to set and get and change altitude and yaw references
 */

#ifndef REFERENCES_H_
#define REFERENCES_H_
/*
 * Initialises what is needed for references
 */
void initReferences(void);

/*
 * Sets the alititude reference
 */
void setAltitudeReference(int32_t new_altitude);
/*
 * Gets the current altitude reference
 */
int32_t getAltitudeReference(void);

/*
 * Sets the yaw reference
 */
bool setYawReference(int16_t new_yaw);
/*
 * Gets the current yaw reference
 */
int32_t getYawReference(void);

/*
 * Updates the altitude and yaw references given a pressed button
 */
void UpdateReferences(int8_t pressed_button);


#endif /* REFERENCES_H_ */
