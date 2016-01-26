/*
 * rexAPI.h
 *
 *  Created on: Jan 26, 2016
 *      Author: Anton
 */
#include "main.h"

#ifndef REXAPI_H_
#define REXAPI_H_

int (*motors[10])( char );

char motorInputs[10];

/*
 * Main joystick.
 */
extern joy main;

/*
 * Partner joystick.
 */
extern joy partner;

/*
 * Structure that encompases all button groups on the joystick.
 */
typedef struct buttonGroup
{
	bool up;
	bool down;
	bool right;
	bool left;
}btnGrp;

/*
 * Structure that contains all the values returned from a joystick.
 */
typedef struct joystick
{
	int rightVertical;
	int rightHorizontal;
	int leftVertical;
	int leftHorizontal;
	btnGrp rightBumper;
	btnGrp leftBumper;
	btnGrp rightDpad;
	btnGrp leftDpad;
} joy;

/*
 * Function that should be run in a task in order to get joystick values constantly.
 * @param ignore Should be null.
 */
void getJoysticks(void *ignore);


#endif /* REXAPI_H_ */
