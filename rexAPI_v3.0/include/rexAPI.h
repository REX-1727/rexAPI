/*
 * rexAPI.h
 *
 *  Created on: Jan 26, 2016
 *      Author: Anton
 */
#include "main.h"

#ifndef REXAPI_H_
#define REXAPI_H_

#define MOTOR_REFRESH_TIME 20


/*
 * foreach implementation for item in array.
 */
#define foreach(item, array) \
		for(int keep = 1, \
				count = 0,\
				size = sizeof (array) / sizeof *(array); \
				keep && count != size; \
				keep = !keep, count++) \
				for(item = (array) + count; keep; keep = !keep)

int (*motors[10])( char );

char motorInputs[10];


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

/*
 * Main joystick.
 */
extern joy main;

/*
 * Partner joystick.
 */
extern joy partner;

/*
 * PID parameter structure.
 */
typedef struct pidParams
{
	void *ignore;
	int	(*input)();
	int	(*target)();
	int timeOut;
	float kP;
	float kI;
	float kD;
	int outputs[4];

}pidParams;

/*
 * Positional version of a PID loop, make sure that input and target values are sensor positions
 * not changes in sensor position.
 *
 * @param input Function which returns the current sensor feedback value.
 * @param target Function which returns the current target value.
 * @param timeOut Time, in milliseconds, for which the pid will be active.
 * @param outputs Array of output motors. If reversed, port number should be negative.
 * @param kP Proportional constant.
 * @param kI Integral constant.
 * @param kD Derivative constant.
 */
void positionPIDControl(void *ignore);

/*
 * Velocity version of a PID loop, make sure that input and target values are sensor velocities
 * not sensor positions.
 *
 * @param input Function which returns the current sensor feedback value .
 * @param target Function which returns the current target value.
 * @param timeOut Time, in milliseconds, for which the pid will be active.
 * @param outputs Array of output motors. If reversed, port number should be negative.
 * @param kP Proportional constant.
 * @param kI Integral constant.
 * @param kD Derivative constant.
 */
void velocityPIDControl( void *ignore);


#endif /* REXAPI_H_ */
