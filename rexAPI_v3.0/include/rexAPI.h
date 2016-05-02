/*
 * rexAPI.h
 *
 *  Created on: Jan 26, 2016
 *      Author: Anton
 */


#ifndef REXAPI_H_
#define REXAPI_H_

#include <API.h>
#include "math.h"

/*
 * Standard motor refresh time
 */
#define MOTOR_REFRESH_TIME	20

#define ADD_HOLONOMIC_DRIVE	0
#define ADD_MOTOR_AXIS		1
#define ADD_PNEUMATIC_AXIS	2




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

/*
 * Helper for setMotorOutputFunction_lcd function. Handles lcd printing cases.
 */
#define OUTPUT_CASE_LCD(num, output_func) case num:\
		lcdPrint(uart1, 1, #output_func);\
		lcdPrint(uart1, 2,"<-   select   ->");\
		output = output_func;\
		break;

/*
 * Helper for setMotorOutputAxis_lcd function. Handles lcd printing cases.
 */
#define AXIS_CASE_LCD(num, output_axis) case num:\
		lcdPrint(uart1, 1, #output_axis);\
		lcdPrint(uart1, 2,"<-   select   ->");\
		axis = &output_axis;\
		break;

/*
 * Structure that encompases axis on the joystick.
 */
typedef struct axis
{
	int axisValue;
}axis;

/*
 * Structure that contains all the values returned from a joystick.
 */
typedef struct joystick
{
	axis rightVertical;
	axis rightHorizontal;
	axis leftVertical;
	axis leftHorizontal;
	axis rightBumper;
	axis leftBumper;
	axis rightDpad;
	axis leftDpad;
} joy;

/*
 * Structure containing the motor assignments for a holonomic drive.
 */
typedef struct holonomicDrive
{
	int RB;
	int LB;
	int RF;
	int LF;
}holonomicDrive;

/*
 *	Structure that contains the needed parameters for a motor powered motion axis.
 */
typedef struct motorAxisParameters
{
	int motorPort;
	axis *controlAxis;
	bool reversed;
}motorAxisParams;

/*
 *	Structure that contains the needed parameters for a pneumatic powered motion axis.
 */
typedef struct pneumaticAxisParameters
{
	bool defaultPosition;
	axis *controlAxis;
	unsigned long timeDelay;

}pneumaticAxisParams;

/*
 *	Structure that contains the needed parameters for a Holonomic Drivetrain
 */
typedef struct holonomicAxisParameters
{
	holonomicDrive drive;
	joy *controlJoystick;
}holonomicAxisParams;


/*
 * Definition of motion function type.
 */
typedef int (*motionFunction)(void*);

/*
 * List of motion Functions related to holonomic Drive
 */
motionFunction *holonomicFunctionList;

int holonomicFunctionListSize;

/*
 * List of motion Functions related to motor Drive
 */
motionFunction *motorFunctionList;

int motorFunctionListSize;

/*
 * List of motion Functions related to pneumatic Drive
 */
motionFunction *pneumaticFunctionList;

int pneumaticFunctionListSize;

/*
 * Array of motion functions
 */
motionFunction *motionAxes;

/*
 * number of motion axes.
 */
int axesNumber;

/*
 * Array of motion input structures
 */
void *motionInputs;

/*
 * Stores current motor outputs.
 */
int currentMotorOutputs[10];

/*
 * Function that should be run in a task in order to get joystick values constantly.
 * @param ignore Should be null.
 */
void getJoysticks(void *ignore);

/*
 * Main joystick.
 */
joy main;

/*
 * Partner joystick.
 */
joy partner;

/*
 * PID parameter structure.
 */
typedef struct pidParams_raw
{
	void *ignore;
	int	(*input)();
	int	(*target)();
	int timeOut;
	float kP;
	float kI;
	float kD;
	int *outputs;

}pidParams_raw;

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
	int *outputs;

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
void positionPIDControl(void *parameters);

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

/*
 * Positional version of a PID loop, make sure that input and target values are sensor positions
 * not changes in sensor position.
 *
 * @param input Function which returns the current sensor feedback value.
 * @param target Function which returns the current target value.
 * @param timeOut Time, in milliseconds, for which the pid will be active.
 * @param outputs Pointer to where output is stored.
 * @param kP Proportional constant.
 * @param kI Integral constant.
 * @param kD Derivative constant.
 */
void positionPIDControl_raw(void *parameters);

/*
 * Velocity version of a PID loop, make sure that input and target values are sensor velocities
 * not sensor positions.
 *
 * @param input Function which returns the current sensor feedback value .
 * @param target Function which returns the current target value.
 * @param timeOut Time, in milliseconds, for which the pid will be active.
 * @param outputs Pointer to value where output is stored.
 * @param kP Proportional constant.
 * @param kI Integral constant.
 * @param kD Derivative constant.
 */
void velocityPIDControl_raw( void *ignore);

/*
 * Function that returns the digital binary value of a digital button group
 *
 * @param joystick the joystick slot to check
 * @param axis one of 5, 6, 7, or 8 to request that button as labelled on the joystick
 */
int joystickGetDigitalAxis(unsigned char joystick, unsigned char axis);

/*
 * Adds Holonomic drive movement Axis
 */
void addHolonomicDrive();

/*
 * Adds Motor powered movement Axis
 */
void addMotorAxis();

/*
 * Adds pneumatic piston movement axis
 */
void addPneumaticAxis();

/*
 * Function run on first startup of robot
 */
void initRobotLcd();

/*
 * Saves motor settings to file with name "settings". Saves functions then axis.
 */
void saveMotorSettings();

/*
 * Loads motor settings.
 *
 * @return true if read succesfully, false otherwise.
 */
bool loadMotorSettings();

/*
 * Prompts user for reset
 *
 * @return true if reset requested false otherwise.
 */
bool checkReset();

/*
 * Resets all motor settings to NULL
 */
void resetMotorSettings();

/*
 * Sets motors to their currentMotorPower values.
 */
void setAllMotors();

/*
 * Sets currentMotorPower values to their respective power output functions.
 */
void setMotorPowers();

/*
 * Runs motors based on motor settings.
 */
void runMotors_op(void *parameters);


#endif /* REXAPI_H_ */
