/*
 * rexAPI.c
 *
 *  Created on: Jan 26, 2016
 *      Author: Anton
 */

#include "rexAPI.h"



void getJoysticks(void *ignore)
{
	long unsigned int timeStart;
	while(1)
	{
		timeStart = millis();
		main.rightVertical = joystickGetAnalog(1,2);
		main.rightHorizontal = joystickGetAnalog(1,1);
		main.leftVertical = joystickGetAnalog(1,3);
		main.leftHorizontal = joystickGetAnalog(1,4);
		partner.rightVertical = joystickGetAnalog(2,2);
		partner.rightHorizontal = joystickGetAnalog(2,1);
		partner.leftVertical = joystickGetAnalog(2,3);
		partner.leftHorizontal = joystickGetAnalog(2,4);
		main.rightBumper.up = joystickGetDigital(1,6,JOY_UP);
		main.rightBumper.down = joystickGetDigital(1,6,JOY_DOWN);
		main.leftBumper.up = joystickGetDigital(1,5,JOY_UP);
		main.leftBumper.down = joystickGetDigital(1,5,JOY_DOWN);
		main.rightDpad.up = joystickGetDigital(1,8,JOY_UP);
		main.rightDpad.down = joystickGetDigital(1,8,JOY_DOWN);
		main.rightDpad.right = joystickGetDigital(1,8,JOY_RIGHT);
		main.rightDpad.left = joystickGetDigital(1,8,JOY_LEFT);
		main.leftDpad.up = joystickGetDigital(1,7,JOY_UP);
		main.leftDpad.down = joystickGetDigital(1,7,JOY_DOWN);
		main.leftDpad.right = joystickGetDigital(1,7,JOY_RIGHT);
		main.leftDpad.left = joystickGetDigital(1,7,JOY_LEFT);
		taskDelayUntil(&timeStart,20);
	}
}


void positionPIDControl(void *ignore)
{
	float error = 0;
	float integral = 0;
	float derivative = 0;
	float previousError = 0;
	unsigned long startTime = millis();
	unsigned long loopTime;
	float output = 0;
	pidParams params = *((pidParams*)ignore);

	if(params.timeOut>0)
	{
		while(millis() < startTime + params.timeOut)
		{
			loopTime = millis();
			error = params.target() - params.input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params.kI)
				integral = 50/params.kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output = (error*params.kP) + (integral*params.kI) + (derivative*params.kD);
			if(output>127)
			{
				output = 127;
			}
			foreach(int *motor, params.outputs)
			{
				motorSet(abs(*motor), output*(*motor/abs(*motor)));
			}
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	else
	{
		while(true)
		{
			loopTime = millis();
			error = params.target() - params.input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params.kI)
				integral = 50/params.kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output = (error*params.kP) + (integral*params.kI) + (derivative*params.kD);

			if(output>127)
			{
				output = 127;
			}

			foreach(int *motor, params.outputs)
			{
				motorSet(abs(*motor), output*(*motor/abs(*motor)));
			}
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
}


void velocityPIDControl( void *ignore)
{
	float error = 0;
	float integral = 0;
	float derivative = 0;
	float previousError = 0;
	unsigned long startTime = millis();
	unsigned long loopTime;
	float output = 0;
	pidParams params = *((pidParams*)ignore);

	if(params.timeOut>0)
	{
		while(millis() < startTime + params.timeOut)
		{
			loopTime = millis();
			error = params.target() - params.input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params.kI)
				integral = 50/params.kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output += (error*params.kP) + (integral*params.kI) + (derivative*params.kD);
			if(output>127)
			{
				output = 127;
			}
			foreach(int *motor, params.outputs)
			{
				motorSet(abs(*motor), output*(*motor/abs(*motor)));
			}
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	else
	{
		while(true)
		{
			loopTime = millis();
			error = params.target() - params.input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params.kI)
				integral = 50/params.kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output += (error*params.kP) + (integral*params.kI) + (derivative*params.kD);

			if(output>127)
			{
				output = 127;
			}

			foreach(int *motor, params.outputs)
			{
				motorSet(abs(*motor), output*(*motor/abs(*motor)));
			}
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
}

