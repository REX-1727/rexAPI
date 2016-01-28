/*
 * rexAPI.c
 *
 *  Created on: Jan 26, 2016
 *      Author: Anton
 */

#include "rexAPI.h"

int joystickGetDigitalAxis(unsigned char joystick, unsigned char axis)
{
	return 	joystickGetDigital(joystick, axis, JOY_UP)*JOY_UP+
			joystickGetDigital(joystick, axis, JOY_DOWN)*JOY_DOWN+
			joystickGetDigital(joystick, axis, JOY_RIGHT)*JOY_RIGHT+
			joystickGetDigital(joystick, axis, JOY_LEFT)*JOY_LEFT;
}

void getJoysticks(void *ignore)
{
	long unsigned int timeStart;
	while(1)
	{
		timeStart = millis();
		main.rightVertical.axisValue 		= 	joystickGetAnalog(1,2);
		main.rightHorizontal.axisValue 		= 	joystickGetAnalog(1,1);
		main.leftVertical.axisValue 		= 	joystickGetAnalog(1,3);
		main.leftHorizontal.axisValue 		= 	joystickGetAnalog(1,4);
		partner.rightVertical.axisValue 	= 	joystickGetAnalog(2,2);
		partner.rightHorizontal.axisValue 	= 	joystickGetAnalog(2,1);
		partner.leftVertical.axisValue 		= 	joystickGetAnalog(2,3);
		partner.leftHorizontal.axisValue 	= 	joystickGetAnalog(2,4);
		main.rightBumper.axisValue 			= 	joystickGetDigitalAxis(1,6);
		main.leftBumper.axisValue 			= 	joystickGetDigitalAxis(1,5);
		main.rightDpad.axisValue 			= 	joystickGetDigitalAxis(1,8);
		main.leftDpad.axisValue 			= 	joystickGetDigitalAxis(1,7);
		taskDelayUntil(&timeStart,20);
	}
}

void positionPIDControl(void *parameters)
{
	float error = 0;
	float integral = 0;
	float derivative = 0;
	float previousError = 0;
	unsigned long startTime = millis();
	unsigned long loopTime;
	float output = 0;
	pidParams params = *((pidParams*)parameters);

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

void velocityPIDControl(void *parameters)
{
	float error = 0;
	float integral = 0;
	float derivative = 0;
	float previousError = 0;
	unsigned long startTime = millis();
	unsigned long loopTime;
	float output = 0;
	pidParams params = *((pidParams*)parameters);

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

motorOutput setMotorOutputFunction_lcd()
{
	int position = 1;
	motorOutput output;

	while(true)
	{
		switch(position)
		{
		OUTPUT_CASE_LCD(1, NULL)

		}

		if(lcdReadButtons(uart1) == 1)
		{
			position -= 1;
			delay(100);
		}

		else if(lcdReadButtons(uart1) == 2)
		{
			return output;
		}

		else if(lcdReadButtons(uart1) == 4)
		{
			position +=1;
			delay(100);
		}

		if(position < 1)
			position = 1;

		else if(position > OUTPUT_TYPES )
			position = OUTPUT_TYPES;
	}
	return NULL;
}

axis* setMotorOutputAxis_lcd()
{
	int position = 1;
	axis *axis;

	while(true)
	{
		switch(position)
		{
		AXIS_CASE_LCD(1, main.rightHorizontal)
								AXIS_CASE_LCD(2, main.rightVertical)
								AXIS_CASE_LCD(3, main.leftVertical)
								AXIS_CASE_LCD(4, main.leftHorizontal)
								AXIS_CASE_LCD(5, main.leftBumper)
								AXIS_CASE_LCD(6, main.rightBumper)
								AXIS_CASE_LCD(7, main.leftDpad)
								AXIS_CASE_LCD(8, main.rightDpad)
								AXIS_CASE_LCD(9, partner.rightHorizontal)
								AXIS_CASE_LCD(10, partner.rightVertical)
								AXIS_CASE_LCD(11, partner.leftVertical)
								AXIS_CASE_LCD(12, partner.leftHorizontal)
								AXIS_CASE_LCD(13, partner.leftBumper)
								AXIS_CASE_LCD(14, partner.rightBumper)
								AXIS_CASE_LCD(15, partner.leftDpad)
								AXIS_CASE_LCD(16, partner.rightDpad)
		}

		if(lcdReadButtons(uart1) == 1)
		{
			position -= 1;
			delay(100);
		}

		else if(lcdReadButtons(uart1) == 2)
		{
			return axis;
		}

		else if(lcdReadButtons(uart1) == 4)
		{
			position +=1;
			delay(100);
		}

		if(position < 1)
			position = 1;

		else if(position > 16 )
			position = 16;
	}
	return NULL;
}

void initializeMotors_lcd()
{
	motors[0] = setMotorOutputFunction_lcd();
	motors[1] = setMotorOutputFunction_lcd();
	motors[2] = setMotorOutputFunction_lcd();
	motors[3] = setMotorOutputFunction_lcd();
	motors[4] = setMotorOutputFunction_lcd();
	motors[5] = setMotorOutputFunction_lcd();
	motors[6] = setMotorOutputFunction_lcd();
	motors[7] = setMotorOutputFunction_lcd();
	motors[8] = setMotorOutputFunction_lcd();
	motors[9] = setMotorOutputFunction_lcd();

	motorInputs[0] = setMotorOutputAxis_lcd();
	motorInputs[1] = setMotorOutputAxis_lcd();
	motorInputs[2] = setMotorOutputAxis_lcd();
	motorInputs[3] = setMotorOutputAxis_lcd();
	motorInputs[4] = setMotorOutputAxis_lcd();
	motorInputs[5] = setMotorOutputAxis_lcd();
	motorInputs[6] = setMotorOutputAxis_lcd();
	motorInputs[7] = setMotorOutputAxis_lcd();
	motorInputs[8] = setMotorOutputAxis_lcd();
	motorInputs[9] = setMotorOutputAxis_lcd();

}

void saveMotorSettings()
{
	FILE *settings;
	settings = fopen("settings", "w");
	fwrite(&motors, sizeof(motors), 1, settings);
	fwrite(&motorInputs, sizeof(motorInputs), 1, settings);
	fclose(settings);
}

bool loadMotorSettings()
{
	FILE *settings;
	settings = fopen("settings", "r");
	if(settings != NULL)
	{
		fread(&motors, sizeof(motors), 1, settings);
		fread(&motorInputs, sizeof(motorInputs), 1, settings);
		fclose(settings);
		return true;
	}
	else
		return false;
}

bool checkReset()
{
	lcdPrint(uart1, 1, "reset?");
	lcdPrint(uart1, 2, "Y..............N");
	while(true)
	{
		if(lcdReadButtons(uart1) == LCD_BTN_LEFT)
			return true;
		else if(lcdReadButtons(uart1) == LCD_BTN_RIGHT)
			return false;
	}
	return false;
}

void resetMotorSettings()
{
	for(int i=0;i<10;i++)
	{
		motors[i] = NULL;
		motorInputs[i] = NULL;
	}

}

void setMotorPowers()
{
	for(int i; i<10; i++)
	{
		currentMotorOutputs[i] = motors[i](*motorInputs[i]);
	}
}

void setAllMotors()
{
	for(int i; i<10; i++)
	{
		motorSet(i,currentMotorOutputs[i]);
	}
}

void runMotors_op(void *parameters)
{
	while(true)
	{
		setMotorPowers();
		setAllMotors();
	}
}
