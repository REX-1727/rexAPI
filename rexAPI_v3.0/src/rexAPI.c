/*
 * rexAPI.c
 *
 *  Created on: Jan 26, 2016
 *      Author: Anton
 */

#include "rexAPI.h"

int axesNumber = 0;

motionFunction *holonomicFunctionList;

int holonomicFunctionListSize;

motionFunction *motorFunctionList;

int motorFunctionListSize;

motionFunction *pneumaticFunctionList;

int pneumaticFunctionListSize;

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
	pidParams *params;
	params = (pidParams*)parameters;

	if(params->timeOut>0)
	{
		while(millis() < startTime + params->timeOut)
		{
			loopTime = millis();
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output = (error*params->kP) + (integral*params->kI) + (derivative*params->kD);
			if(output>127)
			{
				output = 127;
			}
			if(output<-127)
			{
				output = -127;
			}
			foreach(int *motor, params->outputs)
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
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output = (error*params->kP) + (integral*params->kI) + (derivative*params->kD);

			if(output>127)
			{
				output = 127;
			}
			if(output<-127)
			{
				output = -127;
			}

			foreach(int *motor, params->outputs)
			{
				motorSet(abs(*motor), output*(*motor/abs(*motor)));
			}
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	taskDelete(NULL);
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
	pidParams *params;
	params = (pidParams*)parameters;

	if(params->timeOut>0)
	{
		while(millis() < startTime + params->timeOut)
		{
			loopTime = millis();
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(abs(error) < 200)
				integral = 0;

			previousError = error;

			output += (error*params->kP) + (integral*params->kI) + (derivative*params->kD);
			if(output>127)
			{
				output = 127;
			}
			if(output<0)
			{
				output = 0;
			}
			foreach(int *motor, params->outputs)
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
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output += (error*params->kP) + (integral*params->kI) + (derivative*params->kD);

			if(output>127)
			{
				output = 127;
			}

			if(output<0)
			{
				output = 0;
			}

			foreach(int *motor, params->outputs)
			{
				if(params->target() == 0)
				{
					motorSet(abs(*motor),0);
					output = 0;
				}
				else
					motorSet(abs(*motor), output*(*motor/abs(*motor)));
			}
			//printf("%f\n\r",output);
			//lcdPrint(uart1,2,"%f",output);
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	taskDelete(NULL);
}

void positionPIDControl_raw(void *parameters)
{
	float error = 0;
	float integral = 0;
	float derivative = 0;
	float previousError = 0;
	unsigned long startTime = millis();
	unsigned long loopTime;
	float output = 0;
	pidParams_raw *params;
	params = (pidParams_raw*)parameters;

	if(params->timeOut>0)
	{
		while(millis() < startTime + params->timeOut)
		{
			loopTime = millis();
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output = (error*params->kP) + (integral*params->kI) + (derivative*params->kD);
			if(output>127)
			{
				output = 127;
			}
			if(output<-127)
			{
				output = -127;
			}

			*(params->outputs) = output;
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	else
	{
		while(true)
		{
			loopTime = millis();
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output = (error*params->kP) + (integral*params->kI) + (derivative*params->kD);

			if(output>127)
			{
				output = 127;
			}
			if(output<-127)
			{
				output = -127;
			}

			*(params->outputs) = output;
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	taskDelete(NULL);
}

void velocityPIDControl_raw(void *parameters)
{
	float error = 0;
	float integral = 0;
	float derivative = 0;
	float previousError = 0;
	unsigned long startTime = millis();
	unsigned long loopTime;
	float output = 0;
	pidParams *params;

	if(params->timeOut>0)
	{
		while(millis() < startTime + params->timeOut)
		{
			loopTime = millis();
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(abs(error) < 200)
				integral = 0;

			previousError = error;

			output += (error*params->kP) + (integral*params->kI) + (derivative*params->kD);
			if(output>127)
			{
				output = 127;
			}
			if(output<0)
			{
				output = 0;
			}
			*(params->outputs) = output;
			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	else
	{
		while(true)
		{
			loopTime = millis();
			error = params->target() - params->input();
			integral += error;
			derivative = error - previousError;

			if(integral > 50/params->kI)
				integral = 50/params->kI;

			if(error == 0)
				integral = 0;

			previousError = error;

			output += (error*params->kP) + (integral*params->kI) + (derivative*params->kD);

			if(output>127)
			{
				output = 127;
			}

			if(output<0)
			{
				output = 0;
			}

			*(params->outputs) = output;

			taskDelayUntil(&loopTime,MOTOR_REFRESH_TIME);
		}
	}
	taskDelete(NULL);
}

int selectMotorReversal(int motor)
{
	lcdPrint(uart1,1,"    reversed");
	lcdPrint(uart1,1,"y..............N");
	delay(500);
	while(true)
	{
		if(lcdReadButtons(uart1) == 1)
		{
			return motor;
		}
		else if(lcdReadButtons(uart1) == 4)
		{
			return -motor;
		}
	}
}

void selectHolonomicMotor(int *motor)
{
	int currentPos = 1;
	while(true)
	{
		lcdPrint(uart1,2,"       %d",currentPos);

		if(lcdReadButtons(uart1) == 1)
		{
			currentPos--;
			delay(500);
		}
		else if(lcdReadButtons(uart1) == 2)
		{
			*motor = selectMotorReversal(currentPos);
			delay(500);
			return;
		}
		else if(lcdReadButtons(uart1) == 4)
		{
			currentPos++;
			delay(500);
		}
		if(currentPos<1)
			currentPos = 1;
		if(currentPos>10)
			currentPos = 10;
	}
}

void selectHolonomicJoystick( joy *controlJoystick)
{
	lcdPrint(uart1,1,"Select Joystick");
	lcdPrint(uart1,2,"Main.....Partner");
	while(true)
	{
		if(lcdReadButtons(uart1) == 1)
		{
			controlJoystick = &main;
			return;
		}
		else if(lcdReadButtons(uart1) == 4)
		{
			controlJoystick = &partner;
			return;
		}
	}
}

motionFunction selectHolonomicFunction()
{

}

void addHolonomicDrive()
{
	lcdPrint(uart1,1,"RB Motor");
	axesNumber++;
	realloc(motionAxes,axesNumber*(sizeof(motionFunction)));
	realloc(motionInputs, axesNumber*(sizeof(void*)));
	holonomicAxisParams *holoParams = malloc(sizeof(holonomicAxisParams));
	delay(500);
	selectHolonomicMotor(&(holoParams->drive.RB));
	delay(500);
	lcdClear(uart1);
	lcdPrint(uart1,1,"LB Motor");
	selectHolonomicMotor(&(holoParams->drive.LB));
	delay(500);
	lcdClear(uart1);
	lcdPrint(uart1,1,"RF Motor");
	selectHolonomicMotor(&(holoParams->drive.RF));
	delay(500);
	lcdClear(uart1);
	lcdPrint(uart1,1,"LF Motor");
	selectHolonomicMotor(&(holoParams->drive.LF));
	delay(500);
	lcdClear(uart1);
	selectHolonomicJoystick(holoParams->controlJoystick);
	delay(500);
	lcdClear(uart1);


}

void addMotorAxis()
{

}

void addPneumaticAxis()
{

}

void initRobotLcd()
{
	int currentPos = 0;
	bool doneFlag = false;
	motionAxes = calloc(axesNumber,sizeof(motionFunction));
	motionInputs = calloc(axesNumber,sizeof(void*));
	while(!doneFlag)
	{
		switch(currentPos)
		{
		case ADD_HOLONOMIC_DRIVE:
			lcdPrint(uart1,1,"Add Holo");
			lcdPrint(uart1,2,"Drive");
			break;
		case ADD_MOTOR_AXIS:
			lcdPrint(uart1,1,"Add Motor");
			lcdPrint(uart1,2,"Axis");
			break;
		case ADD_PNEUMATIC_AXIS:
			lcdPrint(uart1,1,"Add Piston");
			lcdPrint(uart1,2,"Axis");
			break;
		}

		if(lcdReadButtons(uart1) == 1)
		{
			currentPos--;
			delay(500);
		}
		else if(lcdReadButtons(uart1) == 2)
		{
			switch(currentPos)
			{
			case ADD_HOLONOMIC_DRIVE:
				addHolonomicDrive();
				break;
			case ADD_MOTOR_AXIS:
				addMotorAxis();
				break;
			case ADD_PNEUMATIC_AXIS:
				addPneumaticAxis();
				break;
			}
		}
		else if(lcdReadButtons(uart1) == 4)
		{
			currentPos++;
			delay(500);
		}
		if(currentPos<0)
		{
			currentPos = 0;
		}
		if(currentPos>2)
		{
			currentPos = 2;
		}
	}
}

void saveMotorSettings()
{
	FILE *settings;
	settings = fopen("settings", "w");
	fwrite(&motionAxes, sizeof(motionAxes), 1, settings);
	fwrite(&motionInputs, sizeof(motionInputs), 1, settings);
	fclose(settings);
}

bool loadMotorSettings()
{
	FILE *settings;
	settings = fopen("settings", "r");
	if(settings != NULL)
	{
		fread(&motionAxes, sizeof(motionAxes), 1, settings);
		fread(&motionInputs, sizeof(motionInputs), 1, settings);
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
		motionAxes[i] = NULL;
		motionInputs[i] = NULL;
	}

}

void setMotorPowers()
{
	for(int i; i<10; i++)
	{
		currentMotorOutputs[i] = motionAxes[i](*motionInputs[i]);
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
