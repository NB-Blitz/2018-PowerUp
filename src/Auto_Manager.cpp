/*
 * Auto_Manager.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Sam
 */


#include "WPILib.h"
#include "Auto_Manager.hpp"

FRC::AutoManager::AutoManager()
{

}


/* Checks to see if the robot will colide with an object with a given direction and distance, then returns which direction to go
 * @parameters
 * double sensorTheta = Direction of sensor reading
 * double sensorDistance = Distance reading from sensor
 * double joyStickX = MecanumDrive x-axis command
 * double joyStickY = mecanumDrive y-axis command
 * double joyStickTheta = Direction of turn from mecanumDrive
 *
 * @returns
 * Boolean stating if current path will result in a collision or not
 */
bool FRC::AutoManager::checkColision(double sensorTheta, double sensorDistance, double joyStickX, double joyStickY, double joyStickTheta)
{
	if(joyStickY < 0 || joyStickY > 0)
	{
		if(sensorTheta == 0)
		{
			if(sensorDistance <= MINIMUM_DISTANCE)
			{
				return true;
			}
		}
	}

	return false;
}


int FRC::AutoManager::avoidColision(double sensorTheta, double sensorDistance[], double joyStickX, double joyStickY, double joyStickTheta)
{
	if(!checkColision(sensorTheta, sensorDistance[sensorTheta], joyStickX, joyStickY, joyStickTheta))
	{
		return 0;
	}
	else if(sensorDistance[90] > sensorDistance[-90])
	{
		return 1;
	}
	else if(sensorDistance[90] < sensorDistance[-90])
	{
		return 2;
	}
	else
	{
		return 3;
	}
}







